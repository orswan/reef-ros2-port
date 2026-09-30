#!/usr/bin/env python3
"""Numerical parity: the ported REEF estimator vs the pinned original (P04/P05).

    check_port.py --port PATH/TO/reef_estimator_event_replay [--update-lock "<reason>"]
    check_port.py --port ... --stream PARAMS EVENTS LABEL     (one recorded stream only)

Default: for every P02 fixture (master, both parameter kinds) and every
vertical (v01-v10) and horizontal (h01-h10) fixture, the same event stream and
parameters go to the reference harness (build_reference.sh master) and to the
port's event driver. Compared per event (ACCEPTANCE.md 4c/4d):
- every continuous state field, Z and XY (state, full P, measurement, R, u,
  dt, g), within 1e-9 x max_run|ref field|, NaN/inf class equal;
- discrete fields exactly (flags, takeoff, counters, use_mocap_xy/z,
  delivered); gate values where the port evaluated a gate;
- the port against the committed P02 golden (decimated rows, all fields);
- wrapper equivalence: the events as ROS 2 messages through the node's
  callbacks (--mode node) give the core's state, and every published message
  carries exactly that state; fusion counts through the node = core;
- negatives: the port against the simulation revision on s09, and the port
  with correction C1 against the reference on s07, must exceed the tolerance;
- published messages: what the node publishes (xyz_estimate and the
  xyz_debug_estimate sent with it) against what the original published,
  recorded by the harness (A6), on every stream (R1 finding 1: the original
  published before checkTakeoffState);
- correction C1 (approved at R1, the port's default): the port's default
  output against the independent step-wise model with C1 (independent.py),
  every stream;
- observation accounting: D1 re-fusion counted with C1 off; with C1 on no
  observation is fused twice: an IMU step fuses exactly when a new observation
  arrived since the previous step (s06, s07, h01, h03-h05, h08); observations
  superseded before the next IMU step (last one wins, as in master) are counted.
All parity runs pin correction_c1_clear_xy_flag to false (master semantics).
--stream compares one recorded stream (for example a simulation run
converted by x3_reef_offline) the same way, without golden or negatives.

Exit: 0 PASS, 1 FAIL, 2 BLOCKED (sources or port binary unavailable).
"""
import argparse
import csv
import gzip
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import independent as ind  # noqa: E402
import runs  # noqa: E402

ROOT, BUILD = runs.ROOT, runs.BUILD
B = ROOT / 'baseline'
OUT = BUILD / 'port_parity'
VFIX = BUILD / 'fixtures_vertical'
HFIX = BUILD / 'fixtures_horizontal'
LOCKS = {'fixtures': (B / 'fixtures.lock.json', B / 'tools' / 'fixtures.py', runs.fixtures_dir()),
         'fixtures_vertical': (B / 'fixtures_vertical.lock.json', B / 'tools' / 'fixtures_vertical.py', VFIX),
         'fixtures_horizontal': (B / 'fixtures_horizontal.lock.json', B / 'tools' / 'fixtures_horizontal.py', HFIX)}
PORT_RTOL = 1e-9          # ACCEPTANCE.md section 4 (unchanged)
STATE = (['z', 'zdot', 'zbias'] + [f'zP{i}{j}' for i in range(3) for j in range(3)]
         + ['vx', 'vy', 'pitch_bias', 'roll_bias', 'ax_bias', 'ay_bias']
         + [f'xyP{i}{j}' for i in range(6) for j in range(6)])
CONT = STATE + ['z_meas', 'zR', 'xy_meas0', 'xy_meas1', 'xyR00', 'xyR11', 'u', 'z_dt', 'xy_dt', 'g_init']
DISC = ('z_flag_before', 'xy_flag_before', 'acc_init', 'takeoff', 'z_flag', 'xy_flag', 'n_prop',
        'use_mocap_xy', 'use_mocap_z', 'n_published', 'delivered')
GOLDEN_DISC = ('acc_init', 'takeoff', 'z_flag', 'xy_flag', 'n_prop', 'use_mocap_xy', 'use_mocap_z')
C1_STREAMS = ('s06', 's07', 'h01', 'h03', 'h04', 'h05', 'h08')
CASES = [   # ACCEPTANCE.md 4c/4d, named cases
    ('stationary', ['s01']), ('ascent', ['s02', 's03']), ('descent and landing', ['v01', 'h08']),
    ('tilt/range geometry', ['s05', 'v09']), ('missing data (range, IMU)', ['v02', 's11']),
    ('range invalid', ['v03', 's10']), ('repeated measurements (z)', ['v04', 'v08']),
    ('timestamp anomalies', ['v05', 'v06', 'v07']), ('gravity and accel bias', ['s04', 's08']),
    ('mocap z', ['s12']), ('z disabled', ['v10']), ('IMU 250 Hz', ['s09']),
    ('airborne start (characterization)', ['s15']),
    ('horizontal: mocap velocity', ['s06', 'h01']), ('horizontal: D1 stale re-fusion', ['s07']),
    ('horizontal: nonzero yaw and tilt', ['h01', 'h02', 'h03']),
    ('horizontal: outliers (gate)', ['h03']), ('horizontal: duplicates, out-of-order', ['h04']),
    ('horizontal: velocity dropout', ['h05']), ('horizontal: RC switch both ways', ['s13', 'h06']),
    ('horizontal: full update, RGB-D covariance', ['s14', 'h07']), ('horizontal: landing reset', ['h08']),
    ('horizontal: enable_xy false', ['h09']), ('horizontal: RGB-D ignored', ['h10']),
]


def read(path):
    with open(path) as fh:
        return list(csv.DictReader(fh))


def num(v):
    return float(v)


def same(a, b):
    x, y = float(a), float(b)
    return (math.isnan(x) and math.isnan(y)) or x == y


def compare(port, ref, cont=CONT, disc=DISC, gate=True):
    """Per-event comparison; returns a result dict."""
    res = dict(rows=len(ref), worst=0.0, discrete=[], gate=[], bitwise=0, values=0)
    if len(port) != len(ref):
        res['discrete'].append(f'row count {len(port)} != {len(ref)}')
        res['ok'] = False
        return res
    for k in cont:
        vals = [num(r[k]) for r in ref]
        scale = max((abs(v) for v in vals if math.isfinite(v)), default=0.0)
        for pr, y in zip(port, vals):
            x = num(pr[k])
            res['values'] += 1
            if (math.isnan(x) and math.isnan(y)) or x == y:
                res['bitwise'] += 1
                continue
            d = abs(x - y) if math.isfinite(x - y) else math.inf
            res['worst'] = max(res['worst'], d / (PORT_RTOL * scale + 1e-14))
    for pr, rr in zip(port, ref):
        bad = [k for k in disc if int(num(pr[k])) != int(num(rr[k]))]
        if bad:
            res['discrete'].append(f"idx {pr['idx']} {bad}")
        if gate and (pr.get('z_gate') == '1' or pr.get('xy_gate') == '1'):
            x, y = num(pr['maha2']), num(rr['maha2'])
            scale = abs(y) if math.isfinite(y) else 0.0
            ok = same(x, y) or (math.isfinite(x - y) and abs(x - y) <= PORT_RTOL * scale + 1e-14)
            if not ok:
                res['gate'].append(f"idx {pr['idx']}: {x!r} vs {y!r}")
    res['ok'] = res['worst'] <= 1.0 and not res['discrete'] and not res['gate']
    return res


def wrapper_equivalence(core, node):
    """Node-mode rows must equal core-mode rows (state, counters, fusions)."""
    problems, published = [], 0
    if len(core) != len(node):
        return [f'row count {len(node)} != {len(core)}'], 0
    cols = [c for c in core[0] if c != 'type']
    for c, n in zip(core, node):
        bad = [k for k in cols if not same(c[k], n[k])]
        if bad:
            problems.append(f"idx {c['idx']} core/node differ in {bad[:4]}")
        if n['msg_published'] == '1':
            published += 1
            if int(n['msg_stamp_ns']) != int(c['t_ns']):
                problems.append(f"idx {c['idx']} stamp {n['msg_stamp_ns']} != {c['t_ns']}")
        if len(problems) > 5:
            break
    return problems, published


def compare_published(port, ref):
    """Published messages: same events, every field bit-identical (NaN = NaN)."""
    if [r['idx'] for r in port] != [r['idx'] for r in ref]:
        return [f'published at different events ({len(port)} vs {len(ref)} messages)'], 0
    bad = []
    for p, r in zip(port, ref):
        diff = [k for k in r if not same(p[k], r[k])]
        if diff:
            bad.append(f"idx {r['idx']}: {diff[:4]}")
    return bad, len(ref)


def fixtures_step(update_reason):
    results = []
    for name, (lock, gen, out) in LOCKS.items():
        subprocess.run([sys.executable, str(gen), str(out)], check=True, stdout=subprocess.DEVNULL)
        got = json.loads((out / 'manifest.json').read_text())
        if update_reason and name == 'fixtures_horizontal':
            lock.write_text(json.dumps({**got, 'reason': update_reason}, indent=2) + '\n')
            print(f'UPDATED {lock.relative_to(ROOT)}: {update_reason}')
        if not lock.exists():
            results.append((name, False, f'{lock.name} missing (use --update-lock "<reason>")'))
            continue
        want = json.loads(lock.read_text())['files']
        diff = sorted(k for k in set(got['files']) | set(want) if got['files'].get(k) != want.get(k))
        results.append((name, not diff, f"{len(got['files'])} files" + (f'; differ: {diff[:4]}' if diff else '')))
    return results


def run_port(port, params, events, out, mode='core', published=None):
    out.parent.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, PORT_PUBLISHED=str(published)) if published else None
    subprocess.run([str(port), str(params), str(events), str(out), '--mode', mode], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
    return read(out)


def run_ref(variant, params, events, out, published=None):
    out.parent.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, REF_PUBLISHED=str(published)) if published else None
    subprocess.run([str(BUILD / variant / 'reef_ref'), str(params), str(events), str(out)], check=True, env=env)
    return read(out)


def streams():
    """(label, short, params, events) for every fixture stream compared."""
    items = []
    for s in runs.scenario_names():
        for kind in runs.KINDS:
            items.append((f'{kind}/{s}', s[:3], runs.param_file(kind, s, 'master'),
                          runs.fixtures_dir() / f'{s}.events'))
    for group, d in (('vertical', VFIX), ('horizontal', HFIX)):
        sets = json.loads((d / 'manifest.json').read_text())['param_set_for_scenario']
        for s in sorted(sets):
            items.append((f'{group}/{s}', s[:3], d / 'params' / f'common_{sets[s]}_master.params', d / f'{s}.events'))
    return items


def golden_rows(s):
    with gzip.open(B / 'golden' / f'master__common__{s}.csv.gz', 'rt') as fh:
        return list(csv.DictReader(fh))


def with_c1(params, out, value=True):
    """Copy of a harness parameter file with correction C1 set explicitly."""
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(Path(params).read_text() + f"correction_c1_clear_xy_flag bool {'true' if value else 'false'}\n")
    return out


def master_params(params, out):
    """Parity runs: C1 off, so the port has master's semantics."""
    return with_c1(params, out, value=False)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--port', type=Path, required=True)
    ap.add_argument('--update-lock', metavar='REASON')
    ap.add_argument('--stream', nargs=3, metavar=('PARAMS', 'EVENTS', 'LABEL'))
    args = ap.parse_args()
    t0 = time.time()
    if not args.port.is_file():
        print(f'BLOCKED: port driver {args.port} not found (build reef_estimator first)')
        return 2
    items = []

    def add(group, name, ok, detail):
        items.append(dict(group=group, name=name, ok=bool(ok), detail=detail))
        print(f"{'PASS' if ok else 'FAIL'} [{group}] {name}: {detail}", flush=True)

    if not args.stream:
        for name, ok, detail in fixtures_step(args.update_lock):
            add('fixtures', f'{name} regenerate identically to the lock', ok, detail)
    r = subprocess.run([str(B / 'build_reference.sh'), 'master', 'sim'], capture_output=True, text=True)
    if r.returncode == 2:
        print(r.stdout + r.stderr + '\nBLOCKED: pinned sources unavailable')
        return 2
    add('build', 'reference (master, sim) builds from pinned sources', r.returncode == 0,
        ' '.join(ln for ln in r.stdout.splitlines() if ln.startswith('OK')) or r.stdout[-300:])
    if r.returncode != 0:
        return 1

    todo = [(args.stream[2], 'rec', Path(args.stream[0]), Path(args.stream[1]))] if args.stream else streams()
    out_dir = OUT / ('stream_' + args.stream[2].replace('/', '_') if args.stream else 'fixtures')
    per_stream = {}
    for label, short, params, events in todo:
        safe = label.replace('/', '__')
        mparams = master_params(params, out_dir / 'params' / f'{safe}.params')
        port = run_port(args.port, mparams, events, out_dir / 'port' / f'{safe}.csv',
                        published=out_dir / 'port' / f'{safe}.published.csv')
        ref = run_ref('master', params, events, out_dir / 'ref' / f'{safe}.csv',
                      published=out_dir / 'ref' / f'{safe}.published.csv')
        res = compare(port, ref)
        node = run_port(args.port, mparams, events, out_dir / 'node' / f'{safe}.csv', mode='node',
                        published=out_dir / 'node' / f'{safe}.published.csv')
        wprob, wpub = wrapper_equivalence(port, node)
        ref_pub = read(out_dir / 'ref' / f'{safe}.published.csv')
        pprob, npub = compare_published(read(out_dir / 'node' / f'{safe}.published.csv'), ref_pub)
        cprob, _ = compare_published(read(out_dir / 'port' / f'{safe}.published.csv'),
                                     read(out_dir / 'node' / f'{safe}.published.csv'))
        # C1 (default): the port's default output against the independent model with C1.
        dflt = run_port(args.port, params, events, out_dir / 'default' / f'{safe}.csv')
        pp, ee = runs.parse_params(params), runs.parse_events(events)
        # the model is first checked against the original (C1 off), then used for C1
        st0 = ind.verify_run('master', pp, ee, runs.read_rows(out_dir / 'ref' / f'{safe}.csv'), c1=False)
        st = ind.verify_run('master', pp, ee, runs.read_rows(out_dir / 'default' / f'{safe}.csv'), c1=True)
        last = port[-1]
        per_stream[label] = dict(short=short, wrapper_ok=not wprob,
                                 xy_accepted=int(last['xy_accepted']), xy_fusions=int(last['xy_fusions']),
                                 **{k: v for k, v in res.items() if k not in ('discrete', 'gate')},
                                 discrete=res['discrete'][:3], gate=res['gate'][:3])
        add('parity', label, res['ok'],
            f"{res['rows']} events, worst {res['worst']:.3g} x tol, {res['bitwise']}/{res['values']} values bit-identical"
            + (f"; discrete {res['discrete'][:2]}" if res['discrete'] else '')
            + (f"; gate {res['gate'][:2]}" if res['gate'] else ''))
        add('wrapper', label, not wprob and wpub == int(last['n_published']) and not cprob,
            f'node state = core state at every event; {wpub} messages, built in core and node mode identical'
            + (f'; {wprob[:2]}' if wprob else '') + (f'; {cprob[:2]}' if cprob else ''))
        add('published', label, not pprob and npub == int(last['n_published']),
            f'{npub} published messages (estimate + debug) bit-identical to what the original published'
            + (f'; {pprob[:2]}' if pprob else ''))
        add('c1', label, st.worst <= 1 and not st.mismatches and st0.worst <= 1 and not st0.mismatches,
            f'independent model vs original (C1 off): worst {st0.worst:.3g} x tol; '
            f'default (C1) vs model with C1: {st.steps} steps, {st.gate_checks} gates, '
            f'worst {st.worst:.3g} x tol' + (f'; {st.worst_where}' if st.worst > 1 else '')
            + (f'; {st.mismatches[:1]}' if st.mismatches else '')
            + (f'; model vs original: {st0.worst_where} {st0.mismatches[:1]}' if st0.worst > 1 or st0.mismatches else ''))
        if label.startswith('common/'):
            s = label.split('/')[1]
            gold = golden_rows(s)
            by_idx = {int(p['idx']): p for p in port}
            mine = [by_idx[int(num(g['idx']))] for g in gold]
            gres = compare(mine, gold, cont=[k for k in CONT if k in gold[0]], disc=GOLDEN_DISC, gate=False)
            add('golden', f'port vs committed P02 golden {s}', gres['ok'],
                f"{gres['rows']} decimated rows, worst {gres['worst']:.3g} x tol"
                + (f"; discrete {gres['discrete'][:2]}" if gres['discrete'] else ''))

    if not args.stream:
        s09 = 's09_imu_250hz'
        sim = run_ref('sim', runs.param_file('common', s09, 'sim'), runs.fixtures_dir() / f'{s09}.events',
                      out_dir / 'ref_sim' / f'{s09}.csv')
        neg = compare(read(out_dir / 'port' / f'common__{s09}.csv'), sim, disc=('acc_init',), gate=False)
        add('negative', 'port vs simulation revision on s09 exceeds the tolerance', neg['worst'] > 1.0,
            f"worst {neg['worst']:.3g} x tol (must be > 1)")

        # Observation accounting and correction C1 (opt-in, NOT APPROVED).
        s07 = runs.fixtures_dir() / 's07_single_mocap.events'
        c1_params = with_c1(runs.param_file('common', 's07_single_mocap', 'master'), out_dir / 'c1' / 's07.params')
        c1 = run_port(args.port, c1_params, s07, out_dir / 'c1' / 's07.csv')   # C1 on (the default)
        ref07 = read(out_dir / 'ref' / 'common__s07_single_mocap.csv')
        neg = compare(c1, ref07, gate=False)
        add('negative', 'port with C1 differs from the reference on s07', not neg['ok'],
            f"worst {neg['worst']:.3g} x tol, {len(neg['discrete'])} discrete differences (must differ)")
        d1 = per_stream['common/s07_single_mocap']
        add('accounting', 'D1 re-fusion counted with C1 off (s07)', d1['xy_fusions'] > d1['xy_accepted'],
            f"{d1['xy_accepted']} accepted observations, {d1['xy_fusions']} fusions")
        for label, short, params, events in streams():
            if short not in C1_STREAMS or label.startswith('shipped/'):
                continue
            safe = label.replace('/', '__')
            rows = run_port(args.port, with_c1(params, out_dir / 'c1' / f'{safe}.params'), events,
                            out_dir / 'c1' / f'{safe}.csv')
            # With C1 an IMU step fuses exactly when a new observation arrived
            # since the previous step, and clears the flag: no observation is
            # fused twice. Observations superseded by a newer one before the
            # next IMU step (last one wins, as in master) are never fused.
            expected, stale, prev_pub, prev_fus = 0, [], 0, 0
            for r in rows:
                pub, fus = int(r['n_published']), int(r['xy_fusions'])
                if pub > prev_pub:                       # an IMU step that produced an estimate
                    new_obs = r['xy_flag_before'] == '1'
                    expected += new_obs
                    if (fus - prev_fus) != int(new_obs) or r['xy_flag'] == '1':
                        stale.append(r['idx'])
                prev_pub, prev_fus = pub, fus
            acc, fus = int(rows[-1]['xy_accepted']), int(rows[-1]['xy_fusions'])
            add('accounting', f'C1: no observation fused twice ({label})',
                not stale and fus == expected and 0 < fus <= acc,
                f'{acc} accepted, {fus} fusions = {expected} IMU steps with a new observation; '
                f'{acc - fus} superseded before the next IMU step'
                + (f'; wrong fusion at {stale[:3]}' if stale else ''))

        for case, shorts in CASES:
            labels = [lb for lb, d in per_stream.items() if d['short'] in shorts]
            ok = bool(labels) and all(per_stream[lb]['ok'] and per_stream[lb]['wrapper_ok'] for lb in labels)
            add('case', case, ok, ', '.join(labels))

    passed = sum(i['ok'] for i in items)
    ok = passed == len(items)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / 'results.json').write_text(json.dumps(dict(
        ok=ok, items=items, streams=per_stream, port=str(args.port), elapsed_s=round(time.time() - t0, 1)), indent=2))
    print(f"{'PASS' if ok else 'FAIL'}: {passed}/{len(items)} assertions "
          f"({len(per_stream)} streams, {sum(d['rows'] for d in per_stream.values())} events), "
          f"{time.time() - t0:.0f} s; report {(out_dir / 'results.json').relative_to(ROOT)}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
