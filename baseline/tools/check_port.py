#!/usr/bin/env python3
"""P04 fidelity check: ported vertical estimator vs the pinned original.

    check_vertical.py --port PATH/TO/reef_estimator_event_replay
                      [--update-lock "<reason>"]

For every P02 fixture (master, both parameter kinds) and every vertical
fixture v01-v10, the same event stream and parameters go to the reference
harness (build_reference.sh master) and to the port's event driver. Compared
per event (ACCEPTANCE.md section 4c):
- continuous Z fields within 1e-9 x max_run|ref field| (NaN/inf class equal),
- discrete fields exactly, and the gate value where the port evaluated a gate,
- the port against the committed P02 golden (decimated rows),
- negative: the port against the simulation revision on s09 must fail,
- wrapper equivalence: the same events as ROS 2 messages through the
  SensorManager node's callbacks (--mode node) give the core's state, and the
  published messages carry exactly that state (horizontal fields NaN).
Horizontal coverage is reported as NOT IMPLEMENTED and never counted.

Exit: 0 PASS, 1 FAIL, 2 BLOCKED (sources or port binary unavailable).
"""
import argparse
import csv
import gzip
import json
import math
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import runs  # noqa: E402

ROOT, BUILD = runs.ROOT, runs.BUILD
B = ROOT / 'baseline'
OUT = BUILD / 'port_vertical'
VFIX = BUILD / 'fixtures_vertical'
LOCKS = {'fixtures': (B / 'fixtures.lock.json', B / 'tools' / 'fixtures.py', runs.fixtures_dir()),
         'fixtures_vertical': (B / 'fixtures_vertical.lock.json', B / 'tools' / 'fixtures_vertical.py', VFIX)}
PORT_RTOL = 1e-9          # ACCEPTANCE.md section 4 (unchanged)
CONT = (['z', 'zdot', 'zbias'] + [f'zP{i}{j}' for i in range(3) for j in range(3)]
        + ['z_meas', 'zR', 'u', 'z_dt', 'g_init'])
DISC = ('z_flag_before', 'acc_init', 'takeoff', 'z_flag', 'n_prop', 'use_mocap_z', 'n_published', 'delivered')
GOLDEN_DISC = ('acc_init', 'takeoff', 'z_flag', 'n_prop', 'use_mocap_z')
CASES = [   # ACCEPTANCE.md section 4c, named vertical cases
    ('stationary', ['s01']), ('ascent', ['s02', 's03']), ('descent and landing', ['v01']),
    ('tilt/range geometry', ['s05', 'v09']), ('missing data', ['v02', 's11']),
    ('range invalid', ['v03', 's10']), ('repeated measurements', ['v04', 'v08']),
    ('timestamp anomalies', ['v05', 'v06', 'v07']), ('gravity and accel bias', ['s04', 's08']),
    ('mocap z', ['s12']), ('RC switch', ['s13']), ('full update', ['s14']), ('z disabled', ['v10']),
    ('IMU 250 Hz', ['s09']), ('airborne start (characterization)', ['s15']),
    ('z during horizontal fixtures', ['s06', 's07']),
]
HORIZONTAL = ['XY velocity with mocap (s06)', 'stale XY re-fusion D1 (s07)', 'RGB-D/mocap XY switch (s13)',
              'XY full update (s14)', 'XY landing reset', 'XY gates']


def read(path):
    with open(path) as fh:
        return list(csv.DictReader(fh))


def num(v):
    return float(v)


def compare(port, ref, cont=CONT, disc=DISC, gate=True):
    """Per-event comparison; returns a result dict."""
    res = dict(rows=len(ref), worst=0.0, discrete=[], gate=[], bitwise=0, values=0)
    if len(port) != len(ref):
        res['discrete'].append(f'row count {len(port)} != {len(ref)}')
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
    for i, (pr, rr) in enumerate(zip(port, ref)):
        bad = [k for k in disc if int(num(pr[k])) != int(num(rr[k]))]
        if bad:
            res['discrete'].append(f"idx {pr['idx']} {bad}")
        if gate and pr.get('z_gate') == '1':
            x, y = num(pr['maha2']), num(rr['maha2'])
            scale = abs(y) if math.isfinite(y) else 0.0
            same = (math.isnan(x) and math.isnan(y)) or x == y or (
                math.isfinite(x - y) and abs(x - y) <= PORT_RTOL * scale + 1e-14)
            if not same:
                res['gate'].append(f"idx {pr['idx']}: {x!r} vs {y!r}")
    res['ok'] = res['worst'] <= 1.0 and not res['discrete'] and not res['gate']
    return res


def fixtures_step(update_reason):
    results = []
    for name, (lock, gen, out) in LOCKS.items():
        subprocess.run([sys.executable, str(gen), str(out)], check=True, stdout=subprocess.DEVNULL)
        got = json.loads((out / 'manifest.json').read_text())
        if update_reason and name == 'fixtures_vertical':
            lock.write_text(json.dumps({**got, 'reason': update_reason}, indent=2) + '\n')
            print(f'UPDATED {lock.relative_to(ROOT)}: {update_reason}')
        if not lock.exists():
            results.append((name, False, f'{lock.name} missing (use --update-lock "<reason>")'))
            continue
        want = json.loads(lock.read_text())['files']
        diff = sorted(k for k in set(got['files']) | set(want) if got['files'].get(k) != want.get(k))
        results.append((name, not diff, f"{len(got['files'])} files" + (f'; differ: {diff[:4]}' if diff else '')))
    return results


def run_port(port, params, events, out, mode='core'):
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([str(port), str(params), str(events), str(out), '--mode', mode], check=True,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return read(out)


def same(a, b):
    x, y = float(a), float(b)
    return (math.isnan(x) and math.isnan(y)) or x == y


def wrapper_equivalence(core, node):
    """Node-mode rows must equal core-mode rows; messages must carry the core state."""
    problems, published = [], 0
    if len(core) != len(node):
        return [f'row count {len(node)} != {len(core)}'], 0
    cols = [c for c in core[0] if c != 'type']
    msg = [('msg_z', 'z'), ('msg_zdot', 'zdot'), ('dbg_bias', 'zbias'), ('dbg_u', 'u')] + \
          [(f'dbg_p{k}', f'zP{k // 3}{k % 3}') for k in range(9)]
    for c, n in zip(core, node):
        bad = [k for k in cols if not same(c[k], n[k])]
        if bad:
            problems.append(f"idx {c['idx']} core/node differ in {bad[:4]}")
        if n['msg_published'] == '1':
            published += 1
            if int(n['msg_stamp_ns']) != int(c['t_ns']):
                problems.append(f"idx {c['idx']} stamp {n['msg_stamp_ns']} != {c['t_ns']}")
            if n['msg_x_dot_isnan'] != '1':
                problems.append(f"idx {c['idx']} horizontal field not NaN")
            bad = [m for m, k in msg if not same(n[m], c[k])]
            if bad:
                problems.append(f"idx {c['idx']} message differs in {bad[:4]}")
        if len(problems) > 5:
            break
    return problems, published


def run_ref(variant, params, events, out):
    out.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run([str(BUILD / variant / 'reef_ref'), str(params), str(events), str(out)], check=True)
    return read(out)


def streams():
    """(label, short, params, events) for every stream compared."""
    items = []
    for s in runs.scenario_names():
        for kind in runs.KINDS:
            items.append((f'{kind}/{s}', s[:3], runs.param_file(kind, s, 'master'),
                          runs.fixtures_dir() / f'{s}.events'))
    sets = json.loads((VFIX / 'manifest.json').read_text())['param_set_for_scenario']
    for s in sorted(sets):
        items.append((f'vertical/{s}', s[:3], VFIX / 'params' / f'common_{sets[s]}_master.params',
                      VFIX / f'{s}.events'))
    return items


def golden_rows(s):
    p = B / 'golden' / f'master__common__{s}.csv.gz'
    with gzip.open(p, 'rt') as fh:
        return list(csv.DictReader(fh))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--port', type=Path, required=True)
    ap.add_argument('--update-lock', metavar='REASON')
    args = ap.parse_args()
    t0 = time.time()
    if not args.port.is_file():
        print(f'BLOCKED: port driver {args.port} not found (build reef_estimator first)')
        return 2
    items = []   # (group, name, ok, detail)

    def add(group, name, ok, detail):
        items.append(dict(group=group, name=name, ok=bool(ok), detail=detail))
        print(f"{'PASS' if ok else 'FAIL'} [{group}] {name}: {detail}", flush=True)

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

    per_stream = {}
    for label, short, params, events in streams():
        safe = label.replace('/', '__')
        port = run_port(args.port, params, events, OUT / 'port' / f'{safe}.csv')
        ref = run_ref('master', params, events, OUT / 'ref' / f'{safe}.csv')
        res = compare(port, ref)
        node = run_port(args.port, params, events, OUT / 'node' / f'{safe}.csv', mode='node')
        wprob, wpub = wrapper_equivalence(port, node)
        add('wrapper', label, not wprob and wpub == int(port[-1]['n_published']),
            f'{wpub} published messages equal the core state' + (f'; {wprob[:2]}' if wprob else ''))
        per_stream[label] = dict(wrapper_ok=not wprob, short=short, **{k: v for k, v in res.items() if k not in ('discrete', 'gate')},
                                 discrete=res['discrete'][:3], gate=res['gate'][:3])
        add('fidelity', label,
            res['ok'], f"{res['rows']} events, worst {res['worst']:.3g} x tol, "
            f"{res['bitwise']}/{res['values']} values bit-identical"
            + (f"; discrete {res['discrete'][:2]}" if res['discrete'] else '')
            + (f"; gate {res['gate'][:2]}" if res['gate'] else ''))
        if label.startswith('common/'):
            s = label.split('/')[1]
            gold = golden_rows(s)
            by_idx = {int(p['idx']): p for p in port}
            mine = [by_idx[int(num(g['idx']))] for g in gold]
            gres = compare(mine, gold, cont=[k for k in CONT if k in gold[0]], disc=GOLDEN_DISC, gate=False)
            add('golden', f'port vs committed P02 golden {s}', gres['ok'],
                f"{gres['rows']} decimated rows, worst {gres['worst']:.3g} x tol"
                + (f"; discrete {gres['discrete'][:2]}" if gres['discrete'] else ''))

    s09 = 's09_imu_250hz'
    port = read(OUT / 'port' / f'common__{s09}.csv')
    sim = run_ref('sim', runs.param_file('common', s09, 'sim'), runs.fixtures_dir() / f'{s09}.events',
                  OUT / 'ref_sim' / f'{s09}.csv')
    neg = compare(port, sim, disc=('acc_init',), gate=False)
    add('negative', 'port vs simulation revision on s09 exceeds the tolerance', neg['worst'] > 1.0,
        f"worst {neg['worst']:.3g} x tol (must be > 1)")

    case_rows = []
    for case, shorts in CASES:
        labels = [lb for lb, d in per_stream.items() if d['short'] in shorts]
        ok = bool(labels) and all(per_stream[lb]['ok'] for lb in labels)
        case_rows.append(dict(case=case, streams=labels, ok=ok))
        add('case', case, ok, ', '.join(labels))
    print('NOT IMPLEMENTED (horizontal filter not ported; not counted): ' + '; '.join(HORIZONTAL))

    passed = sum(i['ok'] for i in items)
    ok = passed == len(items)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / 'results.json').write_text(json.dumps(dict(
        ok=ok, items=items, streams=per_stream, cases=case_rows, horizontal_not_implemented=HORIZONTAL,
        port=str(args.port), elapsed_s=round(time.time() - t0, 1)), indent=2))
    print(f"{'PASS' if ok else 'FAIL'}: {passed}/{len(items)} assertions "
          f"({len(per_stream)} streams, {sum(d['rows'] for d in per_stream.values())} events), "
          f"{time.time() - t0:.0f} s; report {(OUT / 'results.json').relative_to(ROOT)}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
