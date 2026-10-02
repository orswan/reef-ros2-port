#!/usr/bin/env python3
"""P02 baseline evidence: run the reference harness for both pinned
estimator variants over all fixtures, verify it, and characterize it.

    check_baseline.py [--floor] [--update-golden REASON]

Steps (each recorded as an assertion):
  1. fixtures are regenerated and match baseline/fixtures.lock.json
  2. both variants build from the pinned, checksummed original sources
  3. every run (2 variants x 2 parameter sets x all scenarios) completes
  4. independent step-wise re-derivation matches every step (RTOL 1e-12)
  5. covariance invariants (finite, symmetric, positive semidefinite)
  6. analytic and characterization expectations per variant
  7. decimated reference outputs match the committed golden files
  8. negative check: a deliberately mutated build is rejected
  9. (--floor) floating-point floor across compiler builds
Writes build/baseline/report/{summary.md,results.json}.
Exit: 0 all assertions passed, 1 a check failed, 2 prerequisites unavailable.
"""
import argparse
import csv
import gzip
import hashlib
import io
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import independent as ind  # noqa: E402  (imported after the sys.path insert above)
import runs  # noqa: E402  (imported after the sys.path insert above)

ROOT, BUILD = runs.ROOT, runs.BUILD
B = ROOT / 'baseline'
GOLDEN = B / 'golden'
LOCK = B / 'fixtures.lock.json'
T_HOVER = 4.8
STATE = (['z', 'zdot', 'zbias'] + [f'zP{i}{j}' for i in range(3) for j in range(3)]
         + ['vx', 'vy', 'pitch_bias', 'roll_bias', 'ax_bias', 'ay_bias']
         + [f'xyP{i}{j}' for i in range(6) for j in range(6)])
DISCRETE = ('z_flag', 'xy_flag', 'takeoff', 'n_prop', 'use_mocap_xy', 'use_mocap_z', 'acc_init')
PORT_RTOL = 1e-9          # fixed tolerance for future port vs reference (BASELINE_DECISION.md section 8)
GOLDEN_EVERY = 100


class Report:
    def __init__(self):
        self.items = []

    def add(self, group, name, ok, detail):
        self.items.append(dict(group=group, name=name, ok=bool(ok), detail=detail))
        print(f"{'PASS' if ok else 'FAIL'} [{group}] {name}: {detail}", flush=True)

    @property
    def ok(self):
        return all(i['ok'] for i in self.items)


def tsec(r):
    return (r['t_ns'] - 1e9) / 1e9


def imu_rows(rows):
    return [r for r in rows if r['type'] == 'imu']


def last_imu_before(rows, t):
    return [r for r in imu_rows(rows) if tsec(r) <= t + 1e-9][-1]


def truth(scenario):
    with open(runs.fixtures_dir() / f'{scenario}.truth.csv') as f:
        return {int(r['t_ns']): {k: float(v) for k, v in r.items()} for r in csv.DictReader(f)}


# --------------------------------------------------------------------------
def step_fixtures(rep):
    out = runs.fixtures_dir()
    subprocess.run([sys.executable, str(B / 'tools' / 'fixtures.py'), str(out)], check=True,
                   stdout=subprocess.DEVNULL)
    got = json.loads((out / 'manifest.json').read_text())['files']
    if not LOCK.exists():
        rep.add('fixtures', 'fixtures match lock file', False, f'{LOCK.name} missing')
        return
    want = json.loads(LOCK.read_text())['files']
    diff = sorted(k for k in set(got) | set(want) if got.get(k) != want.get(k))
    rep.add('fixtures', 'regenerated fixtures match lock file', not diff,
            f'{len(got)} files' + (f'; differ: {diff[:5]}' if diff else ''))


def step_build(rep):
    r = subprocess.run([str(B / 'build_reference.sh')], capture_output=True, text=True)
    if r.returncode == 2:
        print(r.stdout + r.stderr)
        print('BLOCKED: pinned sources unavailable (baseline/fetch_sources.sh)')
        sys.exit(2)
    rep.add('build', 'both variants build from pinned, checksummed sources', r.returncode == 0,
            ' '.join(l for l in r.stdout.splitlines() if l.startswith('OK')) or r.stdout[-300:])
    return r.returncode == 0


def step_runs(rep):
    outputs, n = {}, 0
    for v in runs.VARIANTS:
        for kind in runs.KINDS:
            for s in runs.scenario_names():
                try:
                    outputs[(v, kind, s)] = runs.read_rows(runs.run(v, kind, s))
                    n += 1
                except subprocess.CalledProcessError as e:
                    rep.add('runs', f'{v}/{kind}/{s}', False, f'harness exit {e.returncode}')
    rep.add('runs', 'all reference runs completed', n == 2 * 2 * len(runs.scenario_names()),
            f'{n} runs, {sum(len(r) for r in outputs.values())} rows')
    return outputs


def step_independent(rep, outputs):
    worst, steps, gates, bad = 0.0, 0, 0, []
    for (v, kind, s), rows in outputs.items():
        st = ind.verify_run(v, runs.parse_params(runs.param_file(kind, s, v)),
                            runs.parse_events(runs.fixtures_dir() / f'{s}.events'), rows)
        steps += st.steps
        gates += st.gate_checks
        worst = max(worst, st.worst)
        if st.worst > 1 or st.mismatches:
            bad.append(f'{v}/{kind}/{s}: {st.worst_where} {st.mismatches[:1]}')
    rep.add('independent', 'step-wise re-derivation matches the reference', not bad,
            f'{steps} propagation steps, {gates} gate decisions, worst deviation {worst:.3g} x tolerance '
            f'(RTOL {ind.RTOL:g}, ATOL {ind.ATOL:g})' + (f'; failing: {bad[:3]}' if bad else ''))


def step_invariants(rep, outputs):
    asym, eig, nonfinite = 0.0, 0.0, []
    for (v, kind, s), rows in outputs.items():
        for r in rows:
            _, Pz, _, Pxy = ind.row_state(r)
            for P in (Pz, Pxy):
                if not np.all(np.isfinite(P)):
                    nonfinite.append((v, kind, s))
                    break
                n = np.max(np.abs(P)) or 1.0
                asym = max(asym, np.max(np.abs(P - P.T)) / n)
                eig = min(eig, np.min(np.linalg.eigvalsh(0.5 * (P + P.T))) / n)
            x = [r[k] for k in STATE[:3] + STATE[12:18]]
            if not all(math.isfinite(val) for val in x):
                nonfinite.append((v, kind, s))
    nonfinite = sorted(set(nonfinite))
    expected_nonfinite = {('sim', k, 's10_range_outliers') for k in runs.KINDS}
    rep.add('invariants', 'covariance symmetric: max |P - P^T| / max|P| <= 1e-12', asym <= 1e-12, f'{asym:.3g}')
    rep.add('invariants', 'covariance PSD: min eig(sym P) / max|P| >= -1e-12', eig >= -1e-12, f'{eig:.3g}')
    rep.add('invariants', 'values finite (except the characterized sim -inf case)',
            set(nonfinite) == expected_nonfinite, f'non-finite runs: {nonfinite}')


# --------------------------------------------------------------------------
def step_analytic(rep, outputs):
    """Expectations from physics / the documented equations, per variant."""
    O = lambda v, s, kind='common': outputs[(v, kind, s)]
    for v in runs.VARIANTS:
        g = f'analytic:{v}'
        # s01: on the ground: never takes off, landing reset keeps zdot at 0, z tracks ground range.
        rows = O(v, 's01_ground_stationary')
        last = rows[-1]
        rep.add(g, 's01 ground: no takeoff, n_prop < 10, z tracks range 0.12 m',
                max(r['takeoff'] for r in rows) == 0 and max(r['n_prop'] for r in rows) < 10
                and abs(last['z'] + 0.12) < 1e-3, f"z={last['z']:.5f}")
        # s02: takeoff shortly after the range passes 0.25 m with the primer; hover converges.
        rows, tr = O(v, 's02_takeoff_hover'), truth('s02_takeoff_hover')
        t_off = next(tsec(r) for r in rows if r['takeoff'])
        t_025 = min((t - 1e9) / 1e9 for t, x in tr.items() if x['h_up'] >= 0.25)
        tail = [r for r in imu_rows(rows) if tsec(r) >= 7.8]
        rep.add(g, 's02 takeoff within 0.1 s of the range passing 0.25 m (primer)',
                0 <= t_off - t_025 <= 0.1, f'range 0.25 m at {t_025:.3f} s, takeoff at {t_off:.3f} s')
        rep.add(g, 's02 hover: |z + h| <= 1 mm, |zdot| <= 1 mm/s over the last 2 s',
                max(abs(r['z'] + 1.52) for r in tail) <= 1e-3 and max(abs(r['zdot']) for r in tail) <= 1e-3,
                f"max |z+h| {max(abs(r['z'] + 1.52) for r in tail):.2e}, max |zdot| {max(abs(r['zdot']) for r in tail):.2e}")
        # s03: constant upward acceleration; NED zdot must be the negative of the up-velocity.
        rows, tr = O(v, 's03_const_accel_up'), truth('s03_const_accel_up')
        errs = [abs(r['zdot'] + tr[int(r['t_ns'])]['v_up']) for r in imu_rows(rows) if 6.3 <= tsec(r) <= 8.8]
        rep.add(g, 's03 constant 0.5 m/s^2 climb: |zdot_NED + v_up| <= 0.02 m/s (sign convention NED)',
                max(errs) <= 0.02, f'max error {max(errs):.4f} m/s')
        # s05: 10 deg roll: REEF uses the range as vertical altitude (no tilt compensation).
        rows = O(v, 's05_tilt_range')
        z = last_imu_before(rows, 10.7)['z']
        want = -1.52 / math.cos(math.radians(10))
        rep.add(g, 's05 tilt: z -> -range = -h/cos(10 deg) (no tilt compensation)',
                abs(z - want) <= 1e-3, f'z={z:.5f}, -h/cos={want:.5f}, true height error {z + 1.52:+.4f} m')
        # s06/s14: mocap velocity tracked with partial and with full updates.
        for s in ('s06_mocap_xy_velocity', 's14_full_update'):
            rows = O(v, s)
            tail = [r for r in imu_rows(rows) if tsec(r) >= 8.8]
            rep.add(g, f'{s[:3]} mocap XY: |vx - 0.5| <= 0.01 over the last 2 s',
                    max(abs(r['vx'] - 0.5) for r in tail) <= 0.01, f"vx end {tail[-1]['vx']:.5f}")
        # s11: NaN IMU sample is skipped; the next dt covers two periods.
        rows = O(v, 's11_imu_nan')
        k = next(i for i, r in enumerate(rows) if r['type'] == 'imu_nan')
        nxt = next(r for r in rows[k + 1:] if r['type'] == 'imu')
        same = all(rows[k][c] == rows[k - 1][c] for c in STATE)
        rep.add(g, 's11 NaN IMU skipped; next dt = 4 ms', same and abs(nxt['z_dt'] - 0.004) < 1e-9,
                f"state unchanged {same}, next dt {nxt['z_dt']:.6f}")
        # s12: mocap Z drives the vertical estimate.
        rows = O(v, 's12_mocap_z')
        rep.add(g, 's12 mocap Z: |z + h| <= 1 mm at the end', abs(rows[-1]['z'] + 1.52) <= 1e-3,
                f"z={rows[-1]['z']:.5f}")
        # s13: RC switch from RGB-D (0.8 v) to mocap (v).
        rows = O(v, 's13_rc_mocap_switch')
        before = last_imu_before(rows, 7.7)['vx']
        after = rows[-1]['vx']
        k = next(i for i, r in enumerate(rows) if r['type'] == 'rc')
        rep.add(g, 's13 RC switch: vx ~ rgbd (0.4) before, mocap (0.5) after; use_mocap_xy flips at the RC event',
                abs(before - 0.4) < 0.01 and abs(after - 0.5) < 0.01 and rows[k - 1]['use_mocap_xy'] == 0
                and rows[k]['use_mocap_xy'] == 1, f'vx before {before:.4f}, after {after:.4f}')

    # ---- differences between the variants (characterization) -------------
    g = 'characterization'
    # Bias sign convention (s04, +0.2 m/s^2 offset on body-z specific force).
    b = {v: O(v, 's04_accel_bias')[-1]['zbias'] for v in runs.VARIANTS}
    rep.add(g, 's04 bias sign: master b -> +offset (a = u - b), sim b -> -offset (a = u + b)',
            b['master'] > 0.05 and b['sim'] < -0.05, f"b_master {b['master']:+.4f}, b_sim {b['sim']:+.4f}")
    # Gravity: master hard-codes 9.81; sim measures it (s08, true 9.80665).
    rows = {v: O(v, 's08_standard_gravity') for v in runs.VARIANTS}
    g_init = {v: next(r['g_init'] for r in rows[v] if r['acc_init']) for v in runs.VARIANTS}
    rep.add(g, 's08 gravity: master g=9.81 (bias absorbs 0.00335), sim g=measured',
            g_init['master'] == 9.81 and abs(g_init['sim'] - 9.80665) < 1e-9
            and rows['master'][-1]['zbias'] > 0.001 and abs(rows['sim'][-1]['zbias']) < 0.001,
            f"g {g_init['master']} / {g_init['sim']:.5f}; bias end {rows['master'][-1]['zbias']:+.5f} / {rows['sim'][-1]['zbias']:+.5f}")
    # 250 Hz IMU: master uses measured dt; sim's Z model is fixed at dt = 2 ms.
    ratio = {}
    for v in runs.VARIANTS:
        rows, tr = O(v, 's09_imu_250hz'), truth('s09_imu_250hz')
        r = last_imu_before(rows, 8.7)
        ratio[v] = -r['zdot'] / tr[int(r['t_ns'])]['v_up']
    rep.add(g, 's09 IMU 250 Hz: master zdot/true ~ 1; sim ~ 2 (fixed 2 ms Z model)',
            abs(ratio['master'] - 1) < 0.02 and 1.8 <= ratio['sim'] <= 2.2,
            f"master {ratio['master']:.3f}, sim {ratio['sim']:.3f}")
    # Stale partial update (s07): master re-applies a single mocap measurement every IMU step.
    stale = {}
    for v in runs.VARIANTS:
        rows = O(v, 's07_single_mocap')
        k = next(i for i, r in enumerate(rows) if r['type'] == 'mocap_twist')
        after = imu_rows(rows[k + 1:])
        stale[v] = (sum(1 for r in after if r['xy_flag']), len(after), after[-1]['xyP00'])
    rep.add(g, 's07 master keeps re-fusing one mocap measurement (flag never cleared after a partial update); sim fuses it once',
            stale['master'][0] == stale['master'][1] and stale['sim'][0] == 0
            and stale['master'][2] < 1e-3 and stale['sim'][2] > 0.1,
            f"flag set on {stale['master'][0]}/{stale['master'][1]} later IMU steps (master) vs {stale['sim'][0]} (sim); "
            f"final xyP00 {stale['master'][2]:.2e} vs {stale['sim'][2]:.3f}")
    # Outliers (s10): master gates; sim accepts the 3.5 m outlier and -inf (state becomes NaN).
    res = {}
    for v in runs.VARIANTS:
        rows, ev = O(v, 's10_range_outliers'), runs.parse_events(runs.fixtures_dir() / 's10_range_outliers.events')
        acc = {}
        for i, e in enumerate(ev):
            if e[0] == 'range' and (e[2] in (3.5, float('inf'), float('-inf'))):
                acc[e[2]] = bool(rows[i]['z_flag']) and not rows[i]['z_flag_before']
        res[v] = (acc, math.isnan(rows[-1]['z']))
    rep.add(g, 's10 outliers: master rejects 3.5 m, -inf, +inf; sim accepts 3.5 m and -inf (state NaN), rejects +inf',
            res['master'] == ({3.5: False, float('-inf'): False, float('inf'): False}, False)
            and res['sim'] == ({3.5: True, float('-inf'): True, float('inf'): False}, True),
            f"master {res['master']}, sim {res['sim']}")
    # Airborne start (s15): master's gate rejects every range against z0 = -0.25 and never takes off.
    st = {v: O(v, 's15_airborne_start') for v in runs.VARIANTS}
    acc = {v: sum(1 for r in st[v] if r['type'] == 'range' and r['z_flag'] and not r['z_flag_before']) for v in runs.VARIANTS}
    rep.add(g, 's15 airborne start: master rejects all ranges and never takes off; sim initializes',
            acc['master'] == 0 and max(r['takeoff'] for r in st['master']) == 0
            and max(r['takeoff'] for r in st['sim']) == 1 and abs(st['sim'][-1]['z'] + 1.52) < 1e-3,
            f"ranges accepted master {acc['master']}, sim {acc['sim']}; sim z end {st['sim'][-1]['z']:.4f}")
    # The port tolerance must discriminate between the two variants.
    worst = min(scaled_diff(O('master', s), O('sim', s)) for s in runs.scenario_names()
                if s not in ('s01_ground_stationary',))
    rep.add(g, f'port tolerance ({PORT_RTOL:g} x field scale) separates master from sim in every flight scenario',
            worst > 1e3, f'smallest master/sim difference = {worst:.3g} x tolerance')


def scaled_diff(a, b):
    """max over rows and state fields of |a - b| / (PORT_RTOL * max_run|b| + 1e-14), NaN-aware."""
    worst = 0.0
    scale = {k: max((abs(r[k]) for r in b if math.isfinite(r[k])), default=0.0) for k in STATE}
    for ra, rb in zip(a, b):
        for k in STATE:
            x, y = ra[k], rb[k]
            if (math.isnan(x) and math.isnan(y)) or x == y:
                continue
            d = abs(x - y) if math.isfinite(x - y) else math.inf
            worst = max(worst, d / (PORT_RTOL * scale[k] + 1e-14))
    return worst


# --------------------------------------------------------------------------
def decimate(rows):
    keep = [r for i, r in enumerate(rows)
            if i % GOLDEN_EVERY == 0 or i == len(rows) - 1
            or (i and any(r[k] != rows[i - 1][k] for k in ('takeoff', 'use_mocap_xy', 'use_mocap_z')))]
    return keep


def golden_path(v, kind, s):
    return GOLDEN / f'{v}__{kind}__{s}.csv.gz'


def write_golden(outputs, reason):
    GOLDEN.mkdir(parents=True, exist_ok=True)
    index = {}
    for (v, kind, s), rows in outputs.items():
        keep = decimate(rows)
        cols = list(rows[0].keys())
        buf = io.StringIO()
        w = csv.writer(buf)
        w.writerow(cols)
        for r in keep:
            w.writerow([r[c] if c == 'type' else repr(float(r[c])) for c in cols])
        with gzip.GzipFile(golden_path(v, kind, s), 'wb', mtime=0) as f:
            f.write(buf.getvalue().encode())
        full = runs.BUILD / 'out' / v / kind / f'{s}.csv'
        index[f'{v}/{kind}/{s}'] = dict(rows=len(rows), kept=len(keep),
                                        full_output_sha256=hashlib.sha256(full.read_bytes()).hexdigest())
    (GOLDEN / 'index.json').write_text(json.dumps(dict(reason=reason, every=GOLDEN_EVERY, runs=index), indent=2))


def step_golden(rep, outputs):
    if not (GOLDEN / 'index.json').exists():
        rep.add('golden', 'reference outputs match committed golden', False, 'no golden files (use --update-golden)')
        return
    index = json.loads((GOLDEN / 'index.json').read_text())['runs']
    worst, missing, discrete, bitwise = 0.0, [], [], 0
    for (v, kind, s), rows in outputs.items():
        p = golden_path(v, kind, s)
        if not p.exists():
            missing.append(f'{v}/{kind}/{s}')
            continue
        with gzip.open(p, 'rt') as f:
            gold = [{k: (val if k == 'type' else float(val)) for k, val in r.items()} for r in csv.DictReader(f)]
        by_idx = {int(r['idx']): r for r in rows}
        mine = [by_idx[int(g['idx'])] for g in gold]
        for g_row, m_row in zip(gold, mine):
            if any(g_row[k] != m_row[k] for k in DISCRETE):
                discrete.append(f"{v}/{kind}/{s} row {int(g_row['idx'])}")
        worst = max(worst, scaled_diff(mine, gold))
        full = runs.BUILD / 'out' / v / kind / f'{s}.csv'
        bitwise += hashlib.sha256(full.read_bytes()).hexdigest() == index.get(f'{v}/{kind}/{s}', {}).get('full_output_sha256')
    rep.add('golden', f'decimated outputs match golden within {PORT_RTOL:g} x field scale, discrete fields exact',
            not missing and not discrete and worst <= 1.0,
            f'worst {worst:.3g} x tolerance; {bitwise}/{len(outputs)} full outputs bit-identical to the golden run'
            + (f'; missing {missing[:3]}' if missing else '') + (f'; discrete {discrete[:3]}' if discrete else ''))


def step_floor(rep):
    """Whole-run floating-point floor: -O0 and FMA builds against -O2 (common parameters)."""
    res = {}
    for tag, flags in (('O0', '-O0'), ('fma', '-O3 -march=native -ffp-contract=fast')):
        r = subprocess.run([str(B / 'build_reference.sh')], env={**__import__('os').environ,
                           'REF_CXXFLAGS': flags, 'REF_TAG': tag}, capture_output=True, text=True)
        if r.returncode:
            rep.add('floor', f'build {tag}', False, r.stdout[-200:])
            continue
        worst, discrete = 0.0, 0
        for v in runs.VARIANTS:
            for s in runs.scenario_names():
                a = runs.read_rows(runs.run(v, 'common', s, binary=BUILD / v / f'reef_ref_{tag}',
                                            out_dir=BUILD / 'out_floor' / tag / v))
                b = runs.read_rows(BUILD / 'out' / v / 'common' / f'{s}.csv')
                discrete += sum(1 for ra, rb in zip(a, b) if any(ra[k] != rb[k] for k in DISCRETE))
                worst = max(worst, scaled_diff(a, b))
        res[tag] = worst
        rep.add('floor', f'{tag} build vs -O2: within port tolerance, discrete fields identical',
                worst <= 1.0 and discrete == 0,
                f'worst {worst * PORT_RTOL:.3g} x field scale ({worst:.3g} x tolerance), discrete divergences {discrete}')
    return res


def step_mutation(rep, outputs):
    """Negative check: a deliberately wrong reference build must be detected by
    both the independent re-derivation and the golden comparison."""
    import os
    env = {**os.environ, 'REF_TAG': 'mut', 'REF_MUTATE': 's/0, 1,  -dt,/0, 1,  dt,/'}   # flip the bias coupling
    r = subprocess.run([str(B / 'build_reference.sh'), 'master'], env=env, capture_output=True, text=True)
    if r.returncode or 'MUTATED' not in r.stdout:
        rep.add('negative', 'mutated build (bias coupling -dt -> +dt)', False, (r.stdout + r.stderr)[-300:])
        return
    s = 's04_accel_bias'
    rows = runs.read_rows(runs.run('master', 'common', s, binary=BUILD / 'master' / 'reef_ref_mut',
                                   out_dir=BUILD / 'out_mutation'))
    st = ind.verify_run('master', runs.parse_params(runs.param_file('common', s, 'master')),
                        runs.parse_events(runs.fixtures_dir() / f'{s}.events'), rows)
    gold_diff = scaled_diff(rows, outputs[('master', 'common', s)])
    rep.add('negative', 'mutated master (F(1,2) = +dt) is REJECTED by the independent check and the golden comparison',
            st.worst > 1 and gold_diff > 1,
            f'independent worst {st.worst:.3g} x tolerance (worst at {st.worst_where}); '
            f'difference from reference {gold_diff:.3g} x port tolerance')


def write_report(rep, elapsed):
    out = BUILD / 'report'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'results.json').write_text(json.dumps(dict(ok=rep.ok, elapsed_s=elapsed, items=rep.items), indent=2))
    lines = ['# P02 baseline check', '', f'Result: **{"PASS" if rep.ok else "FAIL"}** ({elapsed:.0f} s)', '',
             '| Result | Group | Check | Detail |', '|---|---|---|---|']
    lines += [f"| {'PASS' if i['ok'] else 'FAIL'} | {i['group']} | {i['name']} | {i['detail']} |" for i in rep.items]
    (out / 'summary.md').write_text('\n'.join(lines) + '\n')
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--floor', action='store_true', help='also measure the compiler floating-point floor')
    ap.add_argument('--update-golden', metavar='REASON',
                    help='rewrite golden outputs (requires an explained source/algorithm decision)')
    ap.add_argument('--update-lock', action='store_true', help='rewrite fixtures.lock.json')
    a = ap.parse_args()
    t0 = time.time()
    np.seterr(all='ignore')
    rep = Report()
    if a.update_lock:
        subprocess.run([sys.executable, str(B / 'tools' / 'fixtures.py'), str(runs.fixtures_dir())], check=True)
        LOCK.write_text((runs.fixtures_dir() / 'manifest.json').read_text())
    step_fixtures(rep)
    if not step_build(rep):
        write_report(rep, time.time() - t0)
        return 1
    outputs = step_runs(rep)
    if a.update_golden:
        write_golden(outputs, a.update_golden)
        print(f'golden files rewritten: {a.update_golden}')
    step_independent(rep, outputs)
    step_invariants(rep, outputs)
    step_analytic(rep, outputs)
    step_golden(rep, outputs)
    step_mutation(rep, outputs)
    if a.floor:
        step_floor(rep)
    out = write_report(rep, time.time() - t0)
    print(f"{'PASS' if rep.ok else 'FAIL'}: {sum(i['ok'] for i in rep.items)}/{len(rep.items)} assertions; "
          f'report {out.relative_to(ROOT)}/summary.md')
    return 0 if rep.ok else 1


if __name__ == '__main__':
    sys.exit(main())
