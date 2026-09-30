#!/usr/bin/env python3
"""Fault cases F1-F12 of ACCEPTANCE.md section 5 (`faults`).

    check_faults.py --tree TREE --run RUN_DIR

TREE is the per-environment colcon tree (scripts/colcon_tree.py; built,
sourced by the caller). RUN_DIR is a simulation run made with
run_x3_scenario.sh --estimator. Each case checks the specified behaviour;
none passes merely because nothing crashed. Fixture cases compare the port
with the pinned original (parity) and then assert the behaviour on the
port's per-event output.

Exit: 0 all cases pass, 1 a case failed, 2 invalid setup.
"""
import argparse
import csv
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'baseline' / 'tools'))
import check_port as cp  # noqa: E402
import runs  # noqa: E402

T0 = 1_000_000_000
H = 4.8   # T_HOVER of the fixture generators


def ns(t):
    return T0 + round(t * 1e9)


class Cases:
    def __init__(self):
        self.items = []

    def add(self, case, name, ok, detail):
        self.items.append(dict(case=case, name=name, ok=bool(ok), detail=detail))
        print(f"{'PASS' if ok else 'FAIL'} [{case}] {name}: {detail}", flush=True)


def fixture(label):
    for lb, short, params, events in cp.streams():
        if lb == label:
            return params, events
    raise KeyError(label)


def parity(c, case, port_bin, label, out):
    params, events = fixture(label)
    safe = label.replace('/', '__')
    port = cp.run_port(port_bin, params, events, out / 'port' / f'{safe}.csv')
    ref = cp.run_ref('master', params, events, out / 'ref' / f'{safe}.csv')
    node = cp.run_port(port_bin, params, events, out / 'node' / f'{safe}.csv', mode='node')
    res = cp.compare(port, ref)
    wprob, _ = cp.wrapper_equivalence(port, node)
    c.add(case, f'parity and wrapper {label}', res['ok'] and not wprob,
          f"worst {res['worst']:.3g} x tol, {len(res['discrete'])} discrete, node fusions = core: {not wprob}")
    return port


def with_c1(label, out):
    params, events = fixture(label)
    return cp.run_port(PORT, cp.with_c1(params, out / 'c1' / (label.replace('/', '__') + '.params')), events,
                       out / 'c1' / (label.replace('/', '__') + '.csv'))


def f1(c, out):
    lo, hi = ns(H + 1.5), ns(H + 4.5)
    for c1, rows in ((False, parity(c, 'F1', PORT, 'horizontal/h05_mocap_dropout', out)),
                     (True, with_c1('horizontal/h05_mocap_dropout', out))):
        imu = [r for r in rows if r['type'] == 'imu' and lo <= int(r['t_ns']) < hi]
        p00 = [float(r['xyP00']) for r in imu]
        fus = int(imu[-1]['xy_fusions']) - int(imu[0]['xy_fusions'])
        acc = int(imu[-1]['xy_accepted']) - int(imu[0]['xy_accepted'])
        after = next(r for r in rows if r['type'] == 'mocap_twist' and int(r['t_ns']) >= hi)
        before = rows[int(after['idx']) - 1]
        accepted_after = int(after['xy_accepted']) == int(before['xy_accepted']) + 1
        if not c1:
            c.add('F1', 'baseline: stale observation re-fused during the dropout (D1), variance does not grow',
                  fus > 0 and acc == 0 and p00[-1] <= p00[0] and accepted_after,
                  f'{fus} re-fusions, 0 new observations, P_vx {p00[0]:.3g} -> {p00[-1]:.3g}; '
                  f'first observation after the dropout accepted: {accepted_after}')
        else:
            mono = all(b >= a for a, b in zip(p00, p00[1:]))
            c.add('F1', 'C1: no fusion during the dropout, variance grows monotonically',
                  fus == 0 and mono and p00[-1] > p00[0] and accepted_after,
                  f'{fus} fusions, P_vx {p00[0]:.3g} -> {p00[-1]:.3g}, monotonic {mono}; '
                  f'first observation after the dropout accepted: {accepted_after}')


def f2(c, out):
    rows = parity(c, 'F2', PORT, 'horizontal/h03_yaw90_mocap_outliers', out)
    bad = [r for r in rows if r['type'] == 'mocap_twist' and any(int(r['t_ns']) == ns(H + d) for d in (1.5, 2.5, 3.5))]
    ok = len(bad) == 3
    for r in bad:
        prev = rows[int(r['idx']) - 1]
        ok &= (r['xy_gate'] == '1' and float(r['maha2']) > 50 and r['xy_accepted'] == prev['xy_accepted']
               and all(r[k] == prev[k] for k in ('vx', 'vy', 'xy_meas0', 'xy_meas1', 'xy_flag')))
    c.add('F2', 'outliers rejected by the gate; state and flags unchanged by them', ok,
          f"{len(bad)} outliers, maha2 {[round(float(r['maha2'])) for r in bad]} (limit 50)")


def f3_f4(c, out):
    rows = parity(c, 'F3/F4', PORT, 'horizontal/h04_mocap_duplicates_out_of_order', out)
    dups = [r for a, r in zip(rows, rows[1:]) if r['type'] == a['type'] == 'mocap_twist' and r['t_ns'] == a['t_ns']]
    c.add('F3', 'duplicate observations processed as master (both gated; the second supersedes the first)',
          len(dups) >= 100 and all(r['xy_gate'] == '1' for r in dups),
          f'{len(dups)} duplicates, parity above; fusions node = core')
    late = [r for a, r in zip(rows, rows[1:]) if r['type'] == 'mocap_twist' and int(r['t_ns']) < int(a['t_ns']) - 10_000_000]
    c.add('F4', 'out-of-order measurement stamp has no effect beyond arrival order',
          len(late) == 1 and late[0]['delivered'] == '1' and late[0]['xy_gate'] == '1',
          f'{len(late)} late message(s) stamped 20 ms in the past, gated in arrival order (stamps unused, as in master)')


def f5(c, out):
    for label in ('vertical/v05_imu_duplicate_stamp', 'vertical/v06_imu_backward_stamp', 'vertical/v07_imu_gap',
                  'common/s11_imu_nan'):
        rows = parity(c, 'F5', PORT, label, out)
        fin = all(math.isfinite(float(r[k])) for r in rows if r['n_published'] != '0' for k in cp.STATE)
        c.add('F5', f'every output finite ({label})', fin, f'{len(rows)} events')


def f6(c, out):
    rows = parity(c, 'F6', PORT, 'vertical/v02_sonar_dropout', out)
    lo, hi = ns(H + 1.0), ns(H + 3.0)
    win = [r for r in rows if r['type'] == 'imu' and lo <= int(r['t_ns']) < hi]
    z = [float(r['z']) for r in win]
    c.add('F6', 'z propagates during the range dropout', len(win) > 900 and max(z) != min(z) and
          int(win[-1]['n_published']) - int(win[0]['n_published']) == len(win) - 1,
          f'{len(win)} IMU steps, z range {min(z):.4f} .. {max(z):.4f} m')
    rows = parity(c, 'F6', PORT, 'vertical/v03_range_invalid', out)
    invalid = [r for r in rows if r['type'] == 'range' and any(int(r['t_ns']) == ns(H + d) for d in (1.0, 1.5, 2.0, 2.5, 3.0))]
    ok = len(invalid) == 5 and all(r['z_flag'] == rows[int(r['idx']) - 1]['z_flag'] for r in invalid)
    c.add('F6', 'invalid ranges (8.0, NaN, 0.0, -0.5, = max) never gated in', ok,
          f"{len(invalid)} invalid ranges; gated: {[r['z_gate'] for r in invalid]} (8.0 and NaN fail range <= max_range)")


def f7(c, out):
    rows = parity(c, 'F7', PORT, 'common/s02_takeoff_hover', out)
    imu = [r for r in rows if r['type'] == 'imu']
    first = next(k for k, r in enumerate(imu) if r['n_published'] == '1') + 1
    pre = [int(r['n_prop']) for r in imu[20:] if r['takeoff'] == '0']
    cycles = all(b == (a + 1) % 10 for a, b in zip(pre, pre[1:]))
    c.add('F7', 'no output for the first 20 IMU messages; landing reset every 10 propagations before takeoff',
          first == 21 and cycles and len(pre) > 20, f'first estimate at IMU {first}; {len(pre)} steps before takeoff, counter cycles 1..9,0: {cycles}')


def f8(c, out, tree):
    for label in ('vertical/v01_descent_landing', 'horizontal/h08_landing_xy_reset'):
        rows = parity(c, 'F8', PORT, label, out)
        # As in master: the landing transition (setTakeoffState(false)) resets
        # the Z filter to z_x0/z_P0 at once; the landing reset of both filters
        # (counter >= 10, which kept counting while flying) runs at the next IMU
        # step, visible as the propagation counter returning to 0. The XY state
        # itself is not observable at x0 there when an observation is pending,
        # because the update follows the reset in the same step.
        land = [r for a, r in zip(rows, rows[1:]) if a['takeoff'] == '1' and r['takeoff'] == '0']
        ok = len(land) == 1
        detail = f'{len(land)} landing(s)'
        if ok:
            r = land[0]
            nxt = next(x for x in rows[int(r['idx']) + 1:] if x['type'] == 'imu')
            ok = float(r['zP00']) == 0.025 and float(r['z']) == -0.25 and int(nxt['n_prop']) == 0
            detail += f", at landing z {r['z']}, zP00 {r['zP00']}; next IMU step n_prop {nxt['n_prop']} (landing reset of both filters)"
        c.add('F8', f'landing resets: Z at the transition, both filters at the next IMU step ({label})', ok, detail)
    test = Path(tree) / 'build' / 'reef_estimator' / 'test_sensor_manager'
    r = subprocess.run([str(test), '--gtest_filter=*Reset*:*Jump*'], capture_output=True, text=True)
    n = r.stdout.count('[       OK ]')
    c.add('F8', 'reset service, reset = fresh node, backward sim-time jump (node tests)', r.returncode == 0 and n >= 3,
          f'{n} tests passed' + ('' if r.returncode == 0 else f'; {r.stdout[-300:]}'))


def f9(c, out):
    # A second params file that CHANGES the type of a key set by an earlier file
    # is silently ignored by ROS 2's parameter-file merge (the node never sees
    # it; documented finding). The wrong-type case therefore uses a base file
    # without that key.
    cfg = Path(subprocess.run(['ros2', 'pkg', 'prefix', 'reef_estimator'], capture_output=True, text=True).stdout.strip()) \
        / 'share' / 'reef_estimator' / 'config' / 'estimator_master.yaml'
    base = out / 'master_without_channel.yaml'
    base.write_text(''.join(ln for ln in cfg.read_text().splitlines(True) if 'mocap_override_channel' not in ln))
    for name, yaml_text, want in (
            ('wrong length', "/**:\n  ros__parameters:\n    xy_R0: [0.02, 0.0, 0.0]\n", 'xy_R0 has 3 values'),
            ('wrong type', "/**:\n  ros__parameters:\n    mocap_override_channel: 6.0\n", 'mocap_override_channel: wrong type'),
            ('negative variance', "/**:\n  ros__parameters:\n    z_R0: [-0.04]\n", 'z_R0(0,0) is negative')):
        bad = out / f"bad_{name.replace(' ', '_')}.yaml"
        bad.write_text(yaml_text)
        try:
            r = subprocess.run(['ros2', 'run', 'reef_estimator', 'reef_estimator_node', '--ros-args', '-r', '__ns:=/faults_f9',
                                '--params-file', str(base), '--params-file', str(bad)],
                               capture_output=True, text=True, timeout=30)
        except subprocess.TimeoutExpired as e:   # the node started: the invalid value was not rejected
            r = subprocess.CompletedProcess(e.cmd, 124, e.stdout or '', e.stderr or '')
        c.add('F9', f'parameter failure exits 1 naming the parameter ({name})', r.returncode == 1 and want in r.stderr + r.stdout,
              f'exit {r.returncode}; message contains "{want}": {want in r.stderr + r.stdout}')


def f10(c, run):
    for kind in ('clock', 'imu'):
        r = subprocess.run([str(ROOT / 'scripts' / 'check_replay_clock_guard.sh'), str(run), kind],
                           capture_output=True, text=True)
        c.add('F10', f'replay refuses a foreign {kind} publisher in its domain', r.returncode == 0,
              r.stdout.strip().splitlines()[-1] if r.stdout.strip() else f'exit {r.returncode}')


def offline(run, out, extra_args, params=None):
    out.mkdir(parents=True, exist_ok=True)
    cmd = ['ros2', 'run', 'reef_sim', 'x3_reef_offline', str(run), '--out', str(out)] + extra_args
    if params:
        cmd += ['--params', *params]
    return subprocess.run(cmd, capture_output=True, text=True).returncode


def f11_f12(c, run, out):
    from reef_sim import analyze_reef as ar
    from reef_sim.analyze import arrays, read_bag
    phases = {p['name']: p for p in json.loads((run / 'scenario_result.json').read_text())['phases']}
    t0 = phases['forward']['t_start'] + 1.0
    t1 = t0 + 5.0
    cfg = Path(subprocess.run(['ros2', 'pkg', 'prefix', 'reef_estimator'], capture_output=True, text=True).stdout.strip()) \
        / 'share' / 'reef_estimator' / 'config'
    c1_yaml = out / 'c1.yaml'
    c1_yaml.write_text("/**:\n  ros__parameters:\n    correction_c1_clear_xy_flag: true\n")
    params_x3 = __import__('yaml').safe_load((run / 'x3_scenario.yaml').read_text())
    _, data = read_bag(run / 'bag')
    a = arrays(data)
    for c1 in (False, True):
        d = out / ('f11_c1' if c1 else 'f11')
        p = [str(cfg / 'estimator_master.yaml'), str(cfg / 'simulation.yaml')] + ([str(c1_yaml)] if c1 else [])
        rc = offline(run, d, ['--drop', 'velocity', f'{t0}', f'{t1}'], p)
        est, _, _, extra = ar.offline_estimates(d)
        t = est[:, 0]
        (_, _, vlx, vly), inside, _ = ar.truth_at(a, params_x3, t)
        ev = np.hypot(est[:, 4] - vlx, est[:, 5] - vly)
        during = (t >= t0) & (t < t1)
        post = (t >= t1 + 1.0) & (t <= phases['hover_low']['t_end']) & inside
        pvx = est[during, 13]
        fin = bool(np.all(np.isfinite(est[:, 1:])))
        rms_post = float(np.sqrt(np.mean(ev[post] ** 2)))
        rms_during = float(np.sqrt(np.mean(ev[during] ** 2)))
        rows = list(csv.DictReader(open(d / 'estimates.csv')))
        fus_during = [int(r['xy_fusions']) for r in rows if t0 * 1e9 <= int(r['t_ns']) < t1 * 1e9]
        nfus = fus_during[-1] - fus_during[0] if fus_during else 0
        if c1:
            grows = pvx[-1] > pvx[0] and all(b >= a_ for a_, b in zip(pvx, pvx[1:]))
            c.add('F11', 'C1: velocity dropout 5 s: variance grows, finite, error <= 0.10 m/s RMS from 1 s after',
                  rc == 0 and fin and grows and nfus == 0 and rms_post <= 0.10,
                  f'P_vx {pvx[0]:.2e} -> {pvx[-1]:.2e}, {nfus} fusions during, error RMS during {rms_during:.3f}, '
                  f'after {rms_post:.4f} m/s')
        else:
            c.add('F11', 'baseline: velocity dropout 5 s: finite, error <= 0.10 m/s RMS from 1 s after (D1 re-fusion reported)',
                  rc == 0 and fin and rms_post <= 0.10,
                  f'{nfus} re-fusions of the last observation during the dropout (D1); P_vx {pvx[0]:.2e} -> {pvx[-1]:.2e}; '
                  f'error RMS during {rms_during:.3f}, after {rms_post:.4f} m/s')
    d = out / 'f12'
    offline(run, d, ['--drop', 'imu', '0', '1e9'])
    r = subprocess.run(['ros2', 'run', 'reef_sim', 'analyze_reef_vertical', str(run), '--offline', str(d)],
                       capture_output=True, text=True)
    rows = list(csv.DictReader(open(d / 'estimates.csv')))
    c.add('F12', 'missing IMU stream: no estimates, analysis fails', r.returncode == 1 and
          all(x['n_published'] == '0' for x in rows), f'{len(rows)} events, 0 estimates, analysis exit {r.returncode}')
    a1, a2 = out / 'det1', out / 'det2'
    offline(run, a1, [])
    offline(run, a2, [])
    same = (a1 / 'estimates.csv').read_bytes() == (a2 / 'estimates.csv').read_bytes()
    c.add('determinism', 'two offline replays bit-identical', same,
          f"{(a1 / 'estimates.csv').stat().st_size} bytes each; ROS-replay nondeterminism: delivery order (documented)")


def main():
    global PORT, np
    import numpy
    np = numpy
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--tree', type=Path, required=True)
    ap.add_argument('--run', type=Path, required=True)
    a = ap.parse_args()
    PORT = a.tree / 'install' / 'reef_estimator' / 'lib' / 'reef_estimator' / 'reef_estimator_event_replay'
    if not PORT.is_file() or not (a.run / 'bag' / 'metadata.yaml').is_file():
        print('invalid setup: build the tree and pass a finalized --estimator run')
        return 2
    subprocess.run([str(ROOT / 'baseline' / 'build_reference.sh'), 'master'], check=True, stdout=subprocess.DEVNULL)
    for gen, d in ((ROOT / 'baseline/tools/fixtures.py', runs.fixtures_dir()),
                   (ROOT / 'baseline/tools/fixtures_vertical.py', cp.VFIX),
                   (ROOT / 'baseline/tools/fixtures_horizontal.py', cp.HFIX)):
        subprocess.run([sys.executable, str(gen), str(d)], check=True, stdout=subprocess.DEVNULL)
    out = Path(tempfile.mkdtemp(prefix='faults_', dir=runs.BUILD))
    c = Cases()
    f1(c, out); f2(c, out); f3_f4(c, out); f5(c, out); f6(c, out); f7(c, out)
    f8(c, out, a.tree); f9(c, out); f10(c, a.run); f11_f12(c, a.run, out)
    ok = all(i['ok'] for i in c.items)
    (out / 'results.json').write_text(json.dumps(dict(ok=ok, items=c.items, run=str(a.run)), indent=2))
    print(f"{'PASS' if ok else 'FAIL'}: {sum(i['ok'] for i in c.items)}/{len(c.items)} fault assertions; "
          f"report {(out / 'results.json').relative_to(ROOT)}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
