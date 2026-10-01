#!/usr/bin/env python3
"""Controller fidelity check (P06, ACCEPTANCE.md `control` P06).

    check_control.py --port REPLAY [--out DIR] [--stream ESTIMATES_CSV]

REPLAY is reef_control_event_replay (sourced ROS 2 environment for node
mode). The script builds the reference harness from the pinned, unmodified
reef_control 12237b76 (baseline/control/build_control_reference.sh),
regenerates the fixtures and checks their lock, then per stream:
  parity  port core == reference, every column, byte for byte
  node    port node == reference (all columns except the command stamp),
          and the node's command stamp == the triggering estimate stamp
  model   independent step-wise model (control_model.py) vs reference
It also runs the parameter cases, the K1-K13 characterizations, a
determinism rerun, and the negative control (a reference with the D-term
sign flipped must fail the model and the port comparison).
--stream adds a stream built from a recorded REEF estimate run: the
estimates.csv of x3_reef_offline (rows where n_published increments; the
state after each IMU step) or any CSV with t_ns or sec/nsec, z, z_dot,
x_dot, y_dot; see stream_from_csv.
Exit: 0 all PASS, 1 a check failed, 2 blocked (build or input missing).
"""
import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
import control_model as cm  # noqa: E402
import fixtures as fx  # noqa: E402

COLUMNS = (HERE / 'control_columns.txt').read_text().split()
INT_COLS = {'idx', 'n_command', 'n_controller_state', 'cmd_sec', 'cmd_nsec', 'cmd_mode', 'cmd_ignore',
            'cs_sec', 'cs_nsec', 'ds_sec', 'ds_nsec', 'st_sec', 'st_nsec', 'armed', 'initialized', 'is_flying',
            'prev_sec', 'prev_nsec', 'face_target', 'fly_fixed_wing'} | {
    p + f for p in ('cs_', 'ds_') for f in ('attitude_valid', 'position_valid', 'velocity_valid',
                                            'acceleration_valid', 'altitude_only')}
NOT_MODELLED = {'idx', 'kind', 'cmd_sec', 'cmd_nsec'}
F32_COLS = {'cmd_x', 'cmd_y', 'cmd_z', 'cmd_F'}   # printed with %.9g, which round-trips float32
STREAMS = [n for n in fx.build() if n.startswith('c')]
MUTATION = r's/d_term = kd_ \* x_dot;/d_term = -kd_ * x_dot;/'

results = []


def record(group, name, ok, detail):
    results.append({'group': group, 'name': name, 'ok': bool(ok), 'detail': detail})
    print(f"{'PASS' if ok else 'FAIL'} [{group}] {name}: {detail}", flush=True)


def blocked(msg):
    print(f'BLOCKED: {msg}')
    sys.exit(2)


def run(cmd, **kw):
    return subprocess.run([str(c) for c in cmd], capture_output=True, text=True, **kw)


def read_rows(path):
    rows = []
    with open(path) as f:
        for line in f:
            vals = line.rstrip('\n').split(',')
            if len(vals) != len(COLUMNS):
                raise ValueError(f'{path}: {len(vals)} columns, expected {len(COLUMNS)}')
            rows.append(dict(zip(COLUMNS, vals)))
    return rows


def num(v):
    return float(v)


def parity(ref_rows, port_rows, skip=()):
    if len(ref_rows) != len(port_rows):
        return False, f'{len(port_rows)} rows vs {len(ref_rows)}'
    for k, (a, b) in enumerate(zip(ref_rows, port_rows)):
        for c in COLUMNS:
            if c not in skip and a[c] != b[c]:
                return False, f'row {k} ({a["kind"]}) column {c}: port {b[c]} vs reference {a[c]}'
    return True, f'{len(ref_rows)} rows x {len(COLUMNS) - len(skip)} columns identical'


def node_stamps(rows):
    """Where a command was published, its stamp is the estimate's."""
    prev = 0
    n = 0
    for k, r in enumerate(rows):
        c = int(r['n_command'])
        if c != prev:
            n += 1
            if (r['cmd_sec'], r['cmd_nsec']) != (r['st_sec'], r['st_nsec']):
                return False, f'row {k}: command stamp {r["cmd_sec"]}.{r["cmd_nsec"]} vs estimate {r["st_sec"]}.{r["st_nsec"]}'
        prev = c
    return True, f'{n} commands stamped with their estimate'


def to_model_row(r):
    out = {}
    for c in COLUMNS:
        if c in ('kind',):
            out[c] = r[c]
        elif c in INT_COLS:
            out[c] = int(r[c])
        else:
            out[c] = num(r[c])
    return out


def close(a, b, tol_abs=1e-14, tol_rel=1e-12):
    if math.isnan(a) or math.isnan(b):
        return math.isnan(a) and math.isnan(b)
    if math.isinf(a) or math.isinf(b):
        return a == b
    return abs(a - b) <= tol_abs + tol_rel * abs(b)


def model_check(params_text, events_path, ref_rows):
    store = cm.load_params(params_text)
    cfg = cm.initial_config(store)
    prev = cm.initial_row(COLUMNS, store, cfg)
    events = [ln.split() for ln in Path(events_path).read_text().splitlines() if ln and not ln.startswith('#')]
    worst = 0.0
    steps = 0
    for k, (ev, ref) in enumerate(zip(events, ref_rows)):
        pred = cm.step(prev, ev[0], ev[1:], cfg)
        refm = to_model_row(ref)
        for c in COLUMNS:
            if c in NOT_MODELLED:
                continue
            a, b = pred[c], refm[c]
            if c in INT_COLS:
                if int(a) != int(b):
                    return False, f'event {k} ({ev[0]}) {c}: model {a} vs reference {b}', steps
            elif c in F32_COLS:
                fa, fb = cm.f32(float(a)), cm.f32(float(b))
                if not (fa == fb or (math.isnan(fa) and math.isnan(fb))):
                    return False, f'event {k} ({ev[0]}) {c}: model {fa!r} vs reference {fb!r} (float32)', steps
            elif not close(float(a), float(b)):
                return False, f'event {k} ({ev[0]}) {c}: model {a!r} vs reference {b!r}', steps
            elif math.isfinite(float(b)) and float(b) != 0:
                worst = max(worst, abs(float(a) - float(b)) / abs(float(b)))
        steps += int(refm['n_command'] != prev['n_command'])
        prev = refm   # next prediction starts from the reference state
    return True, f'{len(ref_rows)} events, {steps} control steps; worst relative difference {worst:.1e}', steps


def rows_of(rows, kind=None):
    return [r for r in rows if kind is None or r['kind'] == kind]


def step_rows(rows):
    out, prev = [], 0
    for r in rows:
        c = int(r['n_command'])
        if c != prev:
            out.append(r)
        prev = c
    return out


def characterize(ref, params):
    F = lambda r, c: float(r[c])  # noqa: E731

    # K1: D enters with a plus sign on the derivative of the state.
    rows = [r for r in step_rows(ref['c03_velocity_mode'])[1:] if abs(F(r, 'ds_acc_x')) < F(r, 'u_max')]
    plus = sum(close(F(r, 'ds_acc_x'), F(r, 'u_kp') * (F(r, 'ds_vel_x') - F(r, 'st_vx')) + F(r, 'u_ki') * F(r, 'u_integrator')
                     + F(r, 'u_kd') * F(r, 'u_differentiator'), 1e-12, 1e-9) for r in rows)
    minus = sum(close(F(r, 'ds_acc_x'), F(r, 'u_kp') * (F(r, 'ds_vel_x') - F(r, 'st_vx')) + F(r, 'u_ki') * F(r, 'u_integrator')
                      - F(r, 'u_kd') * F(r, 'u_differentiator'), 1e-12, 1e-9) for r in rows)
    record('K', 'K1 D term = +kd * d(state)/dt (anti-damping)', rows and plus == len(rows) and minus < len(rows) // 10,
           f'{plus}/{len(rows)} unsaturated steps match +kd, {minus} match -kd')

    # K2: saturated climb with wD = 0: no back-calculation; the I term alone exceeds max_w.
    rows = step_rows(ref['c02_climb_saturation'])
    sat = [r for r in rows if float(r['cmd_F']) == 1.0]
    peak = max(abs(F(r, 'w_ki') * F(r, 'w_integrator')) for r in sat) if sat else 0
    record('K', 'K2 anti-windup inactive when the D term is 0', sat and peak > F(sat[0], 'w_max'),
           f'{len(sat)} saturated steps; peak |I term| {peak:.3f} > max_w {F(sat[0], "w_max") if sat else "?"}')

    # K3: first-sample derivative kick (last state starts at 0).
    r = step_rows(ref['c03_velocity_mode'])[0]
    tau, dt, x = F(r, 'u_tau'), F(r, 'dt'), F(r, 'st_vx')
    expect = 2 / (2 * tau + dt) * x
    record('K', 'K3 first derivative from last_state = 0', x != 0 and close(F(r, 'u_differentiator'), expect),
           f'differentiator {F(r, "u_differentiator"):.3e} = 2/(2tau+dt)*x = {expect:.3e}')

    # K4: first step dt = stamp - 0.
    r = step_rows(ref['c12_first_step_huge_dt'])[0]
    record('K', 'K4 first step uses dt = stamp (wall-clock stamps: ~1.8e9 s)', F(r, 'dt') > 1e9,
           f'dt = {F(r, "dt"):.6e} s, command published')

    # K5: commands while unarmed.
    rows = ref['c16_no_desired_unarmed']
    record('K', 'K5 no output inhibition: commands while unarmed',
           int(rows[-1]['n_command']) > 0 and all(r['armed'] == '0' for r in rows),
           f'{rows[-1]["n_command"]} commands, armed never set')

    # K6: NaN latch with kd > 0.
    rows = ref['c14_nan_estimate']
    k = next(i for i, r in enumerate(rows) if r['st_vz'] == 'nan')
    after = step_rows(rows[k:])
    record('K', 'K6 one NaN estimate latches NaN throttle (kd > 0)',
           after and all(r['cmd_F'] == 'nan' and r['w_differentiator'] == 'nan' for r in after[1:]),
           f'{len(after)} steps from the NaN on; throttle NaN in all later steps')

    # K7: altitude-only keeps stale x, y, z.
    rows = step_rows(ref['c09_altitude_only'])
    first = next(i for i, r in enumerate(rows) if r['cmd_ignore'] == '7')
    last_before = rows[first - 1]
    stale = all((r['cmd_x'], r['cmd_y'], r['cmd_z']) == (last_before['cmd_x'], last_before['cmd_y'], last_before['cmd_z'])
                for r in rows[first:])
    record('K', 'K7 altitude-only: ignore 0x07, stale x/y/z', stale and len(rows) - first > 10,
           f'{len(rows) - first} steps keep x, y, z = {last_before["cmd_x"]}, {last_before["cmd_y"]}, {last_before["cmd_z"]}')

    # K8: attitude mode not clamped; yaw sent as a rate.
    r = step_rows(ref['c08_attitude_mode'])[0]
    record('K', 'K8 attitude mode: angles not clamped, attitude.yaw in the yaw-rate field',
           cm.f32(F(r, 'cmd_x')) == cm.f32(0.4) and F(r, 'cmd_x') > F(r, 'max_roll') and cm.f32(F(r, 'cmd_z')) == cm.f32(1.3),
           f'x {r["cmd_x"]} (max_roll {r["max_roll"]}), z {r["cmd_z"]} = attitude.yaw')

    # K9: heading without wrapping.
    r = step_rows(ref['c07_heading_wrap'])[0]
    record('K', 'K9 heading error not wrapped (long way round)', F(r, 'ds_vel_yaw') > 0,
           f'desired 3.1, current {F(r, "current_yaw"):.3f}: yaw rate {F(r, "ds_vel_yaw"):+.3f} (wrapped error would be negative)')

    # K10: face_target uses theta of the previous step, 0 at first.
    rows = step_rows(ref['c05_face_target'])
    ok = close(F(rows[0], 'ds_pose_yaw'), 0.0 + F(rows[0], 'current_yaw')) and \
        close(F(rows[1], 'ds_pose_yaw'), F(rows[0], 'theta') + F(rows[1], 'current_yaw'))
    record('K', 'K10 face_target: theta from the previous step (0 before the first lookup)', ok,
           f'first pose.yaw {F(rows[0], "ds_pose_yaw"):.6f} = current yaw; second uses the first step\'s theta')

    # K11: gain changes keep the integrators; integrator switch off freezes the u integrator.
    rows = ref['c15_gain_changes']
    kept = all(rows[i]['u_integrator'] == rows[i - 1]['u_integrator'] and rows[i]['w_integrator'] == rows[i - 1]['w_integrator']
               for i, r in enumerate(rows) if r['kind'] == 'gain')
    k = next(i for i, r in enumerate(rows) if r['kind'] == 'gain' and float(r['u_ki']) == 0.0)
    frozen = step_rows(rows[k:k + 150])
    ok2 = len({r['u_integrator'] for r in frozen}) == 1 and float(frozen[0]['u_integrator']) != 0
    record('K', 'K11 gain changes keep integrators; uIntegrator off freezes (not clears) them', kept and ok2,
           f'integrators unchanged at every gain event; u integrator frozen at {frozen[0]["u_integrator"]}')

    # K12: face_target read globally.
    record('K', 'K12 face_target/fly_fixed_wing read from the global namespace',
           ref['c17_kiwi_global_flags'][0]['face_target'] == '1' and ref['c18_quad_namespaced_flags'][0]['face_target'] == '0'
           and ref['c18_quad_namespaced_flags'][0]['fly_fixed_wing'] == '0',
           'kiwi file (global): true; quad file (inside reef_control_pid): ignored -> false')

    # Missing input: no estimate, no command.
    rows = ref['c19_no_estimate']
    record('K', 'no estimate -> no command', rows[-1]['n_command'] == '0', f'{len(rows)} events, 0 commands')


def cfg_tables():
    """The Gains.cfg table in three hand transcriptions (harness stand-in,
    port, model) against the pinned file itself."""
    import re
    cfg = (ROOT / 'build' / 'baseline' / 'control' / 'tree' / 'reef_control' / 'cfg' / 'Gains.cfg').read_text()
    truth = {}
    for m in re.finditer(r'gen\.add\("(\w+)",\s*(double|bool)_t,\s*0,\s*"[^"]*",\s*([^,)]+)(?:,\s*([^,)]+),\s*([^,)]+))?\)', cfg):
        name, kind, d, lo, hi = m.groups()
        truth[name] = ('bool', d.strip() == 'True') if kind == 'bool' else ('double', float(d), float(lo), float(hi))
    shim = {}
    text = (HERE / 'shim' / 'reef_control' / 'GainsConfig.h').read_text()
    for m in re.finditer(r'X\((\w+), (double|bool), ([\w.]+), ([\w.]+), ([\w.]+)\)', text):
        n, kind, d, lo, hi = m.groups()
        shim[n] = ('bool', d == 'true') if kind == 'bool' else ('double', float(d), float(lo), float(hi))
    port = {}
    text = (ROOT / 'src' / 'reef_control' / 'include' / 'reef_control' / 'gains.hpp').read_text()
    for m in re.finditer(r'X\((\w+), ([\d.]+), ([\d.]+), ([\d.]+)\)', text):
        port[m.group(1)] = ('double', *(float(x) for x in m.groups()[1:]))
    for m in re.finditer(r'X\((\w+), (true|false)\)', text):
        port[m.group(1)] = ('bool', m.group(2) == 'true')
    model = {n: ('double', float(d), float(lo), float(hi)) for n, (d, lo, hi) in cm.CFG.items()}
    model.update({n: ('bool', v) for n, v in cm.CFG_BOOL.items()})
    record('cfg', 'harness GainsConfig stand-in == Gains.cfg', shim == truth, f'{len(truth)} fields')
    record('cfg', 'model table == Gains.cfg', model == truth, f'{len(truth)} fields')
    unused = {'yawRateP', 'yawRateI', 'yawRateD', 'yawRatetau', 'max_n', 'max_e', 'max_yaw_rate'}
    sub = {n: v for n, v in truth.items() if n not in unused}
    record('cfg', 'port gain table == Gains.cfg minus the fields the original never read', port == sub,
           f'{len(port)} fields; not declared: {", ".join(sorted(unused))}')


def stream_from_csv(path, out_dir):
    """Estimate events from a recorded run (reef_offline/estimates.csv or any
    CSV with sec,nsec or t_ns and z, z_dot, x_dot, y_dot), with a scripted
    setpoint sequence: altitude hold, velocity mode, position mode, attitude."""
    rows = list(csv.DictReader(open(path)))
    if not rows:
        blocked(f'{path}: no rows')
    keys = rows[0].keys()
    if 'n_published' in keys:   # x3_reef_offline estimates.csv: one row per input event
        pub, last = [], '0'
        for r in rows:
            if r['n_published'] != last:
                pub.append(r)
            last = r['n_published']
        rows = pub

    def stamp(r):
        if 't_ns' in keys:
            t = int(r['t_ns'])
            return t // 1_000_000_000, t % 1_000_000_000
        return int(r['sec']), int(r['nsec'])

    def val(r, *names):
        for n in names:
            if n in r:
                return r[n]
        raise KeyError(names)
    ev = ['status 1', fx.des(pose=(0, 0, -1.0, 0))]
    n = len(rows)
    for i, r in enumerate(rows):
        if i == n // 4:
            ev.append(fx.des(vv=1, pose=(0, 0, -1.2, 0), vel=(0.3, -0.2, 0, 0.1)))
        if i == n // 2:
            ev += [fx.pose(0.2, 0.1, 0.05), fx.des(pv=1, pose=(1.5, 0.5, -1.2, 0.3))]
        if i == 3 * n // 4:
            ev.append(fx.des(av=1, pose=(0, 0, -1.0, 0), att=(0.05, -0.05, 0, 0.1)))
        s, ns = stamp(r)
        ev.append(f'est {s} {ns} {val(r, "z", "z_plus.z")} {val(r, "z_dot", "zdot", "z_plus.z_dot")} '
                  f'{val(r, "x_dot", "vx", "xy_plus.x_dot")} {val(r, "y_dot", "vy", "xy_plus.y_dot")}')
    (out_dir / 's01_recorded_estimates.params').write_text(fx.params(fx.QUAD))
    (out_dir / 's01_recorded_estimates.events').write_text('\n'.join(ev) + '\n')
    return 's01_recorded_estimates', len(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True)
    ap.add_argument('--out', default=str(ROOT / 'build' / 'baseline' / 'control' / 'check'))
    ap.add_argument('--stream', help='recorded estimates CSV for an extra stream')
    a = ap.parse_args()
    t0 = time.time()
    out = Path(a.out)
    fixdir = out / 'fixtures'
    fixdir.mkdir(parents=True, exist_ok=True)
    port = Path(a.port)
    if not port.exists():
        blocked(f'port replay tool not found: {port}')

    b = run([HERE / 'build_control_reference.sh'])
    if b.returncode == 2:
        blocked('pinned reef_control sources unavailable: ' + b.stdout.strip())
    record('build', 'reference from pinned, checksummed, unmodified sources', b.returncode == 0, b.stdout.strip().splitlines()[-1])
    if b.returncode:
        return finish(out, t0)
    ref_bin = ROOT / 'build' / 'baseline' / 'control' / 'control_ref'

    g = run([sys.executable, HERE / 'fixtures.py', fixdir, '--lock', HERE / 'fixtures.lock.json'])
    record('fixtures', 'regenerated fixtures match the lock', g.returncode == 0, g.stdout.strip())
    streams = list(STREAMS)
    if a.stream:
        name, n = stream_from_csv(a.stream, fixdir)
        streams.append(name)
        record('fixtures', 'recorded estimate stream', True, f'{a.stream}: {n} estimates')

    ref = {}
    for name in streams:
        p, e = fixdir / f'{name}.params', fixdir / f'{name}.events'
        paths = {k: out / f'{name}.{k}.csv' for k in ('ref', 'core', 'node', 'core2')}
        r = run([ref_bin, p, e, paths['ref']])
        c = run([port, p, e, paths['core']])
        nd = run([port, p, e, paths['node'], '--mode', 'node'])
        c2 = run([port, p, e, paths['core2']])
        if r.returncode or c.returncode or nd.returncode:
            record('parity', name, False, f'exit reference {r.returncode}, core {c.returncode}, node {nd.returncode}: '
                   f'{(r.stderr + c.stderr + nd.stderr).strip()[-300:]}')
            continue
        ref_rows = read_rows(paths['ref'])
        ref[name] = ref_rows
        ok, d = parity(ref_rows, read_rows(paths['core']))
        record('parity', name, ok, d)
        node_rows = read_rows(paths['node'])
        ok, d = parity(ref_rows, node_rows, skip={'cmd_sec', 'cmd_nsec'})
        ok2, d2 = node_stamps(node_rows)
        record('node', name, ok and ok2, f'{d}; {d2}')
        ok, d, _ = model_check(p.read_text(), e, ref_rows)
        record('model', name, ok, d)
        record('determinism', name, paths['core'].read_bytes() == paths['core2'].read_bytes(), 'fresh process, identical output')

    # Parameter cases.
    p, e = fixdir / 'k13_out_of_range_gain.params', fixdir / 'k13_out_of_range_gain.events'
    r = run([ref_bin, p, e, out / 'k13.ref.csv'])
    c = run([port, p, e, out / 'k13.core.csv'])
    nd = run([port, p, e, out / 'k13.node.csv', '--mode', 'node'])
    clamped = r.returncode == 0 and read_rows(out / 'k13.ref.csv')[0]['u_kp'] == '2'
    record('params', 'K13 out-of-range gain: original clamps, port refuses (exit 3, named)',
           clamped and c.returncode == 3 and nd.returncode == 3 and 'uP' in c.stderr and 'uP' in nd.stderr,
           f'reference exit {r.returncode} uP -> 2 (clamped); port core exit {c.returncode}, node exit {nd.returncode}')
    p, e = fixdir / 'p01_missing_max_roll.params', fixdir / 'p01_missing_max_roll.events'
    r = run([ref_bin, p, e, out / 'p01.ref.csv'])
    c = run([port, p, e, out / 'p01.core.csv'])
    nd = run([port, p, e, out / 'p01.node.csv', '--mode', 'node'])
    record('params', 'missing max_roll: both stop before any output (exit 3, named)',
           r.returncode == 3 and c.returncode == 3 and nd.returncode == 3 and 'max_roll' in nd.stderr,
           f'reference exit {r.returncode} (ROS_ASSERT), port core {c.returncode}, node {nd.returncode}')

    cfg_tables()
    if all(n in ref for n in STREAMS):
        characterize(ref, None)
    else:
        record('K', 'characterizations', False, 'not all streams ran')

    # Negative control: D-term sign flipped in the reference.
    env = dict(os.environ, REF_TAG='negative', REF_MUTATE=MUTATION)
    b = run([HERE / 'build_control_reference.sh'], env=env)
    name = 'c03_velocity_mode'
    if b.returncode == 0:
        p, e = fixdir / f'{name}.params', fixdir / f'{name}.events'
        m = run([ROOT / 'build' / 'baseline' / 'control' / 'control_ref_negative', p, e, out / 'negative.csv'])
        mrows = read_rows(out / 'negative.csv') if m.returncode == 0 else []
        mok, md, _ = model_check(p.read_text(), e, mrows) if mrows else (True, 'did not run', 0)
        pok, pd = parity(mrows, read_rows(out / f'{name}.core.csv')) if mrows else (True, 'did not run')
        record('negative', 'D-term sign flipped: model and port comparison both fail', mrows and not mok and not pok,
               f'model: {md}; port: {pd}')
    else:
        record('negative', 'mutated reference builds', False, b.stdout.strip())
    return finish(out, t0)


def finish(out, t0):
    ok = all(r['ok'] for r in results)
    n = sum(r['ok'] for r in results)
    (out / 'results.json').write_text(json.dumps({'ok': ok, 'items': results, 'elapsed_s': time.time() - t0}, indent=1))
    print(f"{'PASS' if ok else 'FAIL'}: {n}/{len(results)} assertions, {time.time() - t0:.0f} s; report {out / 'results.json'}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
