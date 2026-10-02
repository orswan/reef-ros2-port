#!/usr/bin/env python3
"""rgbd_to_velocity fidelity check (P08, ACCEPTANCE.md `vision`).

    check_rgbd.py --port REPLAY [--out DIR]

REPLAY is rgbd_to_velocity_event_replay (sourced ROS 2 environment for node
mode). Builds the reference from the pinned, unmodified b7637198
(baseline/rgbd/build_rgbd_reference.sh), regenerates the fixtures and checks
their lock, then per stream: parity (port core == reference, every column,
byte for byte), node (the ROS 2 node == reference), model (independent
step-wise model vs reference), determinism. Then the parameter case, the
legacy quirks Q1-Q10 (docs/VISION.md section 3), and the negative control
(a reference with (1 - alpha) mutated must fail the model and the port
comparison).
Exit: 0 all PASS, 1 a check failed, 2 blocked.
"""
import argparse
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
import fixtures as fx  # noqa: E402  (imported after the sys.path insert above)
import rgbd_model as rm  # noqa: E402  (imported after the sys.path insert above)

COLUMNS = (HERE / 'rgbd_columns.txt').read_text().split()
INT_COLS = {'idx', 'n_init', 'n_body', 'counter', 'msg_sec', 'msg_nsec', 'init_sec', 'init_nsec'}
STREAMS = [n for n in fx.build() if n.startswith('r')]
MUTATION = r's/(1 - alpha)/(1 + alpha)/'
results = []


def record(group, name, ok, detail):
    results.append({'group': group, 'name': name, 'ok': bool(ok), 'detail': detail})
    print(f"{'PASS' if ok else 'FAIL'} [{group}] {name}: {detail}", flush=True)


def run(cmd, **kw):
    return subprocess.run([str(c) for c in cmd], capture_output=True, text=True, **kw)


def read_rows(path):
    rows = []
    for line in open(path):
        vals = line.rstrip('\n').split(',')
        if len(vals) != len(COLUMNS):
            raise ValueError(f'{path}: {len(vals)} columns, expected {len(COLUMNS)}')
        rows.append(dict(zip(COLUMNS, vals)))
    return rows


def parity(a_rows, b_rows):
    if len(a_rows) != len(b_rows):
        return False, f'{len(b_rows)} rows vs {len(a_rows)}'
    for k, (a, b) in enumerate(zip(a_rows, b_rows)):
        for c in COLUMNS:
            if a[c] != b[c]:
                return False, f'row {k} column {c}: port {b[c]} vs reference {a[c]}'
    return True, f'{len(a_rows)} rows x {len(COLUMNS)} columns identical'


def as_num(r):
    return {c: (r[c] if c == 'kind' else (int(r[c]) if c in INT_COLS else float(r[c]))) for c in COLUMNS}


def close(a, b):
    if math.isnan(a) or math.isnan(b):
        return math.isnan(a) and math.isnan(b)
    if math.isinf(a) or math.isinf(b):
        return a == b
    return abs(a - b) <= 1e-14 + 1e-12 * abs(b)


def model_check(params_text, events_path, ref_rows):
    prev = rm.initial_row(COLUMNS, rm.read_params(params_text))
    events = [ln.split() for ln in Path(events_path).read_text().splitlines() if ln and not ln.startswith('#')]
    worst, acc = 0.0, 0
    for k, (ev, ref) in enumerate(zip(events, ref_rows)):
        pred = rm.step(prev, ev[1:])
        refn = as_num(ref)
        for c in COLUMNS:
            if c in ('idx', 'kind'):
                continue
            a, b = pred[c], refn[c]
            if c in INT_COLS:
                if int(a) != int(b):
                    return False, f'event {k} {c}: model {a} vs reference {b}'
            elif not close(float(a), float(b)):
                return False, f'event {k} {c}: model {a!r} vs reference {b!r}'
            elif math.isfinite(b) and b != 0:
                worst = max(worst, abs(float(a) - b) / abs(b))
        acc += int(refn['n_body'] != prev['n_body'])
        prev = refn
    return True, f'{len(ref_rows)} messages, {acc} accepted; worst relative difference {worst:.1e}'


def F(r, c):
    return float(r[c])


def quirks(ref, fixdir, ref_bin, port, out):
    rows = ref['r01_30hz_exact']
    acc = [int(rows[i]['n_body']) - (int(rows[i - 1]['n_body']) if i else 0) for i in range(len(rows))]
    rej = [r for r, a in zip(rows, acc) if not a]
    good = [r for r, a in zip(rows, acc) if a]
    ok = rej and all(F(r, 'DT') < 1.0 / 30.0 for r in rej) and all(F(r, 'DT') >= 1.0 / 30.0 for r in good) and \
        all(abs(F(r, 'DT') - 0.033333333) < 1e-9 for r in rej) and sum(acc) == len(rows) * 2 // 3
    record('Q', 'Q1 30 Hz with ns stamps: the 33 333 333 ns spacings (< 1/30 s) are rejected, one frame in three', ok,
           f'{sum(acc)} of {len(rows)} accepted; rejected DT {rej[0]["DT"] if rej else "-"}; accepted DT all >= 1/30')
    r0 = ref['r07_first_large_stamp'][0]
    record('Q', 'Q2 first message: DT = the stamp, velocity = position / stamp',
           r0['n_body'] == '1' and F(r0, 'DT') > 1e9 and close(F(r0, 'est_vel_z'), F(r0, 'cur_pos_z') * (1 / F(r0, 'DT'))),
           f"DT {r0['DT']} s, est_vel_z {r0['est_vel_z']} m/s")
    r0 = ref['r01_30hz_exact'][1] if ref['r01_30hz_exact'][0]['n_body'] == '1' else None
    first = ref['r02_rates'][0]
    record('Q', 'Q3 the first velocity is measured from position 0 (never-set pose)',
           close(F(first, 'est_vel_x'), F(first, 'cur_pos_x') * (1 / F(first, 'DT'))),
           f"est_vel_x {first['est_vel_x']} = cur_pos_x {first['cur_pos_x']} / DT {first['DT']}")
    rows = [r for r in ref['r05_alpha_covariance'] if r['n_body'] != '0']
    r = rows[-1]
    record('Q', 'Q4 published x/y covariances are the unrotated parameters (rotated matrix computed)',
           F(r, 'msg_cov0') == 0.02 and F(r, 'msg_cov7') == 0.005 and abs(F(r, 'cov_body_00') - 0.02) > 1e-6,
           f"covariance[0] {r['msg_cov0']}, [7] {r['msg_cov7']}; rotated body-level xx {r['cov_body_00']}")
    # Q5: recompute the camera-level-to-body matrix from each accepted row.
    import numpy as np
    sic, wrong = 0, 0
    acc_rows = [r for i, r in enumerate(ref['r04_tilt']) if i == 0 or r['n_body'] != ref['r04_tilt'][i - 1]['n_body']]
    for r in acc_rows:
        c_init_cam = rm.rot(np.array([F(r, 'beta_x'), F(r, 'beta_y'), F(r, 'beta_z')]), F(r, 'beta_0'))
        c_level = np.array([[F(r, f'C_init_cam_{i}{j}') for j in range(3)] for i in range(3)])
        qb = [F(r, f'q_b2c_{i}') for i in range(4)]
        c_lb = rm.rot(np.array(qb[:3]), qb[3]).T @ rm.FLIP @ c_init_cam @ (rm.M @ c_level).T
        sic += close(F(r, 'roll'), math.atan2(c_lb[0, 2], c_lb[2, 2]))
        wrong += abs(F(r, 'roll') - math.atan2(c_lb[1, 2], c_lb[2, 2])) > 1e-6
    record('Q', 'Q5 the 321 roll is atan2(C02, C22) (sic), not the correct atan2(C12, C22)',
           acc_rows and sic == len(acc_rows) and wrong > 0,
           f'{sic}/{len(acc_rows)} accepted messages match the legacy formula; {wrong} differ from the correct roll')
    rows = ref['r05_alpha_covariance']
    stale = [(rows[i - 1], rows[i]) for i in range(1, len(rows)) if rows[i]['n_init'] != rows[i - 1]['n_init']]
    ok = stale and all(b['init_S_up_0'] == a['S_up_0'] and b['init_cov0'] == a['msg_cov0'] for a, b in stale) and \
        any(b['init_S_up_0'] != b['S_up_0'] for a, b in stale)
    record('Q', 'Q6 the init-frame message carries the previous covariances and bounds', ok,
           f'{len(stale)} init-frame messages: bounds equal the previous body-level message\'s')
    # Q7: the translation parameter has no effect.
    p = (fixdir / 'r03_yaw_motion.params').read_text().splitlines()
    p = [ln if not ln.startswith('body_to_camera_trans') else 'body_to_camera_trans list 1.0 -2.0 3.0' for ln in p]
    alt = out / 'q7.params'
    alt.write_text('\n'.join(p) + '\n')
    run([ref_bin, alt, fixdir / 'r03_yaw_motion.events', out / 'q7.csv'])
    a, b = ref['r03_yaw_motion'], read_rows(out / 'q7.csv')
    diff = {c for ra, rb in zip(a, b) for c in COLUMNS if ra[c] != rb[c]}
    record('Q', 'Q7 body_to_camera_trans (lever arm) does not affect any output', diff <= {'t_b2c_x', 't_b2c_y', 't_b2c_z'},
           f'columns that differ with another translation: {sorted(diff)}')
    rows = ref['r06_stamp_anomalies']
    back = [(rows[i - 1], rows[i]) for i in range(1, len(rows)) if F(rows[i], 'DT') < 0]
    ok = back and all(b['n_body'] == a['n_body'] and b['previous_time_stamp'] == a['previous_time_stamp'] for a, b in back)
    record('Q', 'Q8 rejected messages (backward, duplicate, early) change only counter, stamp, DT', ok,
           f'{len(back)} backward stamps rejected; reference stamp kept')
    rows = ref['r09_nan_position']
    k = next(i for i, r in enumerate(rows) if r['cur_pos_x'] == 'nan')
    later = [r for r in rows[k + 1:] if r['n_body'] != rows[k]['n_body']]
    record('Q', 'Q9 one NaN position latches NaN velocity ((1 - alpha) * NaN, alpha = 1); the position recovers',
           later and all(r['filt_vel_x'] == 'nan' and r['body_vel_x'] == 'nan' for r in later)
           and all(r['prev_pos_x'] != 'nan' for r in later),
           f'{len(later)} later accepted messages: velocity NaN in all, stored position finite again')
    pp, pe = fixdir / 'p01_missing_extrinsics.params', fixdir / 'p01_missing_extrinsics.events'
    r = run([ref_bin, pp, pe, out / 'p01.ref.csv'])
    c = run([port, pp, pe, out / 'p01.core.csv'])
    n = run([port, pp, pe, out / 'p01.node.csv', '--mode', 'node'])
    zeros = r.returncode == 0 and read_rows(out / 'p01.ref.csv')[0]['q_b2c_3'] == '0'
    record('Q', 'Q10 missing extrinsics: the original uses zeros, the port refuses (exit 3, named)',
           zeros and c.returncode == 3 and n.returncode == 3 and 'body_to_camera_quat' in n.stderr,
           f'reference exit {r.returncode} (zero quaternion); port core {c.returncode}, node {n.returncode}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True)
    ap.add_argument('--out', default=str(ROOT / 'build' / 'baseline' / 'rgbd' / 'check'))
    a = ap.parse_args()
    t0 = time.time()
    out = Path(a.out)
    fixdir = out / 'fixtures'
    fixdir.mkdir(parents=True, exist_ok=True)
    port = Path(a.port)
    if not port.exists():
        print(f'BLOCKED: port replay tool not found: {port}')
        return 2
    b = run([HERE / 'build_rgbd_reference.sh'])
    if b.returncode == 2:
        print('BLOCKED: pinned sources unavailable')
        return 2
    record('build', 'reference from pinned, checksummed, unmodified sources', b.returncode == 0,
           (b.stdout.strip().splitlines() or ['?'])[-1])
    if b.returncode:
        return finish(out, t0)
    ref_bin = ROOT / 'build' / 'baseline' / 'rgbd' / 'rgbd_ref'
    g = run([sys.executable, HERE / 'fixtures.py', fixdir, '--lock', HERE / 'fixtures.lock.json'])
    record('fixtures', 'regenerated fixtures match the lock', g.returncode == 0, g.stdout.strip())
    ref = {}
    for name in STREAMS:
        p, e = fixdir / f'{name}.params', fixdir / f'{name}.events'
        paths = {k: out / f'{name}.{k}.csv' for k in ('ref', 'core', 'node', 'core2')}
        r = run([ref_bin, p, e, paths['ref']])
        c = run([port, p, e, paths['core']])
        nd = run([port, p, e, paths['node'], '--mode', 'node'])
        c2 = run([port, p, e, paths['core2']])
        if r.returncode or c.returncode or nd.returncode or c2.returncode:
            record('parity', name, False, f'exit reference {r.returncode}, core {c.returncode}, node {nd.returncode}: '
                   f'{(r.stderr + c.stderr + nd.stderr)[-300:]}')
            continue
        rr = read_rows(paths['ref'])
        ref[name] = rr
        record('parity', name, *parity(rr, read_rows(paths['core'])))
        record('node', name, *parity(rr, read_rows(paths['node'])))
        record('model', name, *model_check(p.read_text(), e, rr))
        record('determinism', name, paths['core'].read_bytes() == paths['core2'].read_bytes(), 'fresh process, identical output')
    if all(n in ref for n in STREAMS):
        quirks(ref, fixdir, ref_bin, port, out)
    else:
        record('Q', 'quirks', False, 'not all streams ran')
    env = dict(os.environ, REF_TAG='negative', REF_MUTATE=MUTATION)
    b = run([HERE / 'build_rgbd_reference.sh'], env=env)
    name = 'r05_alpha_covariance'
    if b.returncode == 0:
        p, e = fixdir / f'{name}.params', fixdir / f'{name}.events'
        m = run([ROOT / 'build' / 'baseline' / 'rgbd' / 'rgbd_ref_negative', p, e, out / 'negative.csv'])
        mrows = read_rows(out / 'negative.csv') if m.returncode == 0 else []
        mok, md = model_check(p.read_text(), e, mrows) if mrows else (True, 'did not run')
        pok, pd = parity(mrows, read_rows(out / f'{name}.core.csv')) if mrows else (True, 'did not run')
        record('negative', '(1 - alpha) mutated in the reference: model and port comparison both fail',
               mrows and not mok and not pok, f'model: {md}; port: {pd}')
    else:
        record('negative', 'mutated reference builds', False, b.stdout.strip())
    return finish(out, t0)


def finish(out, t0):
    ok = all(r['ok'] for r in results)
    (out / 'results.json').write_text(json.dumps({'ok': ok, 'items': results, 'elapsed_s': time.time() - t0}, indent=1))
    print(f"{'PASS' if ok else 'FAIL'}: {sum(r['ok'] for r in results)}/{len(results)} assertions, "
          f"{time.time() - t0:.0f} s; report {out / 'results.json'}")
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
