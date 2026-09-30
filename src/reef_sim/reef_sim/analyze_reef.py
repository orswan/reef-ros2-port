"""Score the REEF estimate of an X3 run against truth: physical plausibility (P04/P05).

    ros2 run reef_sim analyze_reef_vertical RUN_DIR [--offline DIR] [--out DIR] [--no-limits]

(The command name is kept from P04; it now scores the combined estimate.)
Live runs (run_x3_scenario.sh --estimator) are read from RUN_DIR/bag
(/x3/reef/xyz_debug_estimate, /x3/reef/is_flying_reef, /x3/reef/diagnostics).
With --offline DIR the estimates come from DIR/estimates.csv (x3_reef_offline,
the ported core with full covariances) instead; truth and phases always come
from RUN_DIR. Numerical parity with the original is a separate check
(check_port.py); this script judges physical plausibility.

Truth (scoring only, never an estimator input): vertical height of the
range-sensor origin h and vertical velocity v_up, and the velocity in REEF's
body-level frame (NED rotated by yaw), all from /x3/truth/odom, linearly
interpolated to each estimate stamp. Errors: e_z = z + h, e_zdot = z_dot +
v_up, e_vx = x_dot - v_level_x, e_vy = y_dot - v_level_y. The odometry twist
lags by about 10 ms (smoothed finite difference), which is part of the
reported velocity errors. The simulated velocity observations are derived
from the same truth (idealized), so horizontal errors mostly show filter lag
and noise, not the performance of any real velocity sensor.

Initialization interval: from start until 2 s after REEF declares takeoff.
Scoring window: from there to the end of hover_low. Limits: ACCEPTANCE.md
4c/4d. Exit 0 all limits met, 1 otherwise (or with --no-limits always 0).

Consistency (reported, no limit): per state, the fraction of samples with
|e| <= 3 sigma and the mean NEES e^2/sigma^2. Assumptions: truth is exact;
errors are zero-mean Gaussian with the filter's variance. The errors are
strongly autocorrelated, so the effective sample size is estimated with an
AR(1) model, n_eff = n (1 - rho1) / (1 + rho1), and reported with the NEES.
"""
import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

from reef_sim.analyze import Checks, arrays, expected_slant_range, read_bag, rot

INIT_AFTER_TAKEOFF_S = 2.0
LIMITS = dict(z_rmse=0.05, z_peak=0.15, zdot_rmse=0.10, v_rmse=0.10, v_peak=0.30,
              attitude_bias=0.05, accel_bias=0.5, output_fraction=0.5,
              cov_asym_rel=1e-9, cov_min_eig_rel=-1e-9, age_p99_s=0.020,
              callback_over_2ms_fraction=0.01, callback_max_us=20000.0)
T = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
B = np.diag([1.0, -1.0, -1.0])


def labels(params):
    vib = params.get('imu_noise', {}).get('ros__parameters', {}).get('vibration_std', 0.0)
    return ('IDEALIZED INPUTS: attitude = truth; range = idealized (derived from truth); velocity '
            'observations = truth + 0.02 m/s noise (not RGB-D); '
            f'IMU vibration {vib} m/s^2 per axis (scenario assumption). '
            'Controller flies on truth; REEF is not in the loop.')


# ------------------------------------------------------------------ inputs
COLS = ['t', 'z', 'zdot', 'zbias', 'vx', 'vy', 'pitch_bias', 'roll_bias', 'ax_bias', 'ay_bias',
        'Pz', 'Pzdot', 'Pzbias', 'Pvx', 'Pvy', 'Ppb', 'Prb', 'Pab', 'Pbb']


def live_estimates(data):
    rows, zP = [], []
    for tb, m in data.get('/x3/reef/xyz_debug_estimate', []):
        z, xy = m.z_plus, m.xy_plus
        sig = [(xy.sigma_plus[k] - v) / 3.0 for k, v in enumerate(
            (xy.x_dot, xy.y_dot, xy.pitch_bias, xy.roll_bias, xy.xa_bias, xy.ya_bias))]
        rows.append([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, z.z, z.z_dot, z.bias,
                     xy.x_dot, xy.y_dot, xy.pitch_bias, xy.roll_bias, xy.xa_bias, xy.ya_bias,
                     z.p[0], z.p[4], z.p[8]] + [s * s for s in sig] + [tb])
        zP.append(np.array(z.p).reshape(3, 3))
    est = np.array(rows).reshape(-1, len(COLS) + 1)
    recv = est[:, -1]
    flying = [(tb, m.data) for tb, m in data.get('/x3/reef/is_flying_reef', [])]
    imu_t = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in data.get('/x3/reef/imu/data', [])])
    return est[:, :-1], flying, imu_t, dict(recv=recv, P=zP, full=False)


def offline_estimates(directory):
    with open(directory / 'estimates.csv') as fh:
        rows = list(csv.DictReader(fh))
    est, P, flying, last_pub, last_tk, imu_t, first_idx, n_imu = [], [], [], 0, '0', [], None, 0
    for r in rows:
        t = int(r['t_ns']) * 1e-9
        if r['type'] == 'imu':
            imu_t.append(t)
            n_imu += 1
        if int(r['n_published']) > last_pub:
            last_pub = int(r['n_published'])
            if first_idx is None:
                first_idx = n_imu
            f = {k: float(r[k]) for k in r if k != 'type'}
            est.append([t, f['z'], f['zdot'], f['zbias'], f['vx'], f['vy'], f['pitch_bias'], f['roll_bias'],
                        f['ax_bias'], f['ay_bias'], f['zP00'], f['zP11'], f['zP22']]
                       + [f[f'xyP{k}{k}'] for k in range(6)])
            P.append((np.array([[f[f'zP{i}{j}'] for j in range(3)] for i in range(3)]),
                      np.array([[f[f'xyP{i}{j}'] for j in range(6)] for i in range(6)])))
        if r['takeoff'] != last_tk:
            flying.append((t, r['takeoff'] == '1'))
            last_tk = r['takeoff']
    return np.array(est).reshape(-1, len(COLS)), flying, np.array(imu_t), dict(P=P, full=True, first_imu_index=first_idx)


def truth_at(a, params, t):
    odom = a['odom']
    to = odom[:, 0]
    R = rot(odom[:, 5:9])
    offset = np.array(params['range_sensor']['ros__parameters']['sensor_offset'], float)
    _, height, _ = expected_slant_range(odom[:, 2:5], R, offset)
    v_enu = np.einsum('nij,nj->ni', R, odom[:, 9:12])
    v_up = v_enu[:, 2]
    v_ned = v_enu @ T.T
    M = np.einsum('ij,njk,kl->nil', T, R, B)                  # R_NED<-FRD
    psi = np.arctan2(M[:, 1, 0], M[:, 0, 0])
    c, s = np.cos(psi), np.sin(psi)
    vlx = c * v_ned[:, 0] + s * v_ned[:, 1]
    vly = -s * v_ned[:, 0] + c * v_ned[:, 1]
    interp = [np.interp(t, to, x) for x in (height, v_up, vlx, vly)]
    return interp, (t >= to[0]) & (t <= to[-1]), float(np.degrees(np.median(psi)))


def stats(e):
    e = e[np.isfinite(e)]
    if e.size == 0:
        return dict(n=0, rmse=math.nan, peak=math.nan, mean=math.nan)
    return dict(n=int(e.size), rmse=float(np.sqrt(np.mean(e ** 2))), peak=float(np.max(np.abs(e))),
                mean=float(np.mean(e)))


def consistency(e, var):
    m = np.isfinite(e) & (var > 0)
    e, var = e[m], var[m]
    if e.size < 3:
        return dict(n=int(e.size))
    x = e - e.mean()
    rho1 = float(np.dot(x[:-1], x[1:]) / np.dot(x, x)) if np.dot(x, x) > 0 else 0.0
    n_eff = e.size * (1 - rho1) / (1 + rho1) if rho1 < 1 else 1.0
    return dict(n=int(e.size), within_3sigma=float(np.mean(np.abs(e) <= 3 * np.sqrt(var))),
                mean_nees=float(np.mean(e * e / var)), lag1_autocorrelation=rho1, effective_n=float(max(n_eff, 1.0)))


def diagnostics(data):
    msgs = data.get('/x3/reef/diagnostics', [])
    if not msgs:
        return None
    last = {kv.key: kv.value for kv in msgs[-1][1].status[0].values}
    windows = [float({kv.key: kv.value for kv in m.status[0].values}['window_callback_us_p99']) for _, m in msgs]
    total, over = int(last['callbacks_total']), int(last['callbacks_over_2ms'])
    return dict(messages=len(msgs), callbacks_total=total, callbacks_over_2ms=over,
                over_2ms_fraction=over / total if total else math.nan,
                callback_us_max=float(last['callback_us_max']), window_p99_us_max=max(windows),
                xy_observations_accepted=int(last['xy_observations_accepted']),
                xy_fusions=int(last['xy_fusions']), correction_c1=last['correction_c1'])


# ---------------------------------------------------------------- analysis
def analyze(run_dir, offline=None, out=None, apply_limits=True):
    params = yaml.safe_load((run_dir / 'x3_scenario.yaml').read_text())
    phases = json.loads((run_dir / 'scenario_result.json').read_text())['phases']
    _, data = read_bag(run_dir / 'bag')
    a = arrays(data)
    if offline:
        est, flying, imu_t, extra = offline_estimates(offline)
        source = f'offline replay {offline} (ported core)'
    else:
        est, flying, imu_t, extra = live_estimates(data)
        source = 'live /x3/reef topics'
    out = out or ((offline or run_dir) / 'analysis_reef')
    out.mkdir(parents=True, exist_ok=True)
    label = labels(params)
    c = Checks()
    ph = {p['name']: p for p in phases}
    takeoffs = [t for t, f in flying if f]
    t_takeoff = takeoffs[0] if takeoffs else None
    asc = ph.get('ascend')
    t_end = ph['hover_low']['t_end'] if 'hover_low' in ph else (phases[-1]['t_end'] if phases else math.inf)
    t = est[:, 0]
    (h, v_up, vlx, vly), inside, yaw_deg = truth_at(a, params, t)
    E = {'z': est[:, 1] + h, 'zdot': est[:, 2] + v_up, 'vx': est[:, 4] - vlx, 'vy': est[:, 5] - vly}
    V = {'z': est[:, 10], 'zdot': est[:, 11], 'vx': est[:, 13], 'vy': est[:, 14]}
    t_init = (t_takeoff + INIT_AFTER_TAKEOFF_S) if t_takeoff is not None else math.inf
    win = (t >= t_init) & (t <= t_end) & inside

    # Initialization behaviour
    c.add('takeoff declared during ascend', t_takeoff is not None and asc is not None
          and asc['t_start'] <= t_takeoff <= asc['t_end'],
          f'takeoff at {t_takeoff:.3f} s' if t_takeoff is not None else 'no takeoff declared',
          f"within ascend [{asc['t_start']:.2f}, {asc['t_end']:.2f}] s" if asc else 'ascend phase present')
    landings = [x for x, f in flying if not f and t_takeoff is not None and t_takeoff < x <= t_end]
    c.add('no landing declared before the end of hover_low', t_takeoff is not None and not landings,
          f'landings at {landings}' if landings else 'none', 'none')
    if extra.get('first_imu_index') is not None:
        c.add('first estimate after exactly 20 IMU messages', extra['first_imu_index'] == 21,
              f"first estimate at IMU message {extra['first_imu_index']}", '21st IMU message')
    pre = (t < (t_takeoff if t_takeoff is not None else math.inf))
    pre_v = float(np.max(np.abs(est[pre, 4:6]))) if pre.any() else math.nan

    # Outputs and covariance
    finite = bool(np.all(np.isfinite(est[win, 1:]))) if win.any() else False
    n_imu_win = int(np.sum((imu_t >= t_init) & (imu_t <= t_end)))
    frac = float(win.sum() / n_imu_win) if n_imu_win else 0.0
    c.add('outputs finite, count >= 50 % of IMU messages after initialization',
          finite and frac >= LIMITS['output_fraction'],
          f'{int(win.sum())} estimates / {n_imu_win} IMU inputs ({frac:.2f}), finite={finite}', 'finite, >= 0.5')
    asym, mineig = 0.0, math.inf
    for item in extra['P']:
        for Pm in (item if isinstance(item, tuple) else (item,)):
            nrm = np.linalg.norm(Pm)
            if nrm == 0 or not np.all(np.isfinite(Pm)):
                asym = math.inf
                continue
            asym = max(asym, np.linalg.norm(Pm - Pm.T) / nrm)
            mineig = min(mineig, float(np.min(np.linalg.eigvalsh((Pm + Pm.T) / 2))) / nrm)
    which = 'every published P (full Z and XY, core output)' if extra['full'] else 'every published Z covariance p (the messages carry only the XY diagonal)'
    c.add(f'covariance: {which} symmetric and PSD',
          asym <= LIMITS['cov_asym_rel'] and mineig >= LIMITS['cov_min_eig_rel'] and bool(np.all(est[:, 10:] >= 0)),
          f'max |P-P^T|/|P| {asym:.2e}, min eig/|P| {mineig:.2e}, {len(extra["P"])} matrices',
          '<= 1e-9, >= -1e-9, diagonal >= 0')

    # Accuracy
    S = {k: stats(E[k][win]) for k in E}
    c.add('altitude error (z vs -h_sensor)', S['z']['n'] > 0 and S['z']['rmse'] <= LIMITS['z_rmse'] and S['z']['peak'] <= LIMITS['z_peak'],
          f"RMSE {S['z']['rmse']:.4f} m, peak {S['z']['peak']:.4f} m (n={S['z']['n']})",
          f"RMSE <= {LIMITS['z_rmse']} m, peak <= {LIMITS['z_peak']} m")
    c.add('vertical velocity error (z_dot vs -v_up)', S['zdot']['n'] > 0 and S['zdot']['rmse'] <= LIMITS['zdot_rmse'],
          f"RMSE {S['zdot']['rmse']:.4f} m/s, peak {S['zdot']['peak']:.4f} m/s", f"RMSE <= {LIMITS['zdot_rmse']} m/s")
    for k, name in (('vx', 'x (forward)'), ('vy', 'y (right)')):
        c.add(f'horizontal velocity error, body-level {name}',
              S[k]['n'] > 0 and S[k]['rmse'] <= LIMITS['v_rmse'] and S[k]['peak'] <= LIMITS['v_peak'],
              f"RMSE {S[k]['rmse']:.4f} m/s, peak {S[k]['peak']:.4f} m/s, mean {S[k]['mean']:+.4f} m/s",
              f"RMSE <= {LIMITS['v_rmse']} m/s, peak <= {LIMITS['v_peak']} m/s")
    ab = float(np.max(np.abs(est[win, 6:8]))) if win.any() else math.inf
    cb = float(np.max(np.abs(est[win, 8:10]))) if win.any() else math.inf
    c.add('bias plausibility (truth: zero)', ab <= LIMITS['attitude_bias'] and cb <= LIMITS['accel_bias'],
          f'max |attitude bias| {ab:.4f} rad, max |accel bias| {cb:.4f} m/s^2',
          f"<= {LIMITS['attitude_bias']} rad, <= {LIMITS['accel_bias']} m/s^2")

    # Timing
    timing = {}
    if not extra['full']:
        age = extra['recv'][win] - t[win]
        timing['age_s'] = dict(p50=float(np.percentile(age, 50)), p99=float(np.percentile(age, 99)),
                               max=float(np.max(age))) if age.size else {}
        c.add('estimate age (receive - stamp, sim time)', bool(age.size) and timing['age_s']['p99'] <= LIMITS['age_p99_s'],
              f"p50 {timing['age_s'].get('p50', math.nan) * 1e3:.1f} ms, p99 {timing['age_s'].get('p99', math.nan) * 1e3:.1f} ms, "
              f"max {timing['age_s'].get('max', math.nan) * 1e3:.1f} ms", f"p99 <= {LIMITS['age_p99_s'] * 1e3:.0f} ms")
        d = diagnostics(data)
        timing['callbacks'] = d
        c.add('callback wall time (node diagnostics)', d is not None
              and d['over_2ms_fraction'] <= LIMITS['callback_over_2ms_fraction'] and d['callback_us_max'] <= LIMITS['callback_max_us'],
              (f"{d['callbacks_over_2ms']}/{d['callbacks_total']} callbacks > 2 ms (p99 <= 2 ms iff <= 1 %), "
               f"max {d['callback_us_max']:.0f} us, worst window p99 {d['window_p99_us_max']:.0f} us") if d else 'no diagnostics',
              '<= 1 % over 2 ms, max <= 20 ms')

    cons = {k: consistency(E[k][win], V[k][win]) for k in E}
    per_phase = []
    for p in phases:
        m = win & (t >= p['t_start']) & (t < p['t_end'])
        if m.any():
            per_phase.append(dict(phase=p['name'], **{k: stats(E[k][m]) for k in E}))
    report = dict(source=source, labels=label, limits=LIMITS, scoring_window_s=[t_init, t_end],
                  takeoff_s=t_takeoff, transitions=flying, truth_median_yaw_deg=yaw_deg,
                  horizontal_before_takeoff_max_abs=pre_v, checks=c.items, ok=c.ok, overall=S,
                  consistency=cons, consistency_assumptions=(
                      'truth exact; errors zero-mean Gaussian with the filter variance; effective sample size '
                      'from an AR(1) fit of the error sequence'), timing=timing, per_phase=per_phase,
                  estimates=int(len(est)))
    (out / 'report.json').write_text(json.dumps(report, indent=2, default=float))
    plots(t, est, (h, v_up, vlx, vly), E, V, phases, t_init, t_end, t_takeoff, label, source, out)
    lines = [f'# REEF estimate vs truth ({source})', '', f'**{label}**', '',
             f'Truth yaw (median) {yaw_deg:.1f} deg in NED; initialization until t = {t_init:.2f} s; '
             f'scoring window to {t_end:.2f} s. Horizontal state before takeoff: max |v| {pre_v:.4f} m/s '
             '(landing reset every 10 propagations).', '', '| Check | Result | Limit | |', '|---|---|---|---|']
    lines += [f"| {i['name']} | {i['detail']} | {i['criterion']} | {'PASS' if i['ok'] else 'FAIL'} |" for i in c.items]
    lines += ['', '| State | within 3 sigma | mean NEES | lag-1 autocorr. | effective n |', '|---|---|---|---|---|']
    lines += [f"| {k} | {v.get('within_3sigma', math.nan):.3f} | {v.get('mean_nees', math.nan):.3f} | "
              f"{v.get('lag1_autocorrelation', math.nan):.3f} | {v.get('effective_n', math.nan):.0f} |" for k, v in cons.items()]
    lines += ['', f"Consistency assumptions: {report['consistency_assumptions']}. No limit applies.", '',
              '| Phase | z RMSE (m) | z_dot RMSE (m/s) | vx RMSE (m/s) | vy RMSE (m/s) |', '|---|---|---|---|---|']
    lines += [f"| {r['phase']} | {r['z']['rmse']:.4f} | {r['zdot']['rmse']:.4f} | {r['vx']['rmse']:.4f} | "
              f"{r['vy']['rmse']:.4f} |" for r in per_phase]
    (out / 'report.md').write_text('\n'.join(lines) + '\n')
    for i in c.items:
        print(f"{'PASS' if i['ok'] else 'FAIL'} {i['name']}: {i['detail']} ({i['criterion']})")
    print(f'report: {out}')
    return 0 if (c.ok or not apply_limits) else 1


def plots(t, est, truth, E, V, phases, t_init, t_end, t_takeoff, label, source, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    h, v_up, vlx, vly = truth

    def shade(ax):
        for p in phases:
            if p['vz'] or p['vx'] or p['vy']:
                ax.axvspan(p['t_start'], p['t_end'], color='orange', alpha=0.12, lw=0)
        if math.isfinite(t_init):
            ax.axvspan(t[0], t_init, color='gray', alpha=0.25, lw=0, hatch='//')
        if t_takeoff is not None:
            ax.axvline(t_takeoff, color='green', lw=1)
        ax.axvline(t_end, color='k', lw=0.8, ls=':')
        ax.grid(alpha=0.3)

    def banner(fig, title):
        fig.suptitle(f'{title}\n{label}\n({source}; gray hatched: initialization, not scored; '
                     'green: REEF takeoff; orange: commanded motion)', fontsize=7)

    def pair(fname, title, rows):
        fig, ax = plt.subplots(len(rows), 1, figsize=(11, 3 * len(rows)), sharex=True)
        for a_, (tr, es, err, var, ylab, lim) in zip(ax, rows):
            if tr is not None:
                a_.plot(t, tr, 'k', lw=1, label='truth')
                a_.plot(t, es, 'b', lw=1, label='REEF')
                a_.fill_between(t, es - 3 * np.sqrt(var), es + 3 * np.sqrt(var), color='b', alpha=0.15, label='±3σ')
            else:
                a_.plot(t, err, 'r', lw=0.8, label='error')
                a_.fill_between(t, -3 * np.sqrt(var), 3 * np.sqrt(var), color='b', alpha=0.15, label='±3σ')
                a_.set_ylim(-lim, lim)
            a_.set_ylabel(ylab, fontsize=8)
            a_.legend(fontsize=7)
            shade(a_)
        ax[-1].set_xlabel('sim time (s)')
        banner(fig, title)
        fig.savefig(out / fname, dpi=100)
        plt.close(fig)

    pair('altitude.png', 'REEF altitude', [
        (h, -est[:, 1], None, V['z'], 'height (m)', 0),
        (None, None, E['z'], V['z'], 'error z + h (m)', 0.3)])
    pair('vertical_velocity.png', 'REEF vertical velocity', [
        (v_up, -est[:, 2], None, V['zdot'], 'v up (m/s)', 0),
        (None, None, E['zdot'], V['zdot'], 'error (m/s)', 0.6)])
    pair('horizontal_velocity.png', 'REEF horizontal velocity (body-level frame)', [
        (vlx, est[:, 4], None, V['vx'], 'vx forward (m/s)', 0),
        (None, None, E['vx'], V['vx'], 'vx error (m/s)', 0.4),
        (vly, est[:, 5], None, V['vy'], 'vy right (m/s)', 0),
        (None, None, E['vy'], V['vy'], 'vy error (m/s)', 0.4)])
    fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    ax[0].plot(t, est[:, 6], label='pitch bias (rad)')
    ax[0].plot(t, est[:, 7], label='roll bias (rad)')
    ax[1].plot(t, est[:, 8], label='xa bias (m/s²)')
    ax[1].plot(t, est[:, 9], label='ya bias (m/s²)')
    ax[1].plot(t, est[:, 3], label='z bias b (m/s²)')
    for k, name in ((10, 'σ_z'), (11, 'σ_zdot'), (13, 'σ_vx'), (14, 'σ_vy'), (15, 'σ_pitch_bias'), (17, 'σ_xa_bias')):
        ax[2].semilogy(t, np.sqrt(np.maximum(est[:, k], 1e-30)), lw=1, label=name)
    for a_ in ax:
        a_.legend(fontsize=7)
        shade(a_)
    ax[2].set_xlabel('sim time (s)')
    banner(fig, 'REEF biases (truth: zero) and standard deviations')
    fig.savefig(out / 'covariance.png', dpi=100)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('run_dir', type=Path)
    ap.add_argument('--offline', type=Path)
    ap.add_argument('--out', type=Path)
    ap.add_argument('--no-limits', action='store_true', help='report only (characterization)')
    a = ap.parse_args(argv)
    return analyze(a.run_dir, a.offline, a.out, not a.no_limits)


if __name__ == '__main__':
    sys.exit(main())
