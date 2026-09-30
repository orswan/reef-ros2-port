"""Score the REEF vertical estimate of an X3 run against truth (P04).

    ros2 run reef_sim analyze_reef_vertical RUN_DIR [--offline DIR] [--out DIR] [--no-limits]

Live runs (run_x3_scenario.sh --estimator) are read from RUN_DIR/bag
(/x3/reef/xyz_debug_estimate, /x3/reef/is_flying_reef). With --offline DIR
the estimates come from DIR/estimates.csv (x3_reef_offline) instead; truth
and phases always come from RUN_DIR.

Truth (scoring only, never an estimator input): the vertical height of the
range-sensor origin, h = p_z + (R r_s)_z, and the vertical velocity of the
body, v_up = (R v_body)_z, from /x3/truth/odom, linearly interpolated to each
estimate stamp. REEF's z is NED (down), so the errors are e_z = z + h and
e_v = z_dot + v_up. The odometry twist lags by about 10 ms (smoothed finite
difference, X3_SCENARIO.md), which is part of the reported velocity error.

Initialization interval: from start until 2 s after REEF declares takeoff.
Scoring window: from there to the end of hover_low. Limits: ACCEPTANCE.md
section 4c. Exit 0 all limits met, 1 otherwise (or with --no-limits always 0).

IDEALIZED INPUTS: attitude from truth, range derived from truth, and the IMU
vibration scenario assumption; the controller flies on truth and REEF is not
in the loop. Every plot and report says so.
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
LIMITS = dict(z_rmse=0.05, z_peak=0.15, v_rmse=0.10, output_fraction=0.5)


def labels(params):
    vib = params.get('imu_noise', {}).get('ros__parameters', {}).get('vibration_std', 0.0)
    return ('IDEALIZED INPUTS: attitude = truth; range = idealized (derived from truth); '
            f'IMU vibration {vib} m/s^2 per axis (scenario assumption). '
            'Controller flies on truth; REEF is not in the loop.')


def live_estimates(data):
    dbg = data.get('/x3/reef/xyz_debug_estimate', [])
    est = np.array([(m.header.stamp.sec * 1e9 + m.header.stamp.nanosec, m.z_plus.z, m.z_plus.z_dot,
                     m.z_plus.bias, m.z_plus.p[0], m.z_plus.p[4], m.z_plus.p[8]) for _, m in dbg]).reshape(-1, 7)
    est[:, 0] *= 1e-9
    flying = [(tb, m.data) for tb, m in data.get('/x3/reef/is_flying_reef', [])]
    n_inputs = len(data.get('/x3/reef/imu/data', []))
    imu_t = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                      for _, m in data.get('/x3/reef/imu/data', [])])
    return est, flying, n_inputs, imu_t


def offline_estimates(directory):
    with open(directory / 'estimates.csv') as fh:
        rows = list(csv.DictReader(fh))
    est, flying, last_pub, last_tk, imu_t = [], [], 0, '0', []
    for r in rows:
        t = int(r['t_ns']) * 1e-9
        if r['type'] == 'imu':
            imu_t.append(t)
        if int(r['n_published']) > last_pub:
            last_pub = int(r['n_published'])
            est.append((t, float(r['z']), float(r['zdot']), float(r['zbias']),
                        float(r['zP00']), float(r['zP11']), float(r['zP22'])))
        if r['takeoff'] != last_tk:
            flying.append((t, r['takeoff'] == '1'))
            last_tk = r['takeoff']
    return np.array(est).reshape(-1, 7), flying, len(imu_t), np.array(imu_t)


def truth_at(a, params, t):
    odom = a['odom']
    to = odom[:, 0]
    R = rot(odom[:, 5:9])
    offset = np.array(params['range_sensor']['ros__parameters']['sensor_offset'], float)
    _, height, _ = expected_slant_range(odom[:, 2:5], R, offset)
    v_up = np.einsum('nj,nj->n', R[:, 2, :], odom[:, 9:12])
    return np.interp(t, to, height), np.interp(t, to, v_up), (t >= to[0]) & (t <= to[-1])


def stats(e):
    e = e[np.isfinite(e)]
    if e.size == 0:
        return dict(n=0, rmse=math.nan, peak=math.nan, mean=math.nan)
    return dict(n=int(e.size), rmse=float(np.sqrt(np.mean(e ** 2))), peak=float(np.max(np.abs(e))),
                mean=float(np.mean(e)))


def analyze(run_dir, offline=None, out=None, apply_limits=True):
    params = yaml.safe_load((run_dir / 'x3_scenario.yaml').read_text())
    result = json.loads((run_dir / 'scenario_result.json').read_text())
    phases = result['phases']
    types, data = read_bag(run_dir / 'bag')
    a = arrays(data)
    if offline:
        est, flying, n_inputs, imu_t = offline_estimates(offline)
        source = f'offline replay {offline}'
    else:
        est, flying, n_inputs, imu_t = live_estimates(data)
        source = 'live /x3/reef/xyz_debug_estimate'
    out = out or ((offline or run_dir) / 'analysis_reef')
    out.mkdir(parents=True, exist_ok=True)
    label = labels(params)
    c = Checks()
    ph = {p['name']: p for p in phases}
    takeoffs = [t for t, f in flying if f]
    t_takeoff = takeoffs[0] if takeoffs else None
    asc = ph.get('ascend')
    c.add('takeoff declared during ascend', t_takeoff is not None and asc is not None
          and asc['t_start'] <= t_takeoff <= asc['t_end'],
          f'takeoff at {t_takeoff}' if t_takeoff is not None else 'no takeoff declared',
          f"within ascend [{asc['t_start']:.2f}, {asc['t_end']:.2f}] s" if asc else 'ascend phase present')
    t_end = ph['hover_low']['t_end'] if 'hover_low' in ph else (phases[-1]['t_end'] if phases else math.inf)
    landings = [t for t, f in flying if not f and t_takeoff is not None and t_takeoff < t <= t_end]
    c.add('no landing declared before the end of hover_low', t_takeoff is not None and not landings,
          f'landings at {landings}' if landings else 'none', 'none')

    t = est[:, 0]
    h, v_up, inside = truth_at(a, params, t)
    e_z, e_v = est[:, 1] + h, est[:, 2] + v_up
    s_z, s_v = np.sqrt(np.maximum(est[:, 4], 0)), np.sqrt(np.maximum(est[:, 5], 0))
    t_init = (t_takeoff + INIT_AFTER_TAKEOFF_S) if t_takeoff is not None else math.inf
    win = (t >= t_init) & (t <= t_end) & inside
    finite = bool(np.all(np.isfinite(est[win, 1:]))) if win.any() else False
    n_imu_win = int(np.sum((imu_t >= t_init) & (imu_t <= t_end)))
    frac = float(win.sum() / n_imu_win) if n_imu_win else 0.0
    c.add('outputs finite, count >= 50 % of IMU messages after initialization', finite and frac >= LIMITS['output_fraction'],
          f'{int(win.sum())} estimates / {n_imu_win} IMU inputs ({frac:.2f}), finite={finite}', 'finite, >= 0.5')
    sz, sv = stats(e_z[win]), stats(e_v[win])
    c.add('altitude error (z vs -h_sensor)', sz['n'] > 0 and sz['rmse'] <= LIMITS['z_rmse'] and sz['peak'] <= LIMITS['z_peak'],
          f"RMSE {sz['rmse']:.4f} m, peak {sz['peak']:.4f} m, mean {sz['mean']:+.4f} m (n={sz['n']})",
          f"RMSE <= {LIMITS['z_rmse']} m, peak <= {LIMITS['z_peak']} m")
    c.add('vertical velocity error (z_dot vs -v_up)', sv['n'] > 0 and sv['rmse'] <= LIMITS['v_rmse'],
          f"RMSE {sv['rmse']:.4f} m/s, peak {sv['peak']:.4f} m/s, mean {sv['mean']:+.4f} m/s (n={sv['n']})",
          f"RMSE <= {LIMITS['v_rmse']} m/s")

    characterization = None
    if t_takeoff is None and asc is not None:
        # Not scored: what the estimate looks like when takeoff is never declared.
        m = (t >= asc['t_start'] + INIT_AFTER_TAKEOFF_S) & (t <= t_end) & inside
        characterization = dict(window_s=[asc['t_start'] + INIT_AFTER_TAKEOFF_S, t_end],
                                z=stats(e_z[m]), v=stats(e_v[m]),
                                z_dot_abs_max=float(np.max(np.abs(est[m, 2]))) if m.any() else math.nan)
        print(f"characterization (no takeoff, not scored): z {characterization['z']}, "
              f"v {characterization['v']}, max |z_dot| {characterization['z_dot_abs_max']:.4f} m/s")
    per_phase = []
    for p in phases:
        m = win & (t >= p['t_start']) & (t < p['t_end'])
        if not m.any():
            continue
        per_phase.append(dict(phase=p['name'], z=stats(e_z[m]), v=stats(e_v[m]),
                              z_in_3sigma=float(np.mean(np.abs(e_z[m]) <= 3 * s_z[m])),
                              v_in_3sigma=float(np.mean(np.abs(e_v[m]) <= 3 * s_v[m]))))
    report = dict(source=source, labels=label, limits=LIMITS, init_interval_s=[None, t_init],
                  scoring_window_s=[t_init, t_end], takeoff_s=t_takeoff, transitions=flying,
                  checks=c.items, ok=c.ok, overall=dict(z=sz, v=sv,
                  z_in_3sigma=float(np.mean(np.abs(e_z[win]) <= 3 * s_z[win])) if win.any() else math.nan,
                  v_in_3sigma=float(np.mean(np.abs(e_v[win]) <= 3 * s_v[win])) if win.any() else math.nan),
                  per_phase=per_phase, estimates=int(len(est)), inputs=n_inputs,
                  no_takeoff_characterization=characterization)
    (out / 'report.json').write_text(json.dumps(report, indent=2, default=float))
    plots(t, est, h, v_up, e_z, e_v, s_z, s_v, phases, t_init, t_end, t_takeoff, label, source, out)
    lines = [f'# REEF vertical estimate vs truth ({source})', '', f'**{label}**', '',
             f'Initialization interval until t = {t_init:.2f} s (takeoff {t_takeoff} + {INIT_AFTER_TAKEOFF_S} s); '
             f'scoring window to {t_end:.2f} s.', '', '| Check | Result | Limit | |', '|---|---|---|---|']
    lines += [f"| {i['name']} | {i['detail']} | {i['criterion']} | {'PASS' if i['ok'] else 'FAIL'} |" for i in c.items]
    lines += ['', '| Phase | z RMSE (m) | z peak (m) | v RMSE (m/s) | z within 3 sigma | v within 3 sigma |',
              '|---|---|---|---|---|---|']
    lines += [f"| {r['phase']} | {r['z']['rmse']:.4f} | {r['z']['peak']:.4f} | {r['v']['rmse']:.4f} | "
              f"{r['z_in_3sigma']:.2f} | {r['v_in_3sigma']:.2f} |" for r in per_phase]
    (out / 'report.md').write_text('\n'.join(lines) + '\n')
    for i in c.items:
        print(f"{'PASS' if i['ok'] else 'FAIL'} {i['name']}: {i['detail']} ({i['criterion']})")
    print(f'report: {out}')
    return 0 if (c.ok or not apply_limits) else 1


def plots(t, est, h, v_up, e_z, e_v, s_z, s_v, phases, t_init, t_end, t_takeoff, label, source, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

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
                     'green: REEF takeoff; orange: commanded motion)', fontsize=8)

    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    ax[0].plot(t, h, 'k', lw=1, label='truth: range-sensor height h')
    ax[0].plot(t, -est[:, 1], 'b', lw=1, label='REEF: -z')
    ax[0].fill_between(t, -est[:, 1] - 3 * s_z, -est[:, 1] + 3 * s_z, color='b', alpha=0.15, label='REEF ±3σ')
    ax[0].set_ylabel('height (m)')
    ax[0].legend(fontsize=8)
    ax[1].plot(t, e_z, 'r', lw=0.8, label='error z + h')
    ax[1].fill_between(t, -3 * s_z, 3 * s_z, color='b', alpha=0.15, label='±3σ')
    ax[1].set_ylim(-0.3, 0.3)
    ax[1].set_ylabel('altitude error (m)')
    ax[1].set_xlabel('sim time (s)')
    ax[1].legend(fontsize=8)
    for a_ in ax:
        shade(a_)
    banner(fig, 'REEF vertical estimate: altitude')
    fig.savefig(out / 'altitude.png', dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    ax[0].plot(t, v_up, 'k', lw=1, label='truth: v_up (odom, ~10 ms lag)')
    ax[0].plot(t, -est[:, 2], 'b', lw=1, label='REEF: -z_dot')
    ax[0].fill_between(t, -est[:, 2] - 3 * s_v, -est[:, 2] + 3 * s_v, color='b', alpha=0.15, label='REEF ±3σ')
    ax[0].set_ylabel('vertical velocity up (m/s)')
    ax[0].legend(fontsize=8)
    ax[1].plot(t, e_v, 'r', lw=0.8, label='error z_dot + v_up')
    ax[1].fill_between(t, -3 * s_v, 3 * s_v, color='b', alpha=0.15, label='±3σ')
    ax[1].set_ylim(-0.6, 0.6)
    ax[1].set_ylabel('velocity error (m/s)')
    ax[1].set_xlabel('sim time (s)')
    ax[1].legend(fontsize=8)
    for a_ in ax:
        shade(a_)
    banner(fig, 'REEF vertical estimate: vertical velocity')
    fig.savefig(out / 'vertical_velocity.png', dpi=110)
    plt.close(fig)

    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for k, name in ((4, 'σ_z (m)'), (5, 'σ_zdot (m/s)'), (6, 'σ_bias (m/s²)')):
        ax[0].semilogy(t, np.sqrt(np.maximum(est[:, k], 1e-30)), lw=1, label=name)
    ax[0].set_ylabel('sqrt(diag P)')
    ax[0].legend(fontsize=8)
    ax[1].plot(t, est[:, 3], 'm', lw=1, label='REEF accel bias b (a = u - b)')
    ax[1].set_ylabel('bias (m/s²)')
    ax[1].set_xlabel('sim time (s)')
    ax[1].legend(fontsize=8)
    for a_ in ax:
        shade(a_)
    banner(fig, 'REEF vertical filter: covariance and bias')
    fig.savefig(out / 'covariance.png', dpi=110)
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
