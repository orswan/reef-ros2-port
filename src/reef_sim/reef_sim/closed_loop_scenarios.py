"""Evaluation of the P07b closed-loop scenarios (faults and position mode).

Called by analyze_closed_loop for runs whose parameters name a scenario
(simulation.scenario). Criteria: docs/ACCEPTANCE.md (control/faults, P07b).
"Judged" items are added as checks; "characterization" items assert the
documented legacy behaviour (also checks, labelled CHARACTERIZATION) and
report numbers in the metrics. A crash is the expected, documented result
where the legacy system has no protection (USER, 2026-10-01).
"""
import math
from pathlib import Path

import numpy as np

from reef_sim.analyze import stamp

TILT = 0.35
BAND = 0.15
FAULTS = ('dropout_short', 'dropout_long', 'estimator_reset', 'controller_restart', 'setpoint_stale',
          'range_loss', 'velocity_loss', 'pause_resume', 'standin_exit')
POSITION = ('position_square', 'position_face_target')


def window(t, t0, t1):
    return (t >= t0) & (t < t1)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Ctx:
    """Arrays of one run, shared by the scenario evaluations."""

    def __init__(self, tr, data, result, phases):
        self.tr, self.data, self.result, self.phases = tr, data, result, phases
        est = data.get('/x3/reef/xyz_estimate', [])
        self.t_est = np.array([stamp(m.header) for _, m in est])
        self.est_z = np.array([m.z_plus.z for _, m in est])
        self.est_ok = all(math.isfinite(v) for _, m in est
                          for v in (m.z_plus.z, m.z_plus.z_dot, m.xy_plus.x_dot, m.xy_plus.y_dot))
        cmd = data.get('/x3/reef/command', [])
        self.t_cmd = np.array([stamp(m.header) for _, m in cmd])
        self.cmd_ok = all(math.isfinite(v) for _, m in cmd for v in m.u[:4])
        self.dbg = np.array([list(m.data) for _, m in data.get('/x3/fc/debug', [])]).reshape(-1, 24)
        self.vz = np.gradient(tr['pos'][:, 2], tr['t'])
        self.speed = np.linalg.norm(np.gradient(tr['pos'], tr['t'], axis=0), axis=1)
        self.tilt = np.maximum(np.abs(tr['roll']), np.abs(tr['pitch']))

    def ph(self, name):
        return self.phases[name]

    def span(self, a, b):
        return window(self.tr['t'], self.ph(a)['t_start'], self.ph(b)['t_end'])

    def max_tilt(self, a, b):
        m = self.span(a, b)
        return float(np.max(self.tilt[m])) if m.any() else float('nan')

    def timeouts_after_arm(self):
        d = self.dbg[self.dbg[:, 0] >= self.ph('takeoff_hover')['t_start']]
        return int(d[-1, 23] - d[0, 23]) if len(d) else -1


def add_tilt(chk, c, a, b, crit):
    t = c.max_tilt(a, b)
    chk.add(f'tilt <= 0.35 rad ({a} .. {b})', t <= TILT, f'max tilt {t:.3f} rad', crit)
    return t


def add_band(chk, c, a, b, target, crit):
    m = c.span(a, b)
    dev = float(np.max(np.abs(c.tr['h'][m] - target))) if m.any() else float('nan')
    chk.add(f'|h - {target}| <= 0.15 m ({a} .. {b})', dev <= BAND, f'max deviation {dev:.3f} m', crit)
    return dev


def end_state(chk, metrics, c, crit):
    ph = c.ph('disarmed')
    me = window(c.tr['t'], ph['t_end'] - 1.0, ph['t_end'])
    de = (c.dbg[:, 0] >= ph['t_end'] - 1.0) & (c.dbg[:, 0] < ph['t_end'])
    h = float(np.max(c.tr['h'][me])) if me.any() else float('nan')
    v = float(np.max(c.speed[me])) if me.any() else float('nan')
    tl = float(np.max(c.tilt[me])) if me.any() else float('nan')
    armed = bool(np.any(c.dbg[de, 1] > 0)) if de.any() else True
    motor = float(np.max(np.abs(c.dbg[de, 17:21]))) if de.any() else float('nan')
    metrics['end_state'] = dict(max_height=h, max_speed=v, max_tilt=tl, armed=armed, max_motor_speed=motor)
    chk.add('end state: disarmed, motors 0, on the ground', de.any() and not armed and motor == 0.0 and h <= 0.05
            and v <= 0.05 and tl <= 0.1, f'armed {armed}, motors {motor}, height {h:.3f} m, speed {v:.3f} m/s, '
            f'tilt {tl:.3f} rad', crit)


def fall(c, t0):
    """Time after t0 until the vehicle falls (vz <= -1 m/s and h < 0.1 m), and the impact speed."""
    t = c.tr['t']
    m = t >= t0
    fast = np.nonzero(m & (c.vz <= -1.0))[0]
    low = np.nonzero(m & (c.tr['h'] < 0.1))[0]
    if not len(fast) or not len(low):
        return None, None
    i = int(low[0])
    impact = float(np.min(c.vz[max(i - 20, 0):i + 1]))
    return float(max(t[fast[0]], t[i]) - t0), impact


def est_gap(c, t0, t1):
    """Longest interval without an estimate in [t0 - 0.2, t1], counting the
    interval from the last estimate to t1 (estimates that never resume)."""
    s = c.t_est[(c.t_est >= t0 - 0.2) & (c.t_est <= t1)]
    if not len(s):
        return float(t1 - t0)
    return float(max(np.max(np.diff(s)) if len(s) > 1 else 0.0, t1 - s[-1]))


def recovery(c, t0, t_end, target):
    """First time after t0 from which |h - target| <= 0.15 m holds until t_end."""
    t, h = c.tr['t'], c.tr['h']
    m = window(t, t0, t_end)
    idx = np.nonzero(m)[0]
    bad = idx[np.abs(h[idx] - target) > BAND]
    if not len(bad):
        return 0.0
    last = bad[-1]
    return float(t[last] - t0) if last + 1 < len(t) and t[last] < t_end - 0.01 else None


def evaluate(scenario, chk, metrics, tr, data, result, phases, run):
    c = Ctx(tr, data, result, phases)
    crit = f'ACCEPTANCE P07b {scenario}'
    char = 'CHARACTERIZATION: '
    # Items whose criterion is 'REPORTED' are measurements that cannot fail: they are printed as
    # REPORTED and never counted as PASS (P09 test accounting).
    faults = result.get('faults') or []
    m = metrics.setdefault('scenario', {'name': scenario})
    if scenario in FAULTS and scenario != 'setpoint_stale':   # the stale case injects no fault
        fault_ev = faults[0] if faults else {}
        m['fault_event'] = fault_ev
        t_f = float(fault_ev.get('t_sim', c.ph('fault')['t_start'] if 'fault' in phases else 0.0))
        chk.add('fault injected', bool(fault_ev.get('ok')), f"{fault_ev.get('fault')}: {fault_ev.get('detail')}", crit)
    if scenario == 'dropout_short':
        gap = est_gap(c, t_f, c.ph('recover')['t_end'])
        m['estimate_gap'] = gap
        chk.add('estimate gap 40-100 ms observed', 0.04 <= gap <= 0.10, f'max gap {1000 * gap:.1f} ms', crit)
        to = c.timeouts_after_arm()
        chk.add('no offboard timeout', to == 0, f'{to} timeouts', crit)
        m['max_tilt'] = add_tilt(chk, c, 'fault', 'recover', crit)
        m['max_dev'] = add_band(chk, c, 'fault', 'recover', 1.0, crit)
        end_state(chk, metrics, c, crit)
    elif scenario == 'dropout_long':
        gap = est_gap(c, t_f, result['phases'][-1]['t_end'])
        m['estimate_gap'] = gap
        chk.add('estimate gap >= 10 s', gap >= 10.0, f'max gap {gap:.2f} s', crit)
        d = c.dbg
        inc = np.nonzero((d[:, 0] >= t_f) & (np.diff(d[:, 23], prepend=d[0, 23]) > 0))[0]
        t_to = float(d[inc[0], 0]) if len(inc) else None
        t_last = float(np.max(c.t_cmd[c.t_cmd <= t_to])) if t_to is not None and (c.t_cmd <= t_to).any() else None
        lag = None if t_to is None or t_last is None else t_to - t_last
        after = d[d[:, 0] > t_to] if t_to is not None else d[:0]
        f0 = bool(len(after) and np.all(after[:, 3] == 0.0))
        m.update(timeout_lag=lag, throttle_zero_after=f0)
        chk.add('offboard timeout within 150 ms of the last command; selected throttle 0 afterwards',
                lag is not None and lag <= 0.150 and f0, f'timeout {lag if lag is None else round(1000 * lag, 1)} ms '
                f'after the last command; throttle 0 afterwards {f0}', crit)
        dt_fall, impact = fall(c, t_to if t_to is not None else t_f)
        m.update(fall_s=dt_fall, impact_vz=impact)
        chk.add(char + 'the vehicle falls (crash, no failsafe in the legacy system)',
                dt_fall is not None and dt_fall <= 2.5, f'fall {dt_fall} s after the timeout, impact {impact} m/s', crit)
    elif scenario == 'estimator_reset':
        chk.add('outputs finite; run completed', c.est_ok and c.cmd_ok and result['status'] == 'completed',
                f'finite {c.est_ok and c.cmd_ok}, {result["status"]}', crit)
        fly = [(tb, mm.data) for tb, mm in data.get('/x3/reef/is_flying_reef', []) if tb >= t_f - 0.05]
        t_land = next((tb for tb, v in fly if not v), None)
        t_up = next((tb for tb, v in fly if v and t_land is not None and tb > t_land), None)
        w = (c.t_est >= t_f - 0.05) & (c.t_est < t_f + 2.0)
        te, ez = c.t_est[w], -c.est_z[w]
        k = int(np.argmax(np.diff(te))) if len(te) > 1 else 0
        gap = float(np.diff(te)[k]) if len(te) > 1 else float('nan')
        first = float(ez[k + 1]) if len(te) > 1 else float('nan')
        h_meas = float(np.interp(te[k + 1], tr['t'], tr['h'])) if len(te) > 1 else float('nan')
        between = 0.25 <= first <= h_meas + 0.02
        span = c.span('fault', 'recover')
        hmin = float(np.min(tr['h'][span]))
        rec = recovery(c, t_f, c.ph('recover')['t_end'], 1.0)
        m.update(landed_at=None if t_land is None else t_land - t_f, takeoff_again_at=None if t_up is None else t_up - t_f,
                 estimate_gap=gap, first_estimate=first, measured_height=h_meas, min_height=hmin,
                 max_tilt=c.max_tilt('fault', 'recover'), recovery_s=rec, crashed=hmin < 0.1)
        chk.add(char + 'reset: landed, estimate gap <= 0.1 s, takeoff again, first estimate between z_x0 and the '
                'measured height', t_land is not None and t_up is not None and gap <= 0.1 and between,
                f"landed at +{m['landed_at']}, flying again at +{m['takeoff_again_at']} s; estimate gap {gap:.3f} s; "
                f'first estimate {first:.3f} m (z_x0 0.25, measured {h_meas:.3f}); min height {hmin:.3f} m, '
                f'crashed {hmin < 0.1}', crit)
    elif scenario == 'controller_restart':
        tc = c.t_cmd[(c.t_cmd >= t_f - 0.2) & (c.t_cmd <= c.ph('recover')['t_end'])]
        gap = float(np.max(np.diff(tc))) if len(tc) > 1 else float('nan')
        resumed = bool(len(c.t_cmd[c.t_cmd > t_f + gap]))
        log = (Path(run) / 'launch.log').read_text(errors='replace') if (Path(run) / 'launch.log').exists() else ''
        starts = sum(1 for ln in log.splitlines() if 'reef_control_node' in ln and 'process started' in ln)
        chk.add('command gap and a new controller process observed; outputs finite; run completed',
                gap >= 0.3 and resumed and starts >= 2 and c.est_ok and c.cmd_ok and result['status'] == 'completed',
                f'command gap {gap:.3f} s, commands resumed {resumed}, controller process starts {starts}', crit)
        span = c.span('fault', 'recover')
        hmin = float(np.min(tr['h'][span]))
        rec = recovery(c, t_f, c.ph('recover')['t_end'], 1.0)
        m.update(command_gap=gap, process_starts=starts, min_height=hmin, altitude_loss=1.0 - hmin,
                 max_tilt=c.max_tilt('fault', 'recover'), recovery_s=rec, crashed=hmin < 0.1)
        chk.add(char + 'integrators restart from 0: altitude loss and recovery reported', True,
                f'min height {hmin:.3f} m (loss {1.0 - hmin:.3f} m), recovery {rec} s, crashed {hmin < 0.1}', 'REPORTED')
    elif scenario == 'setpoint_stale':
        m['max_tilt'] = add_tilt(chk, c, 'takeoff_hover', 'land', crit)
        end_state(chk, metrics, c, crit)
        ph = c.ph('stale')
        w = window(tr['t'], ph['t_end'] - 3.0, ph['t_end'])
        vx = float(np.mean(tr['vx'][w]))
        m['stale_vx'] = vx
        chk.add(char + 'the last setpoint persists (no freshness check): forward 0.3 +- 0.1 m/s',
                abs(vx - 0.3) <= 0.1, f'mean truth forward velocity {vx:+.3f} m/s in the last 3 s of the stale window',
                crit)
    elif scenario == 'range_loss':
        m['max_tilt'] = add_tilt(chk, c, 'fault', 'recover', crit)
        chk.add('outputs finite; run completed', c.est_ok and c.cmd_ok and result['status'] == 'completed',
                f'{result["status"]}', crit)
        w = window(tr['t'], t_f, t_f + 10.0)
        dev = float(np.max(np.abs(tr['h'][w] - 1.0)))
        zi = np.interp(c.t_est, tr['t'], tr['h'])
        we = (c.t_est >= t_f) & (c.t_est < t_f + 10.0)
        zerr = float(np.max(np.abs(-c.est_z[we] - zi[we])))
        m.update(max_height_deviation=dev, max_estimate_error=zerr)
        chk.add(char + 'altitude from the IMU only during the range loss (reported)', True,
                f'max |h - 1.0| {dev:.3f} m, max |REEF -z - h| {zerr:.3f} m over 10 s', 'REPORTED')
    elif scenario == 'velocity_loss':
        dbgm = data.get('/x3/reef/xyz_debug_estimate', [])
        ts = np.array([stamp(mm.header) for _, mm in dbgm])
        sx = np.array([mm.xy_plus.sigma_plus[0] for _, mm in dbgm])
        w = (ts >= t_f + 0.2) & (ts < t_f + 10.0)
        s = sx[w]
        vt = np.array([stamp(mm.header) for _, mm in data.get('/x3/reef/mocap_velocity/body_level_frame', [])])
        n_obs = int(np.sum((vt > t_f + 0.05) & (vt < t_f + 10.0)))
        grows = bool(len(s) > 10 and np.min(s) >= s[0] - 1e-12 and s[-1] >= 10 * s[0])
        m.update(sigma_x_start=float(s[0]) if len(s) else None, sigma_x_end=float(s[-1]) if len(s) else None,
                 sigma_x_min=float(np.min(s)) if len(s) else None, observations_during_loss=n_obs,
                 steps_decreasing=int(np.sum(np.diff(s) < -1e-12)) if len(s) > 1 else None)
        chk.add('no observation during the loss; sigma never below its start value and >= 10x it at the end',
                n_obs == 0 and grows, f"observations {n_obs}; sigma_x {m['sigma_x_start']} (min {m['sigma_x_min']}) -> "
                f"{m['sigma_x_end']} m/s; {m['steps_decreasing']} of {len(s) - 1} steps dip", crit)
        m['max_tilt'] = add_tilt(chk, c, 'fault', 'recover', crit)
        chk.add('run completed', result['status'] == 'completed', result['status'], crit)
        wt = window(tr['t'], t_f, t_f + 10.0)
        p = tr['pos'][wt, :2]
        drift = float(np.linalg.norm(p[-1] - p[0])) if len(p) else float('nan')
        m['drift'] = drift
        chk.add(char + 'horizontal drift during the loss (reported)', True, f'{drift:.3f} m in 10 s', 'REPORTED')
    elif scenario == 'pause_resume':
        to = c.timeouts_after_arm()
        chk.add('no offboard timeout', to == 0, f'{to} timeouts', crit)
        m['max_tilt'] = add_tilt(chk, c, 'fault', 'recover', crit)
        m['max_dev'] = add_band(chk, c, 'fault', 'recover', 1.0, crit)
        end_state(chk, metrics, c, crit)
    elif scenario == 'standin_exit':
        mot = data.get('/x3/fc/motor_speed', [])
        last = list(mot[-1][1].velocity) if mot else []
        zero = bool(last) and all(v == 0.0 for v in last)
        chk.add('the stand-in\'s last motor command is all zeros', zero, f'last command {last}', crit)
        dt_fall, impact = fall(c, t_f)
        m.update(fall_s=dt_fall, impact_vz=impact)
        chk.add(char + 'the vehicle falls (crash, no failsafe in the legacy system)',
                dt_fall is not None and dt_fall <= 2.5, f'fall {dt_fall} s after the exit, impact {impact} m/s', crit)
    elif scenario in POSITION:
        position(scenario, chk, m, c, crit, char)
        end_state(chk, metrics, c, crit)
    else:
        chk.add('known scenario', False, scenario, crit)


def ned(c):
    p = c.tr['pos']
    return np.stack([p[:, 1], p[:, 0]], axis=1)   # ENU -> NED (north, east)


def position(scenario, chk, m, c, crit, char):
    tr = c.tr
    xy = ned(c)
    if scenario == 'position_square':
        legs = ['leg1', 'leg2', 'leg3', 'leg4']
        prev = np.array([0.0, 0.0])
        m['legs'] = {}
        for leg in legs:
            ph = c.ph(leg)
            wp = np.array([ph['px'], ph['py']])
            w2 = window(tr['t'], ph['t_end'] - 2.0, ph['t_end'])
            err = float(np.max(np.linalg.norm(xy[w2] - wp, axis=1)))
            d = wp - prev
            L = float(np.linalg.norm(d))
            wl = window(tr['t'], ph['t_start'], ph['t_end'])
            s = (xy[wl] - prev) @ (d / L)
            over = float(np.max(s) - L)
            w4 = window(tr['t'], ph['t_end'] - 4.0, ph['t_end'])
            hold = float(np.sqrt(np.mean((tr['h'][w4] + ph['z']) ** 2)))
            m['legs'][leg] = dict(waypoint=wp.tolist(), error=err, overshoot=over, hold_rmse=hold)
            chk.add(f'{leg} to {wp.tolist()}: distance <= 0.15 m (last 2 s), overshoot <= 0.30 m, hold RMSE <= 0.10 m',
                    err <= 0.15 and over <= 0.30 and hold <= 0.10,
                    f'distance {err:.3f} m, overshoot {over:+.3f} m, hold RMSE {hold:.3f} m', crit)
            prev = wp
        m['max_tilt'] = add_tilt(chk, c, 'takeoff_hover', 'land', crit)
        # K9: second heading turn goes the long way.
        ph = c.ph('heading_b')
        t0 = ph['t_start']
        yaw0 = float(np.interp(t0, tr['t'], np.unwrap(tr['yaw'])))
        short = wrap(ph['heading'] - yaw0)
        cs = c.data.get('/x3/reef/controller_state', [])
        rate_cmd = [mm.velocity.yaw for tb, mm in cs if t0 <= tb < t0 + 0.5]
        cmd_mean = float(np.mean(rate_cmd)) if rate_cmd else float('nan')
        wr = window(tr['t'], t0 + 0.3, t0 + 1.0)
        truth_rate = float(np.mean(tr['yaw_rate'][wr]))
        m['k9'] = dict(yaw_at_start=yaw0, heading_setpoint=ph['heading'], short_way=short, yaw_rate_command=cmd_mean,
                       truth_yaw_rate=truth_rate)
        chk.add(char + 'K9: heading 2.9 -> -2.9 rad goes the long way (no wrapping)',
                short > 0 and cmd_mean < 0 and truth_rate < 0,
                f'heading at start {yaw0:+.3f} rad, short way {short:+.3f} rad; yaw-rate command {cmd_mean:+.3f}, '
                f'truth yaw rate {truth_rate:+.3f} rad/s', crit)
    else:
        ph = c.ph('leg')
        t0 = ph['t_start']
        cs = [(tb, mm) for tb, mm in c.data.get('/x3/reef/controller_state', []) if mm.position_valid and tb >= t0]
        first = cs[0] if cs else None
        yaw_first = float(np.interp(first[0], tr['t'], tr['yaw'])) if first else float('nan')
        diff0 = abs(wrap(first[1].pose.yaw - yaw_first)) if first else float('nan')
        xy = ned(c)
        wl = window(tr['t'], t0, ph['t_end'])
        idx = np.nonzero(wl)[0]
        dist = np.hypot(ph['px'] - xy[idx, 0], ph['py'] - xy[idx, 1])
        brg = np.arctan2(ph['py'] - xy[idx, 1], ph['px'] - xy[idx, 0])
        err = np.abs((tr['yaw'][idx] - brg + np.pi) % (2 * np.pi) - np.pi)
        near = np.nonzero(dist < 0.15)[0]
        err_start = float(err[0])
        err_near = float(err[near[0]]) if len(near) else float('nan')
        t_near = float(tr['t'][idx[near[0]]] - t0) if len(near) else None
        inside = dist < 0.10
        swing = float(np.ptp(brg[inside])) if inside.any() else float('nan')
        m['k10'] = dict(first_heading_setpoint=first[1].pose.yaw if first else None, heading_at_first=yaw_first,
                        error_at_leg_start=err_start, error_within_0p15=err_near, time_within_0p15=t_near,
                        bearing_swing_in_dead_zone=swing)
        chk.add(char + 'K10: first position step keeps the current heading (theta = 0); the heading error to the '
                'bearing decreases and is <= 0.2 rad within 0.15 m of the target',
                diff0 <= 0.01 and len(near) > 0 and err_near < err_start and err_near <= 0.2,
                f'first setpoint - current heading {diff0:.4f} rad; error {err_start:.3f} rad at the leg start, '
                f'{err_near:.3f} rad at {t_near} s (within 0.15 m); bearing swing inside the dead zone {swing:.3f} rad '
                '(reported)', crit)
        m['max_tilt'] = add_tilt(chk, c, 'takeoff_hover', 'land', crit)
