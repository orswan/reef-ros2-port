"""Score a REEF-controlled X3 run (P07): stability and tracking vs truth.

    ros2 run reef_sim analyze_closed_loop RUN_DIR [--nominal RUN_DIR] [--out DIR] [--no-limits]

Criteria and limits: docs/ACCEPTANCE.md (`control`, P07). The low-level loop
is the STAND-IN (reef_fc_standin, a development tool), and REEF's inputs are
idealized (truth attitude, idealized range, simulated velocity observations,
IMU vibration assumption): these results are simulation-only and are not
ROSflight, hardware, or flight evidence.

Truth (scoring only): the range-sensor origin height h, roll/pitch/yaw of FRD
in NED, and the velocity in REEF's body-level frame (NED rotated by yaw),
all from /x3/truth/odom (pose; velocities by finite differences of the
exact pose). Setpoints from the phases (scenario_result.json): height
-z_c, body-level velocity, yaw rate.

With --nominal (the causality run): the first-hover truth height of this run
must be lower than the nominal run's by the range bias (0.30 +- 0.10 m).
Writes OUT/results.json, OUT/report.txt and plots. Exit 0 all limits met,
1 otherwise (always 0 with --no-limits), 2 unreadable run.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

from reef_sim.analyze import Checks, read_bag, rot, stamp

LIM = dict(takeoff_s=15.0, tilt=0.35, h_min=0.25, overshoot=0.4, band=0.15, settle_s=15.0,
           hold_rmse=0.10, vel_rmse=0.15, yaw_rate_err=0.10, sat_frac=0.05, age_p99=0.020,
           causality=0.30, causality_tol=0.10)
T = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
B = np.diag([1.0, -1.0, -1.0])
SENSOR_OFFSET = np.array([0.0, 0.0, -0.055])
LABEL = ('REEF-controlled X3 with the STAND-IN low-level loop (development tool, not ROSflight); '
         'idealized inputs (truth attitude, idealized range, simulated velocity observations, '
         'IMU vibration assumption). Simulation only.')
HOVERS = ['takeoff_hover', 'hover_fwd', 'hover_left', 'hover_yaw']
MOVES = ['forward', 'left', 'yaw']
STEPS = [('takeoff_hover', 'takeoff to 1.0 m'), ('climb', '1.0 -> 1.5 m'), ('descend', '1.5 -> 0.6 m')]
# The controller's subscriptions and their only allowed publishers: the REEF
# estimate, the setpoint, the stand-in's armed status; is_flying and
# pose_stamped stay unconnected (CONTROL_CHAIN.md, INTERFACES.md section 4).
# /clock is the simulation time source of every use_sim_time node (bridged
# from Gazebo), not a data input.
CONTROLLER_INPUTS = {'/clock': ['/x3_bridge'],
                     '/x3/reef/xyz_estimate': ['/x3/reef/reef_estimator'],
                     '/x3/reef/desired_state': ['/closed_loop_runner'],
                     '/x3/reef/status': ['/reef_fc_standin'],
                     '/x3/reef/is_flying': [], '/x3/reef/pose_stamped': []}


def truth(data):
    odom = data['/x3/truth/odom']
    t = np.array([stamp(m.header) for _, m in odom])
    pos = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for _, m in odom])
    q = np.array([[m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z,
                   m.pose.pose.orientation.w] for _, m in odom])
    R = rot(q)                                   # ENU <- FLU
    h = pos[:, 2] + R[:, 2, :] @ SENSOR_OFFSET   # sensor origin height
    Rn = T @ R @ B                               # NED <- FRD
    roll = np.arctan2(Rn[:, 2, 1], Rn[:, 2, 2])
    pitch = -np.arcsin(np.clip(Rn[:, 2, 0], -1, 1))
    yaw = np.arctan2(Rn[:, 1, 0], Rn[:, 0, 0])
    v_ned = np.gradient(pos, t, axis=0) @ T.T
    c, s = np.cos(yaw), np.sin(yaw)
    vx = c * v_ned[:, 0] + s * v_ned[:, 1]
    vy = -s * v_ned[:, 0] + c * v_ned[:, 1]
    yaw_rate = np.gradient(np.unwrap(yaw), t)
    return dict(t=t, h=h, roll=roll, pitch=pitch, yaw=yaw, vx=vx, vy=vy, yaw_rate=yaw_rate, pos=pos)


def window(t, t0, t1):
    return (t >= t0) & (t < t1)


def rmse(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float('nan')


def analyze(run, nominal=None):
    result = json.loads((run / 'scenario_result.json').read_text())
    params = yaml.safe_load((run / 'x3_scenario.yaml').read_text())
    _, data = read_bag(run / 'bag')
    tr = truth(data)
    phases = {p['name']: p for p in result['phases']}
    chk = Checks()
    metrics = {}
    t_arm = phases['takeoff_hover']['t_start']
    t_end = result['phases'][-1]['t_end']

    # Architecture and data path.
    g = result.get('graph') or {}
    stock_msgs = len(data.get('/x3/cmd_vel', []))
    ok = (not g.get('stock_command_publishers') and stock_msgs == 0
          and g.get('motor_command_publishers') == ['/reef_fc_standin'])
    chk.add('architecture: no stock controller; the stand-in alone commands the motors', ok,
            f"stock command publishers {g.get('stock_command_publishers')}, messages {stock_msgs}; "
            f"motor command publishers {g.get('motor_command_publishers')}", 'ACCEPTANCE P07 architecture')
    inputs = {tp: pubs for tp, pubs in g.get('controller_inputs', {}).items() if tp != '/parameter_events'}
    ok = inputs == CONTROLLER_INPUTS
    chk.add('data path: the controller reads only the REEF estimate, the setpoint, and the armed status', ok,
            f'controller inputs and their publishers: {inputs}', 'ACCEPTANCE P07 data path')
    labels = [m.data for _, m in data.get('/x3/fc/label', [])]
    chk.add('stand-in labelled a development tool', bool(labels) and 'DEVELOPMENT TOOL' in labels[0],
            labels[0][:80] if labels else 'no label', 'ACCEPTANCE P07 architecture')

    # Takeoff.
    fly = [(tb, m.data) for tb, m in data.get('/x3/reef/is_flying_reef', [])]
    t_reef = next((tb for tb, d in fly if d and tb >= t_arm), None)
    after = (tr['t'] >= t_arm)
    up = np.nonzero(after & (tr['h'] > 0.10))[0]
    t_up = float(tr['t'][up[0]]) if len(up) else None
    ok = t_reef is not None and t_up is not None and max(t_reef, t_up) - t_arm <= LIM['takeoff_s']
    metrics['takeoff'] = dict(reef_takeoff_after_arm=None if t_reef is None else t_reef - t_arm,
                              leaves_ground_after_arm=None if t_up is None else t_up - t_arm)
    chk.add('takeoff within 15 s of arming (REEF and truth)', ok, json.dumps(metrics['takeoff']),
            'ACCEPTANCE P07 takeoff')

    # Stability over the flight window.
    reach = np.nonzero(after & (tr['h'] >= LIM['h_min']))[0]
    t_fly = float(tr['t'][reach[0]]) if len(reach) else t_end
    w = window(tr['t'], t_fly, t_end)
    est = data.get('/x3/reef/xyz_estimate', [])
    cmd = data.get('/x3/reef/command', [])
    dbg = np.array([list(m.data) for _, m in data.get('/x3/fc/debug', [])]).reshape(-1, 24)
    finite = (all(math.isfinite(v) for _, m in est for v in (m.z_plus.z, m.z_plus.z_dot, m.xy_plus.x_dot, m.xy_plus.y_dot))
              and all(math.isfinite(v) for _, m in cmd for v in m.u[:4]) and np.isfinite(dbg[:, 17:21]).all())
    tilt = float(np.max(np.maximum(np.abs(tr['roll'][w]), np.abs(tr['pitch'][w])))) if w.any() else float('nan')
    hmin = float(np.min(tr['h'][w])) if w.any() else float('nan')
    ok = (result['status'] == 'completed' and finite and len(reach) > 0 and tilt <= LIM['tilt'] and hmin >= LIM['h_min'])
    metrics['stability'] = dict(flight_from=t_fly - t_arm, max_tilt=tilt, min_height=hmin, finite=finite,
                                status=result['status'])
    chk.add('stability: finite, tilt <= 0.35 rad, height >= 0.25 m after takeoff, run completed', ok,
            f'max tilt {tilt:.3f} rad, min height {hmin:.3f} m, finite {finite}, {result["status"]}',
            'ACCEPTANCE P07 stability')

    # Altitude steps.
    metrics['steps'] = {}
    for name, label in STEPS:
        ph = phases[name]
        target = -ph['z']
        m = window(tr['t'], ph['t_start'], ph['t_end'])
        h = tr['h'][m]
        prev = -phases[result['phases'][[p['name'] for p in result['phases']].index(name) - 1]['name']]['z'] \
            if name != 'takeoff_hover' else 0.0
        up_step = target > prev
        over = float(np.max(h) - target) if up_step else float(target - np.min(h))
        settle = window(tr['t'], ph['t_start'] + LIM['settle_s'], ph['t_end'])
        in_band = bool(settle.any() and np.all(np.abs(tr['h'][settle] - target) <= LIM['band']))
        first = np.nonzero(m & (np.abs(tr['h'] - target) <= LIM['band']))[0]
        metrics['steps'][name] = dict(target=target, overshoot=over, in_band_from_15s=in_band,
                                      first_in_band_s=float(tr['t'][first[0]] - ph['t_start']) if len(first) else None)
        chk.add(f'altitude step {label}: overshoot <= 0.4 m, in +-0.15 m from 15 s to phase end',
                over <= LIM['overshoot'] and in_band, f'overshoot {over:.3f} m, in band {in_band}, '
                f"first in band after {metrics['steps'][name]['first_in_band_s']} s", 'ACCEPTANCE P07 altitude step')

    # Holds and velocity tracking.
    metrics['hold'] = {}
    metrics['velocity'] = {}
    for name in HOVERS + MOVES:
        ph = phases[name]
        span = 4.0 if name in HOVERS else 3.0
        m = window(tr['t'], ph['t_end'] - span, ph['t_end'])
        ex, ey = tr['vx'][m] - ph['vx'], tr['vy'][m] - ph['vy']
        metrics['velocity'][name] = dict(rmse_x=rmse(ex), rmse_y=rmse(ey), mean_vx=float(np.mean(tr['vx'][m])),
                                         mean_vy=float(np.mean(tr['vy'][m])))
        v = metrics['velocity'][name]
        chk.add(f'velocity {name} (last {span:.0f} s): RMSE <= 0.15 m/s per axis',
                v['rmse_x'] <= LIM['vel_rmse'] and v['rmse_y'] <= LIM['vel_rmse'],
                f"x {v['rmse_x']:.3f}, y {v['rmse_y']:.3f} m/s (mean {v['mean_vx']:+.3f}, {v['mean_vy']:+.3f}; "
                f"command {ph['vx']:+.2f}, {ph['vy']:+.2f})", 'ACCEPTANCE P07 horizontal velocity')
        if name in HOVERS:
            e = tr['h'][m] + ph['z']
            metrics['hold'][name] = dict(rmse=rmse(e), mean_error=float(np.mean(e)))
            chk.add(f'altitude hold {name} (last 4 s): RMSE <= 0.10 m', rmse(e) <= LIM['hold_rmse'],
                    f'RMSE {rmse(e):.3f} m, mean error {np.mean(e):+.3f} m', 'ACCEPTANCE P07 altitude hold')
    ph = phases['yaw']
    m = window(tr['t'], ph['t_end'] - 3.0, ph['t_end'])
    yr = float(np.mean(tr['yaw_rate'][m]))
    metrics['yaw_rate'] = dict(mean=yr, command=ph['yaw_rate'])
    chk.add('yaw rate (last 3 s of yaw): |mean - 0.3| <= 0.1 rad/s', abs(yr - ph['yaw_rate']) <= LIM['yaw_rate_err'],
            f'mean {yr:+.3f} rad/s (command {ph["yaw_rate"]:+.2f})', 'ACCEPTANCE P07 yaw rate')

    # Saturation, latency, offboard timeouts.
    tc = np.array([stamp(m.header) for _, m in cmd])
    F = np.array([m.u[3] for _, m in cmd])
    mc = tc >= t_fly
    f_sat = float(np.mean((F[mc] <= 0.0) | (F[mc] >= 1.0))) if mc.any() else float('nan')
    md = dbg[:, 0] >= t_fly
    m_sat = float(np.mean(dbg[md, 21] > 0)) if md.any() else float('nan')
    metrics['saturation'] = dict(throttle_fraction=f_sat, motor_fraction=m_sat)
    chk.add('saturation after takeoff: throttle at 0/1 <= 5 %, motor clamping <= 5 %',
            f_sat <= LIM['sat_frac'] and m_sat <= LIM['sat_frac'],
            f'throttle {100 * f_sat:.1f} %, motors {100 * m_sat:.1f} %', 'ACCEPTANCE P07 saturation')
    ma = dbg[:, 0] >= t_arm
    ages = dbg[ma, 22]
    ages = ages[np.isfinite(ages)]
    p99 = float(np.percentile(ages, 99)) if len(ages) else float('nan')
    to = dbg[ma, 23]
    new_timeouts = int(to[-1] - to[0]) if len(to) else -1
    metrics['latency'] = dict(age_p50=float(np.percentile(ages, 50)) if len(ages) else None, age_p99=p99,
                              age_max=float(np.max(ages)) if len(ages) else None, offboard_timeouts=new_timeouts)
    chk.add('estimate age at the motor command p99 <= 20 ms; no offboard timeout after arming',
            p99 <= LIM['age_p99'] and new_timeouts == 0,
            f"p50 {metrics['latency']['age_p50']}, p99 {p99:.4f} s, max {metrics['latency']['age_max']}; "
            f'offboard timeouts {new_timeouts}', 'ACCEPTANCE P07 staleness and latency')

    bias = params.get('range_sensor', {}).get('ros__parameters', {}).get('bias', 0.0)
    metrics['range_bias'] = bias
    if nominal is not None:
        ph = phases['takeoff_hover']
        mine = float(np.mean(tr['h'][window(tr['t'], ph['t_end'] - 4, ph['t_end'])]))
        nres = json.loads((nominal / 'scenario_result.json').read_text())
        nph = {p['name']: p for p in nres['phases']}['takeoff_hover']
        ntr = truth(read_bag(nominal / 'bag')[1])
        theirs = float(np.mean(ntr['h'][window(ntr['t'], nph['t_end'] - 4, nph['t_end'])]))
        drop = theirs - mine
        metrics['causality'] = dict(bias=bias, nominal_height=theirs, biased_height=mine, drop=drop)
        chk.add('causality: a +0.30 m range bias lowers the truth hover height by 0.30 +- 0.10 m',
                abs(bias - LIM['causality']) < 1e-9 and abs(drop - LIM['causality']) <= LIM['causality_tol'],
                f'bias {bias} m; first-hover height nominal {theirs:.3f} m, biased {mine:.3f} m, drop {drop:+.3f} m',
                'ACCEPTANCE P07 causality')
    return chk, metrics, tr, data, result, phases


def plots(tr, data, result, phases, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t0 = phases['takeoff_hover']['t_start']
    t = tr['t'] - t0
    est = data.get('/x3/reef/xyz_estimate', [])
    te = np.array([stamp(m.header) for _, m in est]) - t0
    ez = np.array([-m.z_plus.z for _, m in est])
    evx = np.array([m.xy_plus.x_dot for _, m in est])
    evy = np.array([m.xy_plus.y_dot for _, m in est])

    def setpoint(key, sign=1.0):
        xs, ys = [], []
        for p in result['phases']:
            xs += [p['t_start'] - t0, p['t_end'] - t0]
            ys += [sign * p[key]] * 2
        return xs, ys

    fig, ax = plt.subplots(5, 1, figsize=(11, 15), sharex=True)
    ax[0].plot(t, tr['h'], label='truth sensor height')
    ax[0].plot(te, ez, label='REEF -z', alpha=0.7)
    ax[0].plot(*setpoint('z', -1.0), 'k--', label='setpoint')
    ax[0].set_ylabel('height [m]')
    ax[1].plot(t, tr['vx'], label='truth')
    ax[1].plot(te, evx, alpha=0.7, label='REEF')
    ax[1].plot(*setpoint('vx'), 'k--', label='command')
    ax[1].set_ylabel('v forward [m/s]')
    ax[2].plot(t, tr['vy'], label='truth')
    ax[2].plot(te, evy, alpha=0.7, label='REEF')
    ax[2].plot(*setpoint('vy'), 'k--', label='command')
    ax[2].set_ylabel('v right [m/s]')
    ax[3].plot(t, tr['roll'], label='roll (truth)')
    ax[3].plot(t, tr['pitch'], label='pitch (truth)')
    ax[3].plot(t, tr['yaw_rate'], label='yaw rate (truth)', alpha=0.6)
    ax[3].plot(*setpoint('yaw_rate'), 'k--', label='yaw rate command')
    ax[3].set_ylabel('rad, rad/s')
    cmd = data.get('/x3/reef/command', [])
    tc = np.array([stamp(m.header) for _, m in cmd]) - t0
    ax[4].plot(tc, [m.u[3] for _, m in cmd], label='throttle F (reef_control)')
    ax[4].plot(tc, [m.u[0] for _, m in cmd], label='roll cmd', alpha=0.7)
    ax[4].plot(tc, [m.u[1] for _, m in cmd], label='pitch cmd', alpha=0.7)
    ax[4].set_ylabel('command')
    ax[4].set_xlabel('time since arming [s] (sim)')
    for a in ax:
        for p in result['phases']:
            a.axvline(p['t_start'] - t0, color='0.85', lw=0.8)
        a.legend(loc='upper right', fontsize=7)
        a.grid(alpha=0.3)
    fig.suptitle(LABEL, fontsize=8)
    fig.tight_layout()
    fig.savefig(out / 'closed_loop.png', dpi=110)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--nominal')
    ap.add_argument('--out')
    ap.add_argument('--no-limits', action='store_true')
    a = ap.parse_args(argv)
    run = Path(a.run)
    out = Path(a.out) if a.out else run / 'analysis_closed_loop'
    out.mkdir(parents=True, exist_ok=True)
    try:
        chk, metrics, tr, data, result, phases = analyze(run, Path(a.nominal) if a.nominal else None)
    except (OSError, KeyError, ValueError, IndexError) as e:
        print(f'FAIL cannot analyze {run}: {e!r}')
        return 2
    lines = [LABEL, '']
    for i in chk.items:
        lines.append(f"{'PASS' if i['ok'] else 'FAIL'} {i['name']}: {i['detail']}  [{i['criterion']}]")
    lines.append('ANALYSIS PASSED' if chk.ok else 'ANALYSIS FAILED')
    print('\n'.join(lines))
    (out / 'report.txt').write_text('\n'.join(lines) + '\n')
    (out / 'results.json').write_text(json.dumps(dict(label=LABEL, ok=chk.ok, checks=chk.items, metrics=metrics),
                                                 indent=1, default=float))
    try:
        plots(tr, data, result, phases, out)
    except Exception as e:   # plots are evidence, not a check
        print(f'WARN plots failed: {e!r}')
    return 0 if (chk.ok or a.no_limits) else 1


if __name__ == '__main__':
    sys.exit(main())
