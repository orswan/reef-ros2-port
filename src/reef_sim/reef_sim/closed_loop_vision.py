"""Evaluation of the closed loop on vision (P08, scenario `vision`).

Called by analyze_closed_loop for runs of run_x3_scenario.sh --closed-loop
--vision (config/closed_loop/vision.yaml). Criteria: docs/ACCEPTANCE.md
(vision, closed loop) with the definitions of docs/VISION.md section 7:
P07 limits over the judged window (arming to the start of weak_left); the
weak-texture segment and everything after it are a characterization
(REPORTED, not judged; the legacy system has no failsafe, USER).
"""
import json
import math

import numpy as np

from reef_sim.analyze import stamp
from reef_sim.closed_loop_scenarios import Ctx, window

TILT, H_MIN, VEL_RMSE, SAT, AGE_P99 = 0.35, 0.25, 0.15, 0.05, 0.020
HOVERS = ['takeoff_hover', 'hover_fwd', 'hover_back', 'hover_right', 'hover_left']
MOVES = ['forward', 'back', 'right', 'left']
VELOCITY_TOPIC = '/x3/reef/rgbd_to_velocity/body_level_frame'
ESTIMATOR_INPUTS = {'/clock': ['/x3_bridge'], '/x3/range': ['/range_sensor'],
                    '/x3/reef/imu/data': ['/x3_imu_adapter'], VELOCITY_TOPIC: ['/x3/reef/rgbd_to_velocity_node']}
LABEL = ('REEF-controlled X3 on VISION: REEF controller + STAND-IN low-level loop (development tool, not '
         'ROSflight); REEF horizontal velocity from the REPLACEMENT RGB-D odometry (not demo_rgbd) through '
         'rgbd_to_velocity; attitude = truth (idealized), range = idealized, IMU vibration assumption. '
         'Simulated camera and scene: simulation only.')
CRIT = 'ACCEPTANCE vision closed loop'


def rmse(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float('nan')


def report(chk, name, detail):
    chk.add(name, True, detail, 'REPORTED')


def health(data):
    out = []
    for _, m in data.get('/x3/reef/vo/health', []):
        kv = {x.key: x.value for x in m.status[0].values}
        out.append((stamp(m.header), kv['state'] == 'LOST', kv['published'] == '1'))
    return np.array(out, float).reshape(-1, 3)


def evaluate_vision(chk, metrics, tr, data, result, phases, run):
    c = Ctx(tr, data, result, phases)
    m = metrics.setdefault('vision', {})
    metrics['label'] = LABEL
    t_arm = phases['takeoff_hover']['t_start']
    t_w = phases['weak_left']['t_start']
    t_end = result['phases'][-1]['t_end']

    # Estimator data path (ROS graph, camera_check.json): vision is the only horizontal velocity input.
    cc = run / 'camera_check.json'
    path = json.loads(cc.read_text()).get('estimator_inputs', {}) if cc.exists() else {}
    fed = {t: p for t, p in path.items() if p and t != '/parameter_events'} if 'error' not in path else {}
    ok = fed == ESTIMATOR_INPUTS
    chk.add("estimator data path (ROS graph): horizontal velocity only from rgbd_to_velocity; no truth-derived "
            "velocity", ok, f'estimator inputs and their publishers: {fed}', CRIT)

    # Stability over the judged window (P07 limits).
    after = tr['t'] >= t_arm
    reach = np.nonzero(after & (tr['h'] >= H_MIN))[0]
    t_fly = float(tr['t'][reach[0]]) if len(reach) else t_w
    w = window(tr['t'], t_fly, t_w)
    est = data.get('/x3/reef/xyz_estimate', [])
    te = np.array([stamp(mm.header) for _, mm in est])
    ew = (te >= t_arm) & (te < t_w)
    finite = bool(all(math.isfinite(v) for (_, mm), k in zip(est, ew) if k
                      for v in (mm.z_plus.z, mm.z_plus.z_dot, mm.xy_plus.x_dot, mm.xy_plus.y_dot)))
    cw = (c.t_cmd >= t_arm) & (c.t_cmd < t_w)
    cmd = data.get('/x3/reef/command', [])
    finite = finite and all(math.isfinite(v) for (_, mm), k in zip(cmd, cw) if k for v in mm.u[:4])
    tilt = float(np.max(c.tilt[w])) if w.any() else float('nan')
    hmin = float(np.min(tr['h'][w])) if w.any() else float('nan')
    ok = result['status'] == 'completed' and finite and len(reach) > 0 and tilt <= TILT and hmin >= H_MIN
    m['stability'] = dict(flight_from=t_fly - t_arm, max_tilt=tilt, min_height=hmin, finite=finite,
                          status=result['status'])
    chk.add('stability (judged window: height reached .. start of weak_left): finite, tilt <= 0.35 rad, height '
            '>= 0.25 m; run completed', ok,
            f'max tilt {tilt:.3f} rad, min height {hmin:.3f} m, finite {finite}, {result["status"]}', CRIT)

    # Horizontal velocity tracking (truth body-level vs command).
    m['velocity'] = {}
    for name in HOVERS + MOVES:
        ph = phases[name]
        span = 4.0 if name in HOVERS else 3.0
        mm = window(tr['t'], ph['t_end'] - span, ph['t_end'])
        ex, ey = tr['vx'][mm] - ph['vx'], tr['vy'][mm] - ph['vy']
        v = dict(rmse_x=rmse(ex), rmse_y=rmse(ey), mean_vx=float(np.mean(tr['vx'][mm])),
                 mean_vy=float(np.mean(tr['vy'][mm])))
        m['velocity'][name] = v
        chk.add(f'velocity {name} (last {span:.0f} s): RMSE <= 0.15 m/s per axis',
                v['rmse_x'] <= VEL_RMSE and v['rmse_y'] <= VEL_RMSE,
                f"x {v['rmse_x']:.3f}, y {v['rmse_y']:.3f} m/s (mean {v['mean_vx']:+.3f}, {v['mean_vy']:+.3f}; "
                f"command {ph['vx']:+.2f}, {ph['vy']:+.2f})", CRIT)

    # Saturation and staleness over the judged window (P07 limits).
    F = np.array([mm.u[3] for _, mm in cmd])
    mc = (c.t_cmd >= t_fly) & (c.t_cmd < t_w)
    f_sat = float(np.mean((F[mc] <= 0.0) | (F[mc] >= 1.0))) if mc.any() else float('nan')
    dbg = c.dbg
    md = (dbg[:, 0] >= t_fly) & (dbg[:, 0] < t_w)
    m_sat = float(np.mean(dbg[md, 21] > 0)) if md.any() else float('nan')
    chk.add('saturation (judged window): throttle at 0/1 <= 5 %, motor clamping <= 5 %',
            f_sat <= SAT and m_sat <= SAT, f'throttle {100 * f_sat:.1f} %, motors {100 * m_sat:.1f} %', CRIT)
    ma = (dbg[:, 0] >= t_arm) & (dbg[:, 0] < t_w)
    ages = dbg[ma, 22]
    ages = ages[np.isfinite(ages)]
    p99 = float(np.percentile(ages, 99)) if len(ages) else float('nan')
    to = dbg[ma, 23]
    timeouts = int(to[-1] - to[0]) if len(to) else -1
    m['latency'] = dict(age_p50=float(np.percentile(ages, 50)) if len(ages) else None, age_p99=p99,
                        offboard_timeouts=timeouts)
    chk.add('staleness (judged window): estimate age at the motor command p99 <= 20 ms; no offboard timeout',
            p99 <= AGE_P99 and timeouts == 0, f"p50 {m['latency']['age_p50']}, p99 {p99:.4f} s; offboard "
            f'timeouts {timeouts}', CRIT)

    # Vision chain during the judged window (reported).
    H = health(data)
    hj = (H[:, 0] >= t_arm) & (H[:, 0] < t_w) if len(H) else np.zeros(0, bool)
    report(chk, 'odometry health in the judged window',
           f'{int(hj.sum())} frames, LOST {int(H[hj, 1].sum()) if len(H) else 0}, published '
           f'{int(H[hj, 2].sum()) if len(H) else 0}')

    # Characterization: the weak-texture segment and everything after it.
    lost = H[(H[:, 0] >= t_w) & (H[:, 1] > 0)] if len(H) else H
    t_lost = float(lost[0, 0]) if len(lost) else None
    pub_after = H[(H[:, 0] > (t_lost or math.inf)) & (H[:, 2] > 0)] if len(H) else H
    t_resume = float(pub_after[0, 0]) if len(pub_after) else None
    sig = [(stamp(mm.header), (mm.xy_plus.sigma_plus[0] - mm.xy_plus.x_dot) / 3.0,
            (mm.xy_plus.sigma_plus[1] - mm.xy_plus.y_dot) / 3.0)
           for _, mm in data.get('/x3/reef/xyz_debug_estimate', [])]
    sig = np.array(sig, float).reshape(-1, 3)

    def sigma_at(t):
        if t is None or not len(sig):
            return None
        i = min(np.searchsorted(sig[:, 0], t), len(sig) - 1)
        return [float(sig[i, 1]), float(sig[i, 2])]
    peak = None
    if t_lost is not None and len(sig):
        ms = (sig[:, 0] >= t_lost) & (sig[:, 0] <= (t_resume or t_end))
        peak = [float(sig[ms, 1].max()), float(sig[ms, 2].max())] if ms.any() else None
    m['weak'] = dict(first_lost=t_lost, lost_after_weak_start=None if t_lost is None else t_lost - t_w,
                     resume=t_resume, sigma_at_lost=sigma_at(t_lost), sigma_peak=peak)
    report(chk, 'characterization: odometry loss in the weak-texture segment',
           f"first LOST {('%.2f s after the start of weak_left' % (t_lost - t_w)) if t_lost else 'never'}; "
           f"resumed {('%.2f s after the loss' % (t_resume - t_lost)) if t_resume else 'never'}; REEF "
           f"sigma(x_dot, y_dot) at the loss {sigma_at(t_lost)}, peak {peak}")
    seg = {}
    for name in ('weak_left', 'hover_weak', 'weak_return', 'hover_ret', 'approach', 'land'):
        ph = phases[name]
        mm = window(tr['t'], ph['t_start'], ph['t_end'])
        if not mm.any():
            continue
        # drift: truth horizontal displacement minus the commanded body-level displacement, both in NED
        # (tr['yaw'] is the NED yaw of FRD; no yaw command in this profile)
        pos = tr['pos'][mm]
        yaw0 = tr['yaw'][mm][0]
        d = pos[-1] - pos[0]
        d_ned = np.array([d[1], d[0]])                                       # ENU -> NED (north, east)
        cmd_ned = np.array([[math.cos(yaw0), -math.sin(yaw0)], [math.sin(yaw0), math.cos(yaw0)]]) @ \
            (np.array([ph['vx'], ph['vy']]) * ph['duration'])
        seg[name] = dict(rmse_x=rmse(tr['vx'][mm] - ph['vx']), rmse_y=rmse(tr['vy'][mm] - ph['vy']),
                         max_tilt=float(np.max(c.tilt[mm])), min_height=float(np.min(tr['h'][mm])),
                         drift_m=float(np.linalg.norm(d_ned - cmd_ned)))
    m['weak_segments'] = seg
    for name, v in seg.items():
        report(chk, f'characterization: {name}',
               f"truth velocity vs command RMSE x {v['rmse_x']:.3f}, y {v['rmse_y']:.3f} m/s; drift from the "
               f"commanded path {v['drift_m']:.2f} m; max tilt {v['max_tilt']:.3f} rad; min height "
               f"{v['min_height']:.3f} m")
    ph = phases['disarmed']
    me = window(tr['t'], ph['t_end'] - 1.0, ph['t_end'])
    de = (dbg[:, 0] >= ph['t_end'] - 1.0) & (dbg[:, 0] < ph['t_end'])
    endv = dict(height=float(np.max(tr['h'][me])) if me.any() else None,
                speed=float(np.max(c.speed[me])) if me.any() else None,
                tilt=float(np.max(c.tilt[me])) if me.any() else None,
                armed=bool(np.any(dbg[de, 1] > 0)) if de.any() else None,
                max_motor=float(np.max(np.abs(dbg[de, 17:21]))) if de.any() else None)
    home = tr['pos'][np.argmin(np.abs(tr['t'] - t_arm)), :2]
    endv['distance_from_takeoff_m'] = float(np.linalg.norm(tr['pos'][-1, :2] - home))
    endv['position_end_enu'] = tr['pos'][-1, :3].tolist()
    m['end_state'] = endv
    report(chk, 'characterization: end state (P07 end-state values, not judged after the weak segment)',
           ', '.join(f'{k} {v:.3f}' if isinstance(v, float) else f'{k} {v}' for k, v in endv.items()
                     if k != 'position_end_enu') + f", final position ENU {np.round(tr['pos'][-1, :3], 2).tolist()}")
