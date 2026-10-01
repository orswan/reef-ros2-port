"""Score a vision run (P08, ACCEPTANCE.md `vision`) against truth.

    ros2 run reef_sim analyze_vision RUN_DIR [--out DIR]

Reads RUN_DIR/bag, scenario_result.json, x3_scenario.yaml, camera_check.json
(run_x3_scenario.sh --vision). The chain is camera -> reef_rgbd_odometry
(REPLACEMENT odometry, not demo_rgbd) -> rgbd_to_velocity (ported) -> REEF,
open loop: the stock controller flies on truth. REEF's attitude input is the
truth attitude (idealized); its only horizontal velocity input is vision.

Truth (scoring only): the body-level velocity (x forward, y right, yaw-aligned)
from /x3/truth/odom, interpolated to each message stamp (analyze_reef.truth_at).

Scoring definitions (docs/VISION.md section 7; fixed before the first scored run):
  steady segments  the phases with the camera on the rich texture and the wall
                   within the clip range: hover_start, forward, hover_fwd,
                   back, hover_back, right, hover_right, left, hover_left,
                   hover_ret, hover_end, hover_low, each without its first
                   1.0 s (controller transient); REEF samples additionally
                   only from 2 s after REEF declares takeoff (as P05)
  sign/frame       forward -> mean vision x > +0.1, back -> < -0.1, right
                   (FLU vy < 0) -> mean vision y > +0.1, left -> < -0.1; the
                   other axis |mean| below half the primary one
  rate             vision velocity messages per sim second in the steady segments
  weak texture     event t_w: first time in weak_left at which the camera's
                   right image edge ray meets the wall beyond the rich region
                   (y > 1.5: no rich texture in view); end t_we: first time in
                   weak_return at which the optical axis meets the rich region
  depth loss       event t_d: first time in far_back at which the planar depth
                   of the wall at the image centre exceeds the far clip (8 m);
                   end t_de: first time in far_return at which it is back below
  loss detection   a health message (stamp = image stamp) with state LOST in
                   [segment phase start, event + 0.5 s]
  no publication   no cam_to_init and no rgbd_to_velocity message with a stamp
                   strictly inside any LOST interval (first LOST stamp to the
                   next published stamp), anywhere in the run
  variance growth  REEF's sigma(x_dot) and sigma(y_dot) at the last estimate
                   before the resume are larger than at the loss
  resume           the last LOST -> published transition between the segment
                   phase start and the end of the following hover phase is
                   at most 1.0 s after the segment end
  REEF recovery    REEF velocity RMSE per axis over [end + 3 s, end + 4 s] <= 0.10
  faults run       (fault_schedule set: labelled hooks) per fault window
                   [phase start + offset, + span] extended by 2 s: every
                   vision velocity finite, per-axis |error| <= 0.3 m/s; latency
                   (bag receive time - stamp, both sim) and rate REPORTED;
                   the nominal items are then REPORTED only (not judged)
REPORTED items (noise vs covariance, latency, performance, gate counts) are
never counted as PASS. Writes OUT/analysis_vision.json and plots. Exit 0 if
every judged item passes, 1 otherwise.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

from reef_sim.analyze import Checks, arrays, read_bag, rot
from reef_sim.analyze_reef import truth_at

STEADY = ['hover_start', 'forward', 'hover_fwd', 'back', 'hover_back', 'right', 'hover_right', 'left',
          'hover_left', 'hover_ret', 'hover_end', 'hover_low']
TRIM_S = 1.0
INIT_AFTER_TAKEOFF_S = 2.0
LIM = dict(vo_rmse=0.10, vo_bias=0.03, rate_hz=10.0, reef_rmse=0.10, detect_s=0.5, resume_s=1.0,
           recover_after_s=3.0, recover_window_s=1.0, fault_err=0.3, fault_tail_s=2.0, sign_min=0.1)
WALL_X, RICH_Y_MAX, FAR_CLIP, HALF_FOV = 4.0, 1.5, 8.0, 1.0471975511965976 / 2
VELOCITY_TOPIC = '/x3/reef/rgbd_to_velocity/body_level_frame'
LABEL = ('REPLACEMENT RGB-D odometry (OpenCV, not demo_rgbd) -> rgbd_to_velocity (ported) -> REEF, OPEN LOOP: '
         'the stock controller flies on truth. REEF attitude input = truth attitude (idealized); range = '
         'idealized; horizontal velocity = vision only. Simulated camera, procedural scene: not hardware evidence.')


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def window(phases, name, trim=0.0):
    for p in phases:
        if p['name'] == name:
            return p['t_start'] + trim, p['t_end']
    return None


def following_hover(phases, name):
    names = [p['name'] for p in phases]
    for p in phases[names.index(name) + 1:]:
        if p['name'].startswith('hover'):
            return p
    return None


def mask_of(t, wins):
    m = np.zeros(len(t), bool)
    for w in wins:
        m |= (t >= w[0]) & (t < w[1])
    return m


def rms(x):
    return float(np.sqrt(np.mean(np.square(x)))) if len(x) else math.nan


# ------------------------------------------------------------------ inputs
def vision_msgs(data):
    # legacy rgbd_to_velocity stamps only vel.header (the outer header stays 0; kept by the port)
    rows = [(st(m.vel.header), tb, m.vel.twist.twist.linear.x, m.vel.twist.twist.linear.y,
             m.vel.twist.covariance[0], m.vel.twist.covariance[7]) for tb, m in data.get(VELOCITY_TOPIC, [])]
    return np.array(rows, float).reshape(-1, 6)


def health_msgs(data):
    out = []
    for tb, m in data.get('/x3/reef/vo/health', []):
        kv = {x.key: x.value for x in m.status[0].values}
        out.append(dict(t=st(m.header), tb=tb, state=kv['state'], published=kv['published'] == '1',
                        ms=float(kv['processing_ms_wall']), wall=float(kv.get('wall_time_s', 'nan')),
                        tracks=int(kv['tracks']), depth_tracks=int(kv['depth_tracks']), inliers=int(kv['inliers']),
                        dropped=int(kv['frames_dropped_by_hook']), lost_events=int(kv['lost_events'])))
    return out


def reef_msgs(data):
    rows = []
    for tb, m in data.get('/x3/reef/xyz_debug_estimate', []):
        xy = m.xy_plus
        rows.append((st(m.header), xy.x_dot, xy.y_dot, (xy.sigma_plus[0] - xy.x_dot) / 3.0,
                     (xy.sigma_plus[1] - xy.y_dot) / 3.0, m.z_plus.z, m.z_plus.z_dot))
    return np.array(rows, float).reshape(-1, 7)


def camera_geometry(a, params, t):
    """Wall intersections of the optical axis and the image edges, from truth (scoring only)."""
    odom = a['odom']
    R = rot(odom[:, 5:9])
    mount = np.array(params['camera_check']['ros__parameters']['mount_flu'], float)
    cam = odom[:, 2:5] + np.einsum('nij,j->ni', R, mount)
    out = {}
    for name, ang in (('axis', 0.0), ('left', HALF_FOV), ('right', -HALF_FOV)):
        d = np.einsum('nij,j->ni', R, np.array([math.cos(ang), math.sin(ang), 0.0]))
        with np.errstate(divide='ignore', invalid='ignore'):
            s = np.where(d[:, 0] > 1e-6, (WALL_X - cam[:, 0]) / d[:, 0], np.inf)
        out[name + '_y'] = np.interp(t, odom[:, 0], cam[:, 1] + s * d[:, 1])
        if name == 'axis':
            out['depth'] = np.interp(t, odom[:, 0], s)   # planar depth at the image centre
    return out


def first_time(t, cond):
    i = np.flatnonzero(cond)
    return float(t[i[0]]) if len(i) else None


def lost_intervals(health):
    out, start = [], None
    for h in health:
        if h['state'] == 'LOST' and start is None:
            start = h['t']
        if h['published'] and start is not None:
            out.append((start, h['t']))
            start = None
    if start is not None:
        out.append((start, math.inf))
    return out


# ----------------------------------------------------------------- scoring
def analyze(run):
    params = yaml.safe_load((run / 'x3_scenario.yaml').read_text())
    result = json.loads((run / 'scenario_result.json').read_text())
    phases = result['phases']
    types, data = read_bag(run / 'bag')
    a = arrays(data)
    vo_params = params.get('/x3/reef/reef_rgbd_odometry', {}).get('ros__parameters', {})
    faults = vo_params.get('fault_schedule', []) or []
    faults = [f for f in faults if f]
    manifest = yaml.safe_load((run / 'manifest.yaml').read_text()) if (run / 'manifest.yaml').exists() else {}
    faults_requested = str((manifest.get('run') or {}).get('vision_faults', '0')) == '1'
    c, rep = Checks(), {}

    def judged(name, ok, detail, criterion):
        c.add(name if not faults else f'(nominal item, REPORTED in a faults run) {name}', ok or bool(faults),
              detail + ('' if not faults else f' [would be {"PASS" if ok else "FAIL"}]'), criterion)

    if faults_requested != bool(faults):
        c.add('fault schedule matches the run (manifest vision_faults)', False,
              f'manifest vision_faults={int(faults_requested)}, fault_schedule for /x3/reef/reef_rgbd_odometry: '
              f'{faults}', 'a faults run has a schedule, a nominal run none')
    v = vision_msgs(data)
    health = health_msgs(data)
    reef = reef_msgs(data)
    cam_t = np.array([st(m.header) for _, m in data.get('/x3/reef/cam_to_init', [])])
    if len(v) == 0 or not health or len(reef) == 0:
        c.add('vision chain outputs present', False,
              f'velocity {len(v)}, health {len(health)}, REEF {len(reef)} messages', 'all > 0')
        return c, rep, None
    (_, _, vtx, vty), vvalid, _ = truth_at(a, params, v[:, 0])
    (_, _, rtx, rty), rvalid, _ = truth_at(a, params, reef[:, 0])
    ex, ey = v[:, 2] - vtx, v[:, 3] - vty
    rex, rey = reef[:, 1] - rtx, reef[:, 2] - rty
    flying = [(tb, m.data) for tb, m in data.get('/x3/reef/is_flying_reef', [])]
    t_takeoff = next((t for t, f in flying if f), None)
    steady = [w for w in (window(phases, n, TRIM_S) for n in STEADY) if w]
    sv = mask_of(v[:, 0], steady) & vvalid
    sr = mask_of(reef[:, 0], steady) & rvalid
    if t_takeoff is not None:
        sr &= reef[:, 0] >= t_takeoff + INIT_AFTER_TAKEOFF_S

    # ---------------- outputs finite
    fin_v = bool(np.all(np.isfinite(v[:, 2:])))
    fin_r = bool(np.all(np.isfinite(reef[:, 1:])))
    c.add('outputs finite (vision velocity and REEF estimate, whole run)', fin_v and fin_r,
          f'vision {len(v)} msgs finite {fin_v}; REEF {len(reef)} estimates finite {fin_r}', 'all finite')

    # ---------------- replacement odometry open loop
    steady_s = sum(w[1] - w[0] for w in steady)
    rate = sv.sum() / steady_s if steady_s else 0.0
    judged('vision velocity rate in steady segments (sim)', rate >= LIM['rate_hz'],
           f'{sv.sum()} msgs over {steady_s:.1f} s = {rate:.2f} Hz', f">= {LIM['rate_hz']} Hz")
    for ax, e in (('x', ex), ('y', ey)):
        r, b = rms(e[sv]), float(np.mean(e[sv])) if sv.any() else math.nan
        judged(f'vision velocity {ax} vs truth, steady segments', r <= LIM['vo_rmse'] and abs(b) <= LIM['vo_bias'],
               f'RMSE {r:.4f} m/s, bias {b:+.4f} m/s (n {sv.sum()})',
               f"RMSE <= {LIM['vo_rmse']}, |bias| <= {LIM['vo_bias']} m/s")
        rep[f'vo_{ax}'] = dict(rmse=r, bias=b, std=float(np.std(e[sv])), n=int(sv.sum()))
    for leg, axis, sign in (('forward', 0, 1), ('back', 0, -1), ('right', 1, 1), ('left', 1, -1)):
        w = window(phases, leg, TRIM_S)
        m = mask_of(v[:, 0], [w]) if w else np.zeros(len(v), bool)
        mean = v[m, 2:4].mean(axis=0) if m.any() else np.array([math.nan, math.nan])
        tmean = np.array([vtx[m].mean(), vty[m].mean()]) if m.any() else mean
        ok = bool(m.any() and sign * mean[axis] > LIM['sign_min'] and abs(mean[1 - axis]) < 0.5 * abs(mean[axis])
                  and np.sign(tmean[axis]) == sign)
        judged(f'sign/frame: {leg} leg -> {"+" if sign > 0 else "-"}{"xy"[axis]} (body level, FRD)', ok,
               f'vision mean ({mean[0]:+.3f}, {mean[1]:+.3f}), truth mean ({tmean[0]:+.3f}, {tmean[1]:+.3f}) m/s',
               f"primary {'+' if sign > 0 else '-'}{'xy'[axis]} beyond {LIM['sign_min']}, cross axis < half")
    cov = v[sv, 4:6]
    rep['noise_vs_covariance'] = dict(
        measured_std=[rep['vo_x']['std'], rep['vo_y']['std']],
        configured_std=[float(np.sqrt(np.median(cov[:, 0]))) if len(cov) else math.nan,
                        float(np.sqrt(np.median(cov[:, 1]))) if len(cov) else math.nan])
    nv = rep['noise_vs_covariance']
    c.add('noise vs configured covariance (steady segments)', True,
          f"measured error std x {nv['measured_std'][0]:.4f}, y {nv['measured_std'][1]:.4f} m/s; configured "
          f"sqrt(covariance) x {nv['configured_std'][0]:.4f}, y {nv['configured_std'][1]:.4f} m/s", 'REPORTED')
    lat = v[:, 1] - v[:, 0]
    rep['latency_s'] = dict(median=float(np.median(lat)), p99=float(np.percentile(lat, 99)), max=float(lat.max()))
    c.add('vision velocity latency (bag receive - image stamp, sim)', True,
          f"median {rep['latency_s']['median'] * 1e3:.1f} ms, p99 {rep['latency_s']['p99'] * 1e3:.1f} ms, "
          f"max {rep['latency_s']['max'] * 1e3:.1f} ms", 'REPORTED')

    # ---------------- REEF on vision
    for ax, e in (('x', rex), ('y', rey)):
        r = rms(e[sr])
        judged(f'REEF {ax}_dot vs truth on vision input only, steady segments', r <= LIM['reef_rmse'],
               f'RMSE {r:.4f} m/s, bias {np.mean(e[sr]):+.4f} m/s (n {sr.sum()})', f"RMSE <= {LIM['reef_rmse']} m/s")
        rep[f'reef_{ax}'] = dict(rmse=r, bias=float(np.mean(e[sr])), n=int(sr.sum()))
    path = json.loads((run / 'camera_check.json').read_text()).get('estimator_inputs', {}) \
        if (run / 'camera_check.json').exists() else {}
    fed = {t: p for t, p in path.items() if p} if isinstance(path, dict) and 'error' not in path else {}
    allowed = {'/x3/reef/imu/data': ['/x3_imu_adapter'], '/x3/range': ['/range_sensor'],
               VELOCITY_TOPIC: ['/x3/reef/rgbd_to_velocity_node']}
    others = {t: p for t, p in fed.items() if t not in allowed and t not in ('/clock', '/parameter_events')}
    ok = bool(fed) and fed.get(VELOCITY_TOPIC) == allowed[VELOCITY_TOPIC] and not others and \
        all(fed.get(t) in (None, p) for t, p in allowed.items())
    c.add("data path (ROS graph): REEF's horizontal velocity input is rgbd_to_velocity only", ok,
          f'estimator subscriptions with publishers: {fed}; other fed inputs: {others}',
          f'{VELOCITY_TOPIC} <- rgbd_to_velocity_node only; no other velocity source')
    diag = data.get('/x3/reef/diagnostics', [])
    if diag:
        kv = {x.key: x.value for x in diag[-1][1].status[0].values}
        rep['reef_counts'] = {k: kv.get(k) for k in ('xy_observations_accepted', 'xy_fusions', 'xy_gates', 'z_gates',
                                                     'rgbd_ignored_by_enable_measurements', 'correction_c1')}
    c.add('REEF gate decisions and fusion counts (end of run)', True, str(rep.get('reef_counts')), 'REPORTED')

    # ---------------- degraded segments
    geo_t = a['odom'][:, 0]
    geo = camera_geometry(a, params, geo_t)
    lost = lost_intervals(health)
    inside = [(lo, hi) for lo, hi in lost]
    bad_v = int(sum(((v[:, 0] > lo) & (v[:, 0] < hi)).sum() for lo, hi in inside))
    bad_c = int(sum(((cam_t > lo) & (cam_t < hi)).sum() for lo, hi in inside))
    judged('no velocity published while the odometry is LOST (whole run)', bad_v == 0 and bad_c == 0,
           f'{len(lost)} LOST intervals; cam_to_init inside {bad_c}, rgbd_to_velocity inside {bad_v}', '0 and 0')
    rep['lost_intervals'] = [[lo, hi] for lo, hi in lost]
    hstamp = np.array([h['t'] for h in health])
    pub_t = np.array([h['t'] for h in health if h['published']])
    for seg, start_phase, end_phase, ev_cond, end_cond in (
            ('weak texture', 'weak_left', 'weak_return', geo['right_y'] > RICH_Y_MAX, geo['axis_y'] <= RICH_Y_MAX),
            ('depth loss', 'far_back', 'far_return', geo['depth'] > FAR_CLIP, geo['depth'] <= FAR_CLIP)):
        w0, w1 = window(phases, start_phase), window(phases, end_phase)
        if not (w0 and w1):
            judged(f'{seg}: segment present', False, 'phases missing', 'present')
            continue
        t_ev = first_time(geo_t, ev_cond & (geo_t >= w0[0]) & (geo_t < w0[1]))
        t_end = first_time(geo_t, end_cond & (geo_t >= w1[0]) & (geo_t < w1[1])) if t_ev else None
        hold = following_hover(phases, end_phase)
        info = dict(event=t_ev, end=t_end)
        rep[seg] = info
        if t_ev is None or t_end is None:
            judged(f'{seg}: geometric event reached', False, f'event {t_ev}, end {t_end}', 'reached')
            continue
        lost_h = [h['t'] for h in health if h['state'] == 'LOST' and w0[0] <= h['t'] <= t_ev + LIM['detect_s']]
        det = lost_h[0] - t_ev if lost_h else None
        judged(f'{seg}: health shows the loss', bool(lost_h),
               f'event at {t_ev:.2f} s; first LOST at {lost_h[0]:.2f} s ({det:+.2f} s)' if lost_h else
               f'event at {t_ev:.2f} s; no LOST by event + {LIM["detect_s"]} s', f"LOST by event + {LIM['detect_s']} s")
        info['detection_s'] = det
        seg_lost = [(lo, hi) for lo, hi in lost if w0[0] <= lo <= hold['t_end']]
        resume = seg_lost[-1][1] if seg_lost else None
        ok = resume is not None and math.isfinite(resume) and resume <= t_end + LIM['resume_s']
        judged(f'{seg}: velocity output resumes', ok,
               f'segment end {t_end:.2f} s; resumed at {resume:.2f} s ({resume - t_end:+.2f} s)'
               if resume is not None and math.isfinite(resume) else f'segment end {t_end:.2f} s; not resumed',
               f"<= end + {LIM['resume_s']} s")
        info['resume'] = resume
        if seg_lost:
            lo = seg_lost[0][0]
            hi = resume if math.isfinite(resume) else reef[-1, 0]
            i0 = np.searchsorted(reef[:, 0], lo)
            i1 = max(np.searchsorted(reef[:, 0], hi) - 1, 0)
            s0, s1 = reef[min(i0, len(reef) - 1), 3:5], reef[i1, 3:5]
            ok = bool(np.all(s1 > s0))
            judged(f"{seg}: REEF's horizontal variance grows while lost", ok,
                   f'sigma(x_dot, y_dot) {s0[0]:.4f}, {s0[1]:.4f} at {lo:.2f} s -> {s1[0]:.4f}, {s1[1]:.4f} m/s '
                   f'at {reef[i1, 0]:.2f} s', 'both grow')
            info['sigma'] = [s0.tolist(), s1.tolist()]
        rw = (t_end + LIM['recover_after_s'], t_end + LIM['recover_after_s'] + LIM['recover_window_s'])
        m = mask_of(reef[:, 0], [rw]) & rvalid
        rx, ry = rms(rex[m]), rms(rey[m])
        judged(f'{seg}: REEF velocity error back within limits', m.any() and rx <= LIM['reef_rmse'] and
               ry <= LIM['reef_rmse'], f'RMSE over [{rw[0]:.2f}, {rw[1]:.2f}] s: x {rx:.4f}, y {ry:.4f} m/s',
               f"<= {LIM['reef_rmse']} per axis over [end + 3 s, end + 4 s]")
        lost_frames = sum(1 for h in health if h['state'] == 'LOST' and w0[0] <= h['t'] <= hold['t_end'])
        info['lost_frames'] = lost_frames

    # ---------------- faults run (labelled test hooks)
    rep['faults'] = []
    for f in faults:
        kind, phase, off, span = f.split()[:4]
        w = window(phases, phase)
        if not w:
            c.add(f'fault {f}: phase present', False, 'phase missing', 'present')
            continue
        t0, t1 = w[0] + float(off), w[0] + float(off) + float(span)
        m = mask_of(v[:, 0], [(t0, t1 + LIM['fault_tail_s'])]) & vvalid
        worst = float(max(np.max(np.abs(ex[m])), np.max(np.abs(ey[m])))) if m.any() else math.nan
        fin = bool(np.all(np.isfinite(v[m, 2:4])))
        c.add(f'fault "{f}": vision velocity finite, error <= {LIM["fault_err"]} m/s (window + {LIM["fault_tail_s"]} s)',
              bool(m.any()) and fin and worst <= LIM['fault_err'],
              f'{m.sum()} msgs in [{t0:.2f}, {t1 + LIM["fault_tail_s"]:.2f}] s, max |error| {worst:.3f} m/s',
              f"finite, <= {LIM['fault_err']} m/s per axis")
        mi = mask_of(v[:, 0], [(t0, t1)])
        mg = mask_of(v[:, 0], [(t0 - 0.5, t1 + 0.5)])   # the gap of a drop spans the window edges
        gaps = np.diff(v[mg, 0]) if mg.sum() > 1 else np.array([math.nan])
        hd = [h for h in health if t0 <= h['t'] < t1]
        d = dict(fault=f, window=[t0, t1], msgs=int(mi.sum()), rate_hz=float(mi.sum() / (t1 - t0)),
                 latency_ms_median=float(np.median(lat[mi]) * 1e3) if mi.any() else None,
                 latency_ms_max=float(np.max(lat[mi]) * 1e3) if mi.any() else None,
                 max_stamp_gap_s=float(np.nanmax(gaps)), max_abs_error=worst,
                 dropped_by_hook=(health[-1]['dropped']), health_msgs_in_window=len(hd))
        rep['faults'].append(d)
        # the hook must have acted (otherwise the fault items would pass vacuously)
        if kind == 'drop':
            acted = d['dropped_by_hook'] > 0 and d['max_stamp_gap_s'] >= float(span) - 0.1
            how = (f"frames dropped by the hook (run total) {d['dropped_by_hook']}, largest stamp gap "
                   f"{d['max_stamp_gap_s']:.3f} s")
            need = f'frames dropped and a stamp gap >= span - 0.1 s ({float(span) - 0.1:.1f} s)'
        else:
            delay = float(f.split()[4])
            acted = d['latency_ms_median'] is not None and d['latency_ms_median'] >= delay * 1e3
            how = f"median latency {d['latency_ms_median']} ms in the window"
            need = f'median latency >= {delay * 1e3:.0f} ms'
        c.add(f'fault "{f}": the test hook acted', acted, how, need)
        c.add(f'fault "{f}": latency and rate', True,
              f"rate {d['rate_hz']:.2f} Hz in the window ({d['msgs']} msgs), latency median "
              f"{d['latency_ms_median'] or math.nan:.0f} ms, max {d['latency_ms_max'] or math.nan:.0f} ms, largest stamp gap "
              f"{d['max_stamp_gap_s']:.3f} s, "
              f"frames dropped by the hook (run total) {d['dropped_by_hook']}", 'REPORTED')
        tw = mask_of(v[:, 0], [(t1, t1 + 1.0)])
        rep['faults'][-1]['first_error_after'] = float(np.max(np.abs(ex[tw]))) if tw.any() else None

    # ---------------- performance (wall time; separate from sim-time correctness)
    ms = np.array([h['ms'] for h in health])
    wall = np.array([h['wall'] for h in health])
    sim_span = hstamp[-1] - hstamp[0]
    wall_span = wall[-1] - wall[0] if np.all(np.isfinite(wall[[0, -1]])) else math.nan
    rtf = sim_span / wall_span if wall_span and wall_span > 0 else math.nan
    n_info = len(data.get('/x3/camera/camera_info', []))
    rep['performance'] = dict(odometry_ms_mean=float(ms.mean()), odometry_ms_p95=float(np.percentile(ms, 95)),
                              odometry_ms_max=float(ms.max()), frames=len(health),
                              processing_share_of_wall=float(ms.sum() / 1e3 / wall_span) if wall_span else None,
                              image_rate_sim_hz=float(n_info / result.get('scenario_duration_s', math.nan)),
                              frame_rate_sim_hz=float(len(health) / sim_span), rtf=float(rtf))
    p = rep['performance']
    c.add('performance (wall time)', True,
          f"odometry {p['odometry_ms_mean']:.1f} ms mean, p95 {p['odometry_ms_p95']:.1f}, max {p['odometry_ms_max']:.1f} "
          f"ms per frame; processing {100 * (p['processing_share_of_wall'] or math.nan):.0f} % of one core (approx. CPU); "
          f"camera_info {p['image_rate_sim_hz']:.2f} Hz, odometry frames {p['frame_rate_sim_hz']:.2f} Hz (sim); "
          f"RTF {p['rtf']:.3f}", 'REPORTED')
    rep['health_summary'] = dict(frames=len(health), published=int(sum(h['published'] for h in health)),
                                 lost_events=health[-1]['lost_events'],
                                 depth_tracks_median_steady=float(np.median(
                                     [h['depth_tracks'] for h in health if mask_of(np.array([h['t']]), steady)[0]])))
    series = dict(v=v, ex=ex, ey=ey, vtx=vtx, vty=vty, reef=reef, rtx=rtx, rty=rty, hstamp=hstamp, pub_t=pub_t,
                  health=health, lost=lost, steady=steady)
    return c, rep, series


def plots(series, phases, out):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    v, reef = series['v'], series['reef']
    fig, ax = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    for i, (k, tk, rk) in enumerate(((2, 'vtx', 'rtx'), (3, 'vty', 'rty'))):
        ax[i].plot(v[:, 0], v[:, k], '.', ms=2, label='vision (rgbd_to_velocity)')
        ax[i].plot(v[:, 0], series[tk], '-', lw=0.8, label='truth (scoring only)')
        ax[i].plot(reef[:, 0], reef[:, 1 + i], '-', lw=0.8, label='REEF')
        ax[i].set_ylabel(f'body-level {"xy"[i]} velocity [m/s]')
        ax[i].legend(loc='upper right', fontsize=7)
    ax[2].plot(reef[:, 0], reef[:, 3], label='REEF sigma(x_dot)')
    ax[2].plot(reef[:, 0], reef[:, 4], label='REEF sigma(y_dot)')
    ax[2].set_ylabel('sigma [m/s]')
    ax[2].legend(fontsize=7)
    h = series['health']
    ax[3].plot([x['t'] for x in h], [x['depth_tracks'] for x in h], label='depth tracks')
    ax[3].plot([x['t'] for x in h], [x['inliers'] for x in h], label='inliers')
    ax[3].set_ylabel('odometry health')
    ax[3].legend(fontsize=7)
    ax[3].set_xlabel('sim time [s]')
    for a in ax:
        for lo, hi in series['lost']:
            a.axvspan(lo, min(hi, v[-1, 0]), color='red', alpha=0.15)
        for w in series['steady']:
            a.axvspan(w[0], w[1], color='green', alpha=0.06)
        for p in phases:
            a.axvline(p['t_start'], color='grey', lw=0.3)
    fig.suptitle('P08 vision open loop (red: odometry LOST; green: steady segments). ' + LABEL[:120] + '...',
                 fontsize=8)
    fig.tight_layout()
    fig.savefig(out / 'vision_velocity.png', dpi=110)
    plt.close(fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('run_dir', type=Path)
    ap.add_argument('--out', type=Path)
    args = ap.parse_args(argv)
    out = args.out or args.run_dir / 'analysis'
    out.mkdir(parents=True, exist_ok=True)
    print(LABEL)
    c, rep, series = analyze(args.run_dir)
    for i in c.items:
        tag = 'REPORTED' if i['criterion'] == 'REPORTED' else ('PASS' if i['ok'] else 'FAIL')
        print(f"{tag:8} {i['name']}: {i['detail']}" + ('' if tag == 'REPORTED' else f"  [{i['criterion']}]"))
    judged = [i for i in c.items if i['criterion'] != 'REPORTED']
    npass = sum(i['ok'] for i in judged)
    print(f'vision analysis: {npass}/{len(judged)} judged items PASS '
          f'({len(c.items) - len(judged)} REPORTED, not counted)')
    (out / 'analysis_vision.json').write_text(json.dumps(dict(label=LABEL, ok=c.ok, items=c.items, report=rep),
                                                         indent=1, default=float))
    if series is not None:
        plots(series, json.loads((args.run_dir / 'scenario_result.json').read_text())['phases'], out)
    return 0 if c.ok else 1


if __name__ == '__main__':
    sys.exit(main())
