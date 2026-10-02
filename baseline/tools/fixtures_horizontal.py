#!/usr/bin/env python3
"""Horizontal-estimator fixtures for the P05 port comparison (h01-h10).

    fixtures_horizontal.py OUT_DIR

Same conventions as fixtures.py (P02): specific force in body FRD, attitude
quaternion (x, y, z, w) of the body in NED, positive range, body-level mocap
and RGB-D velocity, integer-nanosecond stamps, IMU 500 Hz, sonar 50 Hz,
velocity observations 100 Hz. Deterministic: no randomness.

New here: nonzero yaw (the P02 fixtures all have yaw 0) with roll and pitch,
lateral motion, and the observation cases of ACCEPTANCE.md 4d / faults:
outliers, duplicates, out-of-order stamps, dropouts, RC switching both ways,
full update with RGB-D, landing reset of the horizontal filter, enable_xy
false, and RGB-D ignored by enable_measurements.

Motion is specified in the BODY-LEVEL frame (NED rotated by yaw). With a
constant yaw, the specific force in the body frame is C_rp (a_level - g),
independent of yaw, while the quaternion carries the yaw; the estimator
must remove it again (body-level frame). The quaternion is built here as
q_z(yaw) q_y(pitch) q_x(roll) and checked against fixtures.quat_rp at yaw 0.
"""
import hashlib
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fixtures as f  # noqa: E402  (imported after the sys.path insert above)

H = f.T_HOVER


def qmul(a, b):   # (w, x, y, z)
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def quat_rpy(roll, pitch, yaw):
    """Attitude of the body in NED, 3-2-1 Euler angles, as (x, y, z, w)."""
    qz = (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))
    qy = (math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0)
    qx = (math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0)
    w, x, y, z = qmul(qmul(qz, qy), qx)
    return (x, y, z, w)


def _check_quaternion_convention():
    for r, p in ((0.1, -0.2), (-0.3, 0.25), (0.0, 0.0)):
        a, b = quat_rpy(r, p, 0.0), f.quat_rp(r, p)
        assert all(abs(u - v) < 1e-15 for u, v in zip(a, b)), (a, b)


def flight(name, doc, segments, yaw=0.0, att=None, mocap=None, rgbd=None, sonar=True, h0=0.12, extra=None):
    """segments: (duration, a_up, a_x_level, a_y_level) after the standard takeoff.
    att(t) -> (roll, pitch) (default level); yaw constant. mocap/rgbd(s, t, v) add
    observation events at 100 Hz with the true body-level velocity v = (vx, vy);
    extra(s, t, state) may add any event at each IMU step."""
    s = f.Scenario(name, doc, 500)
    segs = [(d, a_up, a_x, 0.0) for d, a_up, a_x in f.TAKEOFF] + segments
    dt = 1.0 / 500
    h, v, vx, vy = h0, 0.0, 0.0, 0.0
    t_primer, i = None, 0
    for dur, a_up, a_x, a_y in segs:
        for _ in range(round(dur * 500)):
            t = i * dt
            r_ang, p_ang = att(t) if att else (0.0, 0.0)
            if t_primer is None and h >= 0.25:
                t_primer = t
            fb = list(f.specific_force_body((a_x, a_y, -a_up), r_ang, p_ang, f.G))
            if t_primer is not None and t_primer <= t < t_primer + 0.2:
                fb[2] += 1.5 if (i % 2) else -1.5
            s.imu(t, fb, quat_rpy(r_ang, p_ang, yaw))
            s.truth.append((s.ns(t), h, v, vx, vy, r_ang, p_ang))
            if sonar and i % 10 == 0:
                cos_tilt = math.cos(r_ang) * math.cos(p_ang)
                s.range(t, h / cos_tilt)
            if i % 5 == 0:
                if mocap:
                    mocap(s, t, (vx, vy))
                if rgbd:
                    rgbd(s, t, (vx, vy))
            if extra:
                extra(s, t, (h, v, vx, vy))
            h += v * dt + 0.5 * a_up * dt * dt
            v += a_up * dt
            vx += a_x * dt
            vy += a_y * dt
            i += 1
    return s


def near(t, t0):
    return abs(t - t0) < 1e-9


MOVE = [(1.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.4, -0.3), (2.0, 0.0, 0.0, 0.0), (1.0, 0.0, -0.4, 0.3), (1.0, 0.0, 0.0, 0.0)]


def tilt(t):
    return (math.radians(8), math.radians(-5)) if t >= H + 1.0 else (0.0, 0.0)


def true_mocap(s, t, v):
    s.mocap_twist(t, v[0], v[1])


def scenarios():
    out = []   # (scenario, param_set)
    out.append((flight('h01_yaw30_tilt_mocap', 'yaw 30 deg, roll 8/pitch -5 deg from T_HOVER+1 s, body-level motion in x and y; true mocap velocity',
                       MOVE, yaw=math.radians(30), att=tilt, mocap=true_mocap), 'default'))
    out.append((flight('h02_yaw_m135_pitch12', 'yaw -135 deg, pitch 12 deg / roll -6 deg from T_HOVER+1 s, diagonal motion; true mocap velocity',
                       MOVE, yaw=math.radians(-135),
                       att=lambda t: (math.radians(-6), math.radians(12)) if t >= H + 1.0 else (0.0, 0.0),
                       mocap=true_mocap), 'default'))

    def outliers(s, t, v):
        vx = v[0] + (2.0 if any(near(t, H + d) for d in (1.5, 2.5, 3.5)) else 0.0)
        s.mocap_twist(t, vx, v[1])
    out.append((flight('h03_yaw90_mocap_outliers', 'yaw 90 deg, motion; mocap vx + 2 m/s at T_HOVER+1.5/2.5/3.5 s (gated)',
                       MOVE, yaw=math.radians(90), mocap=outliers), 'default'))

    def dup_ooo(s, t, v):
        s.mocap_twist(t, v[0], v[1])
        if H + 1.0 <= t < H + 2.0:
            s.mocap_twist(t, v[0], v[1])                    # duplicate
        if near(t, H + 3.0):
            s.events.append((s.ns(t) + 1000, 2, f'mocap_twist {s.ns(t) - 20_000_000} {v[0] + 0.05!r} {v[1]!r} 0.02 0.02'))
            # delivered after the IMU at t, stamped 20 ms earlier (out of order)
    out.append((flight('h04_mocap_duplicates_out_of_order', 'yaw 45 deg; mocap duplicated from T_HOVER+1 s to +2 s; one late message stamped 20 ms in the past at +3 s',
                       MOVE, yaw=math.radians(45), mocap=dup_ooo), 'default'))

    def dropout(s, t, v):
        if not (H + 1.5 <= t < H + 4.5):
            s.mocap_twist(t, v[0], v[1])
    out.append((flight('h05_mocap_dropout', 'yaw -60 deg, motion; no mocap from T_HOVER+1.5 s to +4.5 s',
                       MOVE, yaw=math.radians(-60), mocap=dropout), 'default'))

    def both(s, t, v):
        s.rgbd(t, 0.8 * v[0], 0.8 * v[1])
        s.mocap_twist(t, v[0], v[1])
        if near(t, H + 2.0):
            s.rc(t, [1500, 1500, 1000, 1500, 1000, 1000, 2000, 1000])   # channel 6 high: mocap
        if near(t, H + 4.0):
            s.rc(t, [1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000])   # channel 6 low: back to RGB-D
    out.append((flight('h06_rc_switch_both_ways', 'yaw 20 deg; rgbd = 0.8 v, mocap = v; RC channel 6 high at T_HOVER+2 s, low at +4 s',
                       MOVE, yaw=math.radians(20), mocap=both), 'rc_switch'))

    def rgbd_cov(s, t, v):
        cov = 1.0 if H + 1.0 <= t < H + 2.0 else 0.01
        s.events.append((s.ns(t), 2, f'rgbd {s.ns(t)} {v[0]!r} {v[1]!r} {cov!r} {cov!r}'))
    out.append((flight('h07_rgbd_full_update_cov', 'yaw 10 deg; RGB-D only, full update; covariance 1.0 from T_HOVER+1 s to +2 s, else 0.01',
                       MOVE, yaw=math.radians(10), mocap=rgbd_cov), 'rgbd_full_update'))

    land = [(1.0, 0.0, 0.4, 0.0), (1.0, 0.0, -0.4, 0.0), (1.0, 0.0, 0.0, 0.0)] + \
        [(1.0, -0.5, 0.0, 0.0), (1.8, 0.0, 0.0, 0.0), (1.0, 0.5, 0.0, 0.0), (2.0, 0.0, 0.0, 0.0)]
    out.append((flight('h08_landing_xy_reset', 'yaw 75 deg; move forward and stop, descend 1.40 m to the ground, 2 s on the ground (landing resets both filters)',
                       land, yaw=math.radians(75), mocap=true_mocap), 'default'))
    out.append((flight('h09_xy_disabled', 'yaw 30 deg, motion; enable_xy = false (propagation only, no update, no landing reset)',
                       MOVE, yaw=math.radians(30), mocap=true_mocap), 'xy_disabled'))
    out.append((flight('h10_rgbd_measurements_off', 'yaw 0; RGB-D only with enable_measurements = false (every RGB-D message ignored)',
                       MOVE, mocap=lambda s, t, v: s.rgbd(t, v[0], v[1])), 'rgbd_off'))
    return out


def node_settings(set_name):
    base = f.node_settings('default')
    extra = {'default': {}, 'rc_switch': f.node_settings('rc_switch'),
             'rgbd_full_update': {'enable_rgbd': True, 'enable_mocap_xy': False, 'enable_measurements': True,
                                  'enable_partial_update': False},
             'xy_disabled': {'enable_xy': False},
             'rgbd_off': {'enable_rgbd': True, 'enable_mocap_xy': False, 'enable_measurements': False}}[set_name]
    return {**base, **extra}


def main():
    _check_quaternion_convention()
    out = Path(sys.argv[1])
    (out / 'params').mkdir(parents=True, exist_ok=True)
    master = f.shipped_params('master')
    param_set = {}
    for s, set_name in scenarios():
        s.write(out)
        param_set[s.name] = set_name
    for set_name in sorted(set(param_set.values())):
        f.write_params(out / 'params' / f'common_{set_name}_master.params', {**master, **node_settings(set_name)})
    files = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in sorted(out.rglob('*')) if p.is_file() and p.name != 'manifest.json'}
    (out / 'manifest.json').write_text(json.dumps({'param_set_for_scenario': param_set, 'files': files}, indent=2))
    print(f'wrote {len(param_set)} horizontal scenarios to {out}')


if __name__ == '__main__':
    main()
