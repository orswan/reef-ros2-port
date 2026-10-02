#!/usr/bin/env python3
"""Deterministic, timestamped event fixtures and parameter sets for the
REEF reference harness.

    fixtures.py OUT_DIR

Writes OUT_DIR/<scenario>.events, OUT_DIR/<scenario>.truth.csv, and
OUT_DIR/params/<set>_<variant>.params, plus OUT_DIR/manifest.json with the
SHA-256 of every file. No randomness is used, so the output is identical on
every run.

Conventions (those of the original ROS 1 system, see BASELINE_DECISION.md):
- IMU linear_acceleration is SPECIFIC FORCE in the body FRD frame; at rest,
  level, it reads (0, 0, -g).
- IMU orientation is the body attitude quaternion (x, y, z, w); the estimator
  forms C_NED_to_body = reef_msgs::quaternion_to_rotation(q) (world to body).
- Sonar reports positive range; the estimator uses z = -range (NED).
- Mocap twist is body-level velocity (m/s); mocap pose z is NED (m).
- Time stamps are integer nanoseconds. Default IMU rate 500 Hz (dt = 2 ms,
  the original estimator_dt default), sonar 50 Hz, mocap 100 Hz.
- Flight scenarios start on the ground (range 0.12 m) and climb to about
  1.52 m by T_HOVER = 4.8 s. The original gates the first airborne range
  against its ground initial state (z0 = -0.25) and would reject a start at
  altitude, so an airborne start is not a meaningful master fixture.
"""
import hashlib
import json
import math
import sys
from pathlib import Path

import yaml

G = 9.81            # exact specific-force magnitude used unless a scenario says otherwise
T0 = 1_000_000_000  # first stamp: 1 s (avoids zero stamps)
MAX_RANGE = 7.65

REF = Path(__file__).resolve().parents[2] / 'reference' / 'reef_estimator'
COMMITS = {'master': 'e4179f48c3f26e22bd1366b71ee1e117ce2f5f7f',
           'sim': '95987b5118b624208910d9e51424300022e1f512'}


def quat_rp(roll, pitch):
    """Attitude quaternion (x, y, z, w) for 3-2-1 Euler angles, yaw 0 (body to NED)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (sr * cp, cr * sp, -sr * sp, cr * cp)


def c_ned_to_body(roll, pitch):
    """World-to-body rotation for roll/pitch (yaw 0): (R_y(pitch) R_x(roll))^T, row-major."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    r_nb = [[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]]   # body -> NED
    return [[r_nb[j][i] for j in range(3)] for i in range(3)]


def specific_force_body(a_ned, roll=0.0, pitch=0.0, g=G):
    """Specific force in body FRD for kinematic acceleration a_ned (NED): C (a - g_ned)."""
    f_ned = (a_ned[0], a_ned[1], a_ned[2] - g)
    c = c_ned_to_body(roll, pitch)
    return tuple(sum(c[i][k] * f_ned[k] for k in range(3)) for i in range(3))


class Scenario:
    """Collects events and truth. Times in seconds relative to T0."""

    def __init__(self, name, doc, imu_hz=500):
        self.name, self.doc, self.imu_hz = name, doc, imu_hz
        self.events, self.truth = [], []

    def ns(self, t):
        return T0 + round(t * 1e9)

    def imu(self, t, f, q, nan=False):
        if nan:
            self.events.append((self.ns(t), 0, f'imu_nan {self.ns(t)}'))
        else:
            self.events.append((self.ns(t), 0, 'imu {} {!r} {!r} {!r} {!r} {!r} {!r} {!r}'.format(
                self.ns(t), *f, *q)))

    def range(self, t, r, max_range=MAX_RANGE):
        self.events.append((self.ns(t), 1, f'range {self.ns(t)} {r!r} {max_range!r}'))

    def mocap_twist(self, t, vx, vy, cov=0.02):
        self.events.append((self.ns(t), 2, f'mocap_twist {self.ns(t)} {vx!r} {vy!r} {cov!r} {cov!r}'))

    def rgbd(self, t, vx, vy, cov=0.02):
        self.events.append((self.ns(t), 2, f'rgbd {self.ns(t)} {vx!r} {vy!r} {cov!r} {cov!r}'))

    def mocap_pose(self, t, z_ned):
        self.events.append((self.ns(t), 2, f'mocap_pose {self.ns(t)} {z_ned!r}'))

    def rc(self, t, values):
        self.events.append((self.ns(t), 3, f'rc {self.ns(t)} ' + ' '.join(str(v) for v in values)))

    def write(self, out):
        # IMU first at equal stamps, then sonar, then mocap/rgbd, then rc (the
        # order is part of the fixture and deterministic).
        self.events.sort(key=lambda e: (e[0], e[1]))
        (out / f'{self.name}.events').write_text(
            f'# {self.name}: {self.doc}\n' + '\n'.join(e[2] for e in self.events) + '\n')
        (out / f'{self.name}.truth.csv').write_text(
            't_ns,h_up,v_up,vx,vy,roll,pitch\n' + '\n'.join(','.join(repr(x) for x in r) for r in self.truth) + '\n')


TAKEOFF = [(1.0, 0.0, 0.0), (1.0, 0.5, 0.0), (1.8, 0.0, 0.0), (1.0, -0.5, 0.0)]  # (duration s, a_up, a_x)
T_HOVER = sum(seg[0] for seg in TAKEOFF)   # 4.8 s: hover begins at about 1.52 m


def flight(name, doc, segments, h0=0.12, roll=None, accel_bias_z=None, g=G, imu_hz=500, sonar=True,
           sonar_override=None, mocap=None, takeoff=True, nan_at=None, primer=None):
    """Vertical/horizontal scenario from piecewise-constant accelerations.

    segments: list of (duration, a_up, a_x) after the standard takeoff (if
    takeoff=True, the vehicle starts on the ground at range h0 and climbs to
    about 1.52 m by T_HOVER). roll(t) gives a constant-per-segment roll angle.
    accel_bias_z(t) is added to the body-z specific force (FRD).
    sonar_override(t, r) may replace a range sample (None drops it).
    mocap(s, t, truth) may add mocap/rgbd/rc events at 100 Hz.
    """
    s = Scenario(name, doc, imu_hz)
    segs = (TAKEOFF if takeoff else []) + segments
    dt = 1.0 / imu_hz
    h, v, x, vx = h0, 0.0, 0.0, 0.0
    t_primer = None
    i = 0
    for dur, a_up, a_x in segs:
        n = round(dur * imu_hz)
        for _ in range(n):
            t = i * dt
            r_ang = roll(t) if roll else 0.0
            # Takeoff primer (motor vibration): the original declares takeoff only
            # when the accel-magnitude variance over 20 samples is >= 0.5 while
            # range >= 0.25 m. Applied for 0.2 s when the climb first passes 0.25 m.
            if (takeoff if primer is None else primer) and t_primer is None and h >= 0.25:
                t_primer = t
            f = list(specific_force_body((a_x, 0.0, -a_up), r_ang, 0.0, g))
            if t_primer is not None and t_primer <= t < t_primer + 0.2:
                f[2] += 1.5 if (i % 2) else -1.5
            if accel_bias_z:
                f[2] += accel_bias_z(t)
            s.imu(t, f, quat_rp(r_ang, 0.0), nan=(nan_at is not None and abs(t - nan_at) < 1e-9))
            s.truth.append((s.ns(t), h, v, vx, 0.0, r_ang, 0.0))
            if sonar and i % round(imu_hz / 50) == 0:
                r = h / math.cos(r_ang)
                if sonar_override:
                    r = sonar_override(t, r)
                if r is not None:
                    s.range(t, r)
            if mocap and i % round(imu_hz / 100) == 0:
                mocap(s, t, (h, v, vx))
            h += v * dt + 0.5 * a_up * dt * dt
            v += a_up * dt
            x += vx * dt + 0.5 * a_x * dt * dt
            vx += a_x * dt
            i += 1
    return s


def scenarios():
    H = T_HOVER
    out = []
    out.append(flight('s01_ground_stationary', 'on the ground, level, range 0.12 m; no takeoff',
                      [(4.0, 0.0, 0.0)], takeoff=False))
    out.append(flight('s02_takeoff_hover', 'takeoff (primer at 0.25 m), hover at ~1.52 m for 5 s',
                      [(5.0, 0.0, 0.0)]))
    out.append(flight('s03_const_accel_up', 'takeoff, hover 1 s, +0.5 m/s^2 up for 2 s, climb 1 s at 1 m/s',
                      [(1.0, 0.0, 0.0), (2.0, 0.5, 0.0), (1.0, 0.0, 0.0)]))
    out.append(flight('s04_accel_bias', 'takeoff, hover; body-z specific-force offset +0.2 m/s^2 from T_HOVER+1 s',
                      [(12.0, 0.0, 0.0)], accel_bias_z=lambda t: 0.2 if t >= H + 1.0 else 0.0))
    out.append(flight('s05_tilt_range', 'takeoff level, hover; roll 10 deg from T_HOVER+1 s (slant range)',
                      [(6.0, 0.0, 0.0)], roll=lambda t: math.radians(10) if t >= H + 1.0 else 0.0))
    xy = [(1.0, 0.0, 0.0), (1.0, 0.0, 0.5), (3.0, 0.0, 0.0)]   # accelerate to vx = 0.5, then hold
    out.append(flight('s06_mocap_xy_velocity', 'takeoff, hover, +0.5 m/s^2 forward for 1 s; mocap = true velocity',
                      xy, mocap=lambda s, t, tr: s.mocap_twist(t, tr[2], 0.0)))
    out.append(flight('s07_single_mocap', 'takeoff, hover; one mocap twist vx=0.2 at T_HOVER+1 s, then none',
                      [(3.0, 0.0, 0.0)],
                      mocap=lambda s, t, tr: s.mocap_twist(t, 0.2, 0.0) if abs(t - (H + 1.0)) < 1e-9 else None))
    out.append(flight('s08_standard_gravity', 'takeoff and hover with specific force magnitude 9.80665',
                      [(6.0, 0.0, 0.0)], g=9.80665))
    out.append(flight('s09_imu_250hz', 'as s03 with IMU at 250 Hz (dt 4 ms)',
                      [(1.0, 0.0, 0.0), (2.0, 0.5, 0.0), (1.0, 0.0, 0.0)], imu_hz=250))

    def outliers(t, r):
        for dt_, val in ((1.0, 3.5), (2.0, float('-inf')), (3.0, float('inf'))):
            if abs(t - (H + dt_)) < 1e-9:
                return val
        return r
    out.append(flight('s10_range_outliers', 'hover; range 3.5 m, then -inf, then +inf at T_HOVER+1/2/3 s',
                      [(5.0, 0.0, 0.0)], sonar_override=outliers))
    out.append(flight('s11_imu_nan', 'hover; one NaN IMU sample at T_HOVER+1 s', [(3.0, 0.0, 0.0)],
                      nan_at=H + 1.0))
    out.append(flight('s12_mocap_z', 'takeoff and hover with mocap pose z = -h (NED) at 100 Hz, no sonar',
                      [(4.0, 0.0, 0.0)], sonar=False, mocap=lambda s, t, tr: s.mocap_pose(t, -tr[0])))

    def rgbd_and_mocap(s, t, tr):
        s.rgbd(t, 0.8 * tr[2], 0.0)          # biased RGB-D velocity
        s.mocap_twist(t, tr[2], 0.0)         # true velocity
        if abs(t - (H + 3.0)) < 1e-9:
            s.rc(t, [1500, 1500, 1000, 1500, 1000, 1000, 2000, 1000])   # channel 6 high
    out.append(flight('s13_rc_mocap_switch', 'forward motion; rgbd = 0.8 v, mocap = v; RC channel 6 high at T_HOVER+3 s',
                      xy, mocap=rgbd_and_mocap))
    s14 = flight('s14_full_update', 'as s06, run with enable_partial_update = false', xy,
                 mocap=lambda s, t, tr: s.mocap_twist(t, tr[2], 0.0))
    s14.name = 's14_full_update'
    out.append(s14)
    # S15: already airborne at 1.52 m when the estimator starts (primer at t = 0).
    out.append(flight('s15_airborne_start', 'estimator starts while hovering at 1.52 m (no climb from the ground)',
                      [(5.0, 0.0, 0.0)], h0=1.52, takeoff=False, primer=True))
    return out


# Scenario -> parameter set (see params below).
PARAM_SET = {'s12_mocap_z': 'mocap_z', 's13_rc_mocap_switch': 'rc_switch', 's14_full_update': 'full_update'}


def shipped_params(variant):
    """basic_params, xy_est_params, z_est_params as committed at the variant's revision."""
    import subprocess
    merged = {}
    for f in ('basic_params', 'xy_est_params', 'z_est_params'):
        text = subprocess.run(['git', '-C', str(REF), 'show', f'{COMMITS[variant]}:params/{f}.yaml'],
                              capture_output=True, check=True, text=True).stdout
        merged.update(yaml.safe_load(text))
    return merged


def node_settings(set_name):
    """Launch-level settings, as in sim_helper sim_estimator.launch (sonar + mocap XY)."""
    s = {'enable_rgbd': False, 'enable_sonar': True, 'enable_mocap_xy': True, 'enable_mocap_z': False,
         'enable_mocap_switch': False, 'debug_mode': True}
    if set_name == 'mocap_z':
        s.update(enable_sonar=False, enable_mocap_z=True)
    if set_name == 'rc_switch':
        s.update(enable_rgbd=True, enable_mocap_switch=True, enable_measurements=True)
    if set_name == 'full_update':
        s.update(enable_partial_update=False)
    return s


def write_params(path, values):
    lines = []
    for k, v in sorted(values.items()):
        if isinstance(v, bool):
            lines.append(f'{k} bool {"true" if v else "false"}')
        elif isinstance(v, (int, float)):
            lines.append(f'{k} double {float(v)!r}')
        elif isinstance(v, str):
            lines.append(f'{k} string {v}')
        elif isinstance(v, list):
            lines.append(f'{k} list ' + ' '.join(repr(float(x)) for x in v))
        else:
            raise TypeError(k)
    path.write_text('\n'.join(lines) + '\n')


def main():
    out = Path(sys.argv[1])
    (out / 'params').mkdir(parents=True, exist_ok=True)
    master_values = shipped_params('master')
    for variant in COMMITS:
        own = shipped_params(variant)
        for set_name in ('default', 'mocap_z', 'rc_switch', 'full_update'):
            for kind, base in (('shipped', own), ('common', master_values)):
                write_params(out / 'params' / f'{kind}_{set_name}_{variant}.params',
                             {**base, **node_settings(set_name)})
    for s in scenarios():
        s.write(out)
    files = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in sorted(out.rglob('*')) if p.is_file() and p.name != 'manifest.json'}
    (out / 'manifest.json').write_text(json.dumps(
        {'param_set_for_scenario': PARAM_SET, 'files': files}, indent=2))
    print(f'wrote {len([f for f in files if f.endswith(".events")])} scenarios to {out}')


if __name__ == '__main__':
    main()
