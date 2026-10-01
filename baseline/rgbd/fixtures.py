#!/usr/bin/env python3
"""Deterministic odometry fixtures for the rgbd_to_velocity comparison (P08).

    fixtures.py OUT_DIR [--lock F | --write-lock F]

Formats: baseline/rgbd/rgbd_ref_main.cpp (params: <name> double|list <values>;
events: odom sec nsec px py pz qx qy qz qw on cam_to_init, DEMO's camera
convention: x left, y up, z forward). Parameter sets come from the shipped
files of rgbd_to_velocity b7637198 (params/kiwi_camera.yaml,
params/rgbd_to_velocity_params.yaml) and demo_rgbd 934f7295
(params/kiwi_camera.yaml, a second calibration of the same camera).
Each stream targets a case of ACCEPTANCE.md (vision, P08) and VISION.md Q1-Q10.
Exit: 0 ok, 1 lock mismatch, 2 usage.
"""
import hashlib
import json
import math
import random
import sys
from pathlib import Path

KIWI = dict(alpha=1.0, x_vel_covariance=0.01, y_vel_covariance=0.01,
            body_to_camera_quat=[0.5495504171301594, -0.4573648426502658, 0.5078083888936552, -0.4805646469610469],
            body_to_camera_trans=[0.1354765750332004, 0.0175314092839309, -0.0455417178618858])
DEMO_KIWI = dict(KIWI, body_to_camera_quat=[-0.4817987248385569, 0.5029865744522301, -0.4772858983488609, 0.5357916254497384],
                 body_to_camera_trans=[0.1416058281853279, 0.0208741030516409, -0.0427634441660232])


def params(p):
    out = []
    for k, v in p.items():
        if v is None:
            continue
        if isinstance(v, list):
            out.append(f'{k} list ' + ' '.join(repr(float(x)) for x in v))
        else:
            out.append(f'{k} double {float(v)!r}')
    return '\n'.join(out) + '\n'


def stamp_ns(ns):
    return ns // 1_000_000_000, ns % 1_000_000_000


def quat_axis(axis, angle):
    s = math.sin(angle / 2)
    return (axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2))


def qmul(a, b):   # (x, y, z, w) Hamilton product a*b
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def odom(ns, p, q):
    s, n = stamp_ns(ns)
    return f'odom {s} {n} ' + ' '.join(repr(float(x)) for x in (*p, *q))


def at_rate(n, hz, t0_ns=100 * 10**9):
    """Stamps of n frames at hz, each rounded to the nanosecond as a driver would."""
    return [t0_ns + round(k * 1e9 / hz) for k in range(n)]


def build():
    fx = {}
    ident = (0.0, 0.0, 0.0, 1.0)
    # r01: exactly 30 Hz (Q1: every other frame rejected), forward motion (DEMO z).
    st = at_rate(90, 30.0)
    fx['r01_30hz_exact'] = (params(KIWI), [odom(t, (0.0, 0.0, 0.5 * (t - st[0]) * 1e-9), ident) for t in st])
    # r02: other rates, motion left and up.
    ev, t = [], 100 * 10**9
    for hz, n in ((20.0, 40), (25.0, 40), (100.0 / 3.0, 40), (15.0, 30)):
        for k in range(n):
            t += round(1e9 / hz)
            s = (t - 100 * 10**9) * 1e-9
            ev.append(odom(t, (0.3 * s, 0.1 * math.sin(s), 0.2 * s), ident))
    fx['r02_rates'] = (params(KIWI), ev)
    # r03: camera yaw (about DEMO y, up) with motion in x/z.
    st = at_rate(120, 20.0)
    ev = []
    for k, t in enumerate(st):
        s = k / 20.0
        ev.append(odom(t, (0.4 * math.sin(0.5 * s), 0.0, 0.6 * s), quat_axis((0, 1, 0), 0.2 * s)))
    fx['r03_yaw_motion'] = (params(KIWI), ev)
    # r04: tilted camera (pitch about x, roll about z) and yaw, 3-D motion.
    st = at_rate(120, 20.0)
    ev = []
    for k, t in enumerate(st):
        s = k / 20.0
        q = qmul(qmul(quat_axis((0, 1, 0), 0.3 + 0.1 * s), quat_axis((1, 0, 0), 0.15 * math.sin(s))),
                 quat_axis((0, 0, 1), 0.1 * math.cos(0.7 * s)))
        ev.append(odom(t, (0.2 * s, 0.05 * math.sin(2 * s), 0.5 * s), q))
    fx['r04_tilt'] = (params(KIWI), ev)
    # r05: alpha 0.5, unequal covariances, yaw != 0 (Q4 unrotated covariances, Q6 stale bounds).
    fx['r05_alpha_covariance'] = (params(dict(KIWI, alpha=0.5, x_vel_covariance=0.02, y_vel_covariance=0.005)),
                                  fx['r03_yaw_motion'][1])
    # r06: stamp anomalies: duplicates, backward, jitter, the 1/30 boundary.
    base = 100 * 10**9
    steps = [round(1e9 / 30)] * 5 + [0, 0, -10**7, 50_000_000, 33_333_333, 33_333_334, 33_333_332, 1, 66_666_667,
                                     -2 * 10**9, 3 * 10**9, 40_000_000]
    ev, t = [], base
    for i, d in enumerate(steps):
        t += d
        ev.append(odom(t, (0.01 * i, 0.0, 0.02 * i), ident))
    fx['r06_stamp_anomalies'] = (params(KIWI), ev)
    # r07: first message at a wall-clock stamp (Q2: DT = stamp).
    st = at_rate(30, 15.0, t0_ns=1_790_000_000 * 10**9)
    fx['r07_first_large_stamp'] = (params(KIWI), [odom(t, (1.0, 2.0, 3.0 + 0.1 * k), ident) for k, t in enumerate(st)])
    # r08: the demo_rgbd calibration of the same camera.
    fx['r08_demo_extrinsics'] = (params(DEMO_KIWI), fx['r04_tilt'][1])
    # r09: one NaN position (Q9 latch).
    st = at_rate(40, 15.0)
    ev = []
    for k, t in enumerate(st):
        p = (0.1 * k, 0.0, 0.2 * k) if k != 15 else (float('nan'), 0.0, 3.0)
        ev.append(odom(t, p, ident))
    fx['r09_nan_position'] = (params(KIWI), ev)
    # r10: long randomized stream: random unit quaternions, random walk, mixed stamp steps.
    rng = random.Random(20261001)
    ev, t, p = [], 100 * 10**9, [0.0, 0.0, 0.0]
    for _ in range(5000):
        t += rng.choice([16_666_667, 33_333_333, 33_333_334, 50_000_000, 66_666_667, 0, -5_000_000, 100_000_000])
        t = max(t, 0)
        p = [x + rng.gauss(0, 0.02) for x in p]
        q = [rng.gauss(0, 1) for _ in range(4)]
        nq = math.sqrt(sum(x * x for x in q))
        ev.append(odom(t, p, tuple(x / nq for x in q)))
    fx['r10_random_long'] = (params(dict(KIWI, alpha=0.8)), ev)
    # Parameter case: no extrinsics (the original warns and uses zeros; the port refuses, Q10).
    fx['p01_missing_extrinsics'] = (params(dict(KIWI, body_to_camera_quat=None, body_to_camera_trans=None)),
                                    fx['r01_30hz_exact'][1][:5])
    return fx


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    out = Path(argv[1])
    out.mkdir(parents=True, exist_ok=True)
    digests = {}
    for name, (p, ev) in build().items():
        for ext, text in (('params', p), ('events', '\n'.join(ev) + '\n')):
            (out / f'{name}.{ext}').write_text(text)
            digests[f'{name}.{ext}'] = hashlib.sha256(text.encode()).hexdigest()
    if '--write-lock' in argv:
        Path(argv[argv.index('--write-lock') + 1]).write_text(json.dumps(
            {'_comment': 'SHA-256 of the rgbd_to_velocity fixtures (baseline/rgbd/fixtures.py)', 'files': digests},
            indent=1, sort_keys=True) + '\n')
    if '--lock' in argv:
        lock = json.loads(Path(argv[argv.index('--lock') + 1]).read_text())['files']
        bad = sorted(set(lock) ^ set(digests)) + sorted(n for n in lock if n in digests and lock[n] != digests[n])
        if bad:
            print('FIXTURE LOCK MISMATCH: ' + ', '.join(bad))
            return 1
    print(f'wrote {len(digests)} files to {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
