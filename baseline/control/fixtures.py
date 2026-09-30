#!/usr/bin/env python3
"""Deterministic event fixtures for the controller comparison (P06).

    fixtures.py OUT_DIR            write <name>.params and <name>.events
    fixtures.py OUT_DIR --lock F   also compare SHA-256 with lock file F
    fixtures.py OUT_DIR --write-lock F

Formats: baseline/control/control_ref_main.cpp. Streams are open loop
(scripted estimates), which is enough for step-by-step parity; each covers
one required case of ACCEPTANCE.md (control, P06). Parameter sets come from
the shipped reef_control 12237b76 files (config/legacy/*.yaml) as the
original read them: the gains live in the node's private namespace
(reef_control_pid); face_target and fly_fixed_wing are read globally, so
their values inside the namespace are ignored (K12).
Exit: 0 ok, 1 lock mismatch, 2 usage.
"""
import hashlib
import json
import math
import random
import sys
from pathlib import Path

DT_NS = 4_000_000          # 250 Hz, as the X3 scenario's IMU and REEF estimate rate
T0 = 100                   # fixture start [s]

# Shipped gain files (reef_control 12237b76 params/*.yaml), transcribed.
QUAD = dict(kp=0.8, deadzone=0.1, max_vel=1.5, center_point=1.0, alpha=0.4,
            uP=0.3, uI=0.025, uD=0.0, vP=0.34, vI=0.03, vD=0.005,
            wP=0.6, wI=0.2, wD=0.0, uvtau=0.15, dP=0.75, dI=0.05, dD=0.0, nedtau=0.15,
            yawP=0.7, yawI=0.08, yawD=0.01, yawtau=0.15,
            max_roll=0.25, max_pitch=0.25, max_yaw_rate=2.0,
            max_u=1.0, max_v=1.0, max_w=1.0, max_d=1.0)
Y6 = dict(QUAD, uP=0.28, uI=0.06, uD=0.01, wP=0.54, wI=0.1, dP=1.0, dI=0.1)
KIWI = dict(kp=0.8, deadzone=0.1, max_vel=1.5, center_point=1.0, alpha=0.4,
            uP=0.34, uI=0.01, uD=0.01, vP=0.42, vI=0.0, vD=0.015,
            wP=0.3, wI=0.0, wD=0.005, uvtau=0.15, dP=0.7, dI=0.02, dD=0.06, nedtau=0.15,
            yawP=1.08, yawI=0.08, yawD=0.01, yawtau=0.15,
            max_roll=0.25, max_pitch=0.25, max_yaw_rate=2.0,
            max_u=1.0, max_v=1.0, max_w=1.0, max_d=1.0)


def params(gains, globals_=None, private_extra=None):
    """Parameter lines: gains private ('~'); xIntegrator/uIntegrator are
    booleans here (the shipped files give integers, which roscpp's bool
    getParam rejects, so the cfg default true applied: same value)."""
    lines = [f'~{k} double {v!r}' for k, v in gains.items()]
    lines += ['~xIntegrator bool 1', '~uIntegrator bool 1']
    for k, (kind, v) in (private_extra or {}).items():
        lines.append(f'~{k} {kind} {v}')
    for k, (kind, v) in (globals_ or {}).items():
        lines.append(f'{k} {kind} {v}')
    return '\n'.join(lines) + '\n'


def stamp(k, t0=T0, dt_ns=DT_NS):
    ns = t0 * 1_000_000_000 + k * dt_ns
    return ns // 1_000_000_000, ns % 1_000_000_000


def f(x):
    return repr(float(x))


def est(k, z, zd, vx, vy, **kw):
    s, n = stamp(k, **kw)
    return f'est {s} {n} {f(z)} {f(zd)} {f(vx)} {f(vy)}'


def des(k=0, av=0, pv=0, vv=0, acv=0, ao=0, pose=(0, 0, 0, 0), vel=(0, 0, 0, 0), acc=(0, 0, 0, 0), att=(0, 0, 0, 0)):
    s, n = stamp(k)
    vals = ' '.join(f(x) for x in (*pose, *vel, *acc, *att))
    return f'des {s} {n} {av} {pv} {vv} {acv} {ao} {vals}'


def quat_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def pose(x, y, yaw):
    qx, qy, qz, qw = quat_yaw(yaw)
    return f'pose {f(x)} {f(y)} {f(qx)} {f(qy)} {f(qz)} {f(qw)}'


def smooth(t, a, w, ph=0.0):
    return a * math.sin(w * t + ph)


def build():
    fx = {}

    # c01 altitude hold: armed, target 1 m up, altitude approaches with a damped oscillation.
    ev = ['status 1', des(pose=(0, 0, -1.0, 0))]
    for k in range(1, 1251):
        t = k * DT_NS * 1e-9
        z = -1.0 + math.exp(-t) * math.cos(3 * t)
        zd = -math.exp(-t) * math.cos(3 * t) - 3 * math.exp(-t) * math.sin(3 * t)
        ev.append(est(k, z, zd, 0, 0))
    fx['c01_altitude_hold'] = (params(QUAD), ev)

    # c02 climb saturation and integrator growth (K2: wD = dD = 0, no anti-windup).
    ev = ['status 1', des(pose=(0, 0, -3.0, 0))]
    for k in range(1, 1501):
        t = k * DT_NS * 1e-9
        z = 0.0 if t < 3 else -min(3.0, 0.5 * (t - 3))
        zd = 0.0 if t < 3 or z <= -3.0 else -0.5
        ev.append(est(k, z, zd, 0, 0))
    fx['c02_climb_saturation'] = (params(QUAD), ev)

    # c03 velocity mode, both axes, with D terms (Y6 gains: uD 0.01, vD 0.005).
    ev = ['status 1', des(vv=1, pose=(0, 0, -1, 0), vel=(0.5, -0.3, 0, 0.2))]
    for k in range(1, 1251):
        t = k * DT_NS * 1e-9
        if k == 600:
            ev.append(des(k, vv=1, pose=(0, 0, -1, 0), vel=(-0.4, 0.6, 0, -0.3)))
        ev.append(est(k, -1 + smooth(t, 0.05, 2), smooth(t, 0.1, 2, 1), 0.5 * (1 - math.exp(-t)) + smooth(t, 0.02, 11),
                      -0.3 * (1 - math.exp(-t)) + smooth(t, 0.03, 7)))
    fx['c03_velocity_mode'] = (params(Y6), ev)

    # c04 position mode: lookup table (sigmoid, deadzone), heading PID.
    ev = ['status 1', pose(0, 0, 0.3), des(pv=1, pose=(2.0, 1.0, -1.0, 0.5))]
    for k in range(1, 1501):
        t = k * DT_NS * 1e-9
        x, y = 2.0 * min(1.0, t / 5.0), 1.0 * min(1.0, t / 5.0)   # reaches the target: deadzone at the end
        if k % 5 == 0:
            ev.append(pose(x, y, 0.3 + 0.2 * min(1.0, t / 4)))
        ev.append(est(k, -1 + smooth(t, 0.02, 1), 0, 0.4 * math.exp(-t / 3), 0.2 * math.exp(-t / 3)))
    fx['c04_position_mode'] = (params(QUAD), ev)

    # c05 face_target (global) in position mode; theta from the previous step (K10).
    ev = ['status 1', pose(0, 0, 0.1), des(pv=1, pose=(1.5, -2.0, -1.0, 0.0))]
    for k in range(1, 751):
        t = k * DT_NS * 1e-9
        if k % 5 == 0:
            ev.append(pose(0.1 * t, -0.1 * t, 0.1 - 0.05 * t))
        ev.append(est(k, -1, 0, 0.1, -0.1))
    fx['c05_face_target'] = (params(QUAD, {'face_target': ('bool', 1)}), ev)

    # c06 fly_fixed_wing (global): velocity request scaled by the yaw rate.
    ev = ['status 1', pose(0, 0, 0.0), des(pv=1, pose=(3.0, 0.5, -1.0, 1.2))]
    for k in range(1, 751):
        t = k * DT_NS * 1e-9
        if k % 5 == 0:
            ev.append(pose(0.2 * t, 0.02 * t, 0.3 * t))
        ev.append(est(k, -1, 0, 0.2, 0.0))
    fx['c06_fly_fixed_wing'] = (params(QUAD, {'fly_fixed_wing': ('bool', 1)}), ev)

    # c07 heading across +-pi (K9): desired 3.1 rad, current -3.1 rad.
    ev = ['status 1', pose(0, 0, -3.1), des(pv=1, pose=(0.05, 0.0, -1.0, 3.1))]
    for k in range(1, 501):
        if k % 5 == 0:
            ev.append(pose(0, 0, -3.1 + 0.0005 * k))
        ev.append(est(k, -1, 0, 0, 0))
    fx['c07_heading_wrap'] = (params(QUAD), ev)

    # c08 attitude mode (K8): angles beyond max_roll/pitch, yaw sent as a rate.
    ev = ['status 1', des(av=1, pose=(0, 0, -1, 0), att=(0.4, -0.35, 0, 1.3))]
    for k in range(1, 401):
        if k == 200:
            ev.append(des(k, av=1, pose=(0, 0, -1, 0), att=(-0.1, 0.05, 0, -2.5)))
        ev.append(est(k, -1 + 0.001 * k, 0.01, 0.1, 0.1))
    fx['c08_attitude_mode'] = (params(QUAD), ev)

    # c09 altitude-only after velocity mode (K7: stale x, y, z).
    ev = ['status 1', des(vv=1, pose=(0, 0, -1, 0), vel=(0.3, 0.2, 0, 0.4))]
    for k in range(1, 601):
        if k == 300:
            ev.append(des(k, ao=1, pose=(0, 0, -1.2, 0), vel=(0.3, 0.2, 0, 0.4)))
        ev.append(est(k, -1, 0, 0.05 * math.sin(k / 50), 0.05 * math.cos(k / 50)))
    fx['c09_altitude_only'] = (params(QUAD), ev)

    # c10 no mode flags: acceleration and velocity.yaw pass through, clamped.
    ev = ['status 1', des(pose=(0, 0, -1, 0), vel=(0, 0, 0, 3.0), acc=(0.5, -0.1, 0, 0))]
    for k in range(1, 301):
        if k == 150:
            ev.append(des(k, pose=(0, 0, -1, 0), vel=(0, 0, 0, -0.7), acc=(-0.05, 0.4, 0, 0)))
        ev.append(est(k, -1, 0, 0, 0))
    fx['c10_flags_cleared'] = (params(QUAD), ev)

    # c11 arming and is_flying transitions (integrator clearing; last callback wins).
    ev = [des(vv=1, pose=(0, 0, -1, 0), vel=(0.3, 0.3, 0, 0))]
    seq = {100: 'status 1', 300: 'flying 1', 400: 'flying 0', 500: 'status 1', 600: 'status 0',
           700: 'flying 1', 800: 'status 1', 900: 'flying 1'}
    for k in range(1, 1001):
        if k in seq:
            ev.append(seq[k])
        ev.append(est(k, -0.5, 0.05, 0.0, 0.0))
    fx['c11_arming_flying'] = (params(QUAD), ev)

    # c12 first step with a wall-clock stamp (K4: dt = stamp - 0), already armed.
    ev = ['status 1', des(vv=1, pose=(0, 0, -1, 0), vel=(0.2, 0, 0, 0))]
    for k in range(0, 200):
        ev.append(est(k, -0.9, 0.0, 0.1, 0.0, t0=1790000000))
    fx['c12_first_step_huge_dt'] = (params(QUAD), ev)

    # c13 equal, backward, and sub-1e-7 stamp steps.
    ev = ['status 1', des(vv=1, pose=(0, 0, -1, 0), vel=(0.2, 0.1, 0, 0))]
    ns = T0 * 1_000_000_000
    for step in [4_000_000] * 20 + [0, 0, -4_000_000, 4_000_000, 100, 101, 99, 4_000_000, -1, 2, 4_000_000] + [4_000_000] * 20:
        ns += step
        ev.append(f'est {ns // 1_000_000_000} {ns % 1_000_000_000} -0.95 0.01 0.1 0.05')
    fx['c13_stamp_anomalies'] = (params(QUAD), ev)

    # c14 NaN estimate (K6), kiwi gains (wD, dD > 0: the differentiator stays NaN).
    ev = ['status 1', des(vv=1, pose=(0, 0, -1, 0), vel=(0.1, 0, 0, 0))]
    for k in range(1, 301):
        zd = float('nan') if k == 150 else 0.02
        ev.append(est(k, -0.98, zd, 0.05, 0.0))
    fx['c14_nan_estimate'] = (params(KIWI), ev)

    # c15 runtime gain changes (K11: integrators kept; integrator switch off freezes it).
    ev = ['status 1', des(vv=1, pose=(0, 0, -1.5, 0), vel=(0.4, -0.2, 0, 0))]
    changes = {200: ['gain wI 0.5'], 400: ['gain uIntegrator 0'], 600: ['gain max_w 0.4', 'gain uP 0.9'],
               800: ['gain uIntegrator 1', 'gain dP 2.5']}
    for k in range(1, 1001):
        ev.extend(changes.get(k, []))
        ev.append(est(k, -1.0 - 0.0002 * k, -0.02, 0.1, 0.0))
    fx['c15_gain_changes'] = (params(QUAD), ev)

    # c16 no desired state yet (hold z = 0), unarmed (K5: commands are published anyway).
    ev = [est(k, -0.2 + 0.001 * k, 0.1, 0.01, -0.01) for k in range(1, 301)]
    fx['c16_no_desired_unarmed'] = (params(QUAD), ev)

    # c17 kiwi file: face_target is global there (true), K12.
    ev = ['status 1', pose(0.0, 0.0, 0.0), des(pv=1, pose=(1.0, 1.0, -1.0, 0.0))]
    for k in range(1, 501):
        if k % 5 == 0:
            ev.append(pose(0.001 * k, 0.001 * k, 0.001 * k))
        ev.append(est(k, -1, 0, 0.1, 0.1))
    fx['c17_kiwi_global_flags'] = (params(KIWI, {'face_target': ('bool', 1), 'fly_fixed_wing': ('bool', 0)}), ev)

    # c18 quad file as shipped: face_target inside the namespace is ignored (K12).
    fx['c18_quad_namespaced_flags'] = (params(QUAD, private_extra={'face_target': ('bool', 1), 'fly_fixed_wing': ('bool', 1)}),
                                       fx['c17_kiwi_global_flags'][1])

    # c19 no estimate: desired states and status only, no command (missing input).
    fx['c19_no_estimate'] = (params(QUAD), ['status 1', des(pose=(0, 0, -1, 0)), 'flying 1',
                                            des(vv=1, vel=(1, 1, 0, 0)), pose(1, 2, 0.5)])

    # c20 long randomized stream: every event kind, all modes, gain changes.
    rng = random.Random(20260930)
    ev = []
    k = 0
    gains_ok = {'uP': (0, 2), 'wI': (0, 1), 'dD': (0, 0.5), 'yawP': (0, 2), 'max_u': (0, 2.5), 'alpha': (0.05, 2),
                'kp': (0, 1), 'deadzone': (0, 0.5), 'uvtau': (0, 1), 'xIntegrator': (0, 1)}
    for _ in range(20000):
        r = rng.random()
        if r < 0.80:
            k += rng.choice([1, 1, 1, 1, 2, 0]) if rng.random() < 0.98 else -1
            ev.append(est(max(k, 0), rng.uniform(-3, 0.5), rng.uniform(-1, 1), rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5)))
        elif r < 0.86:
            flags = [int(rng.random() < p) for p in (0.2, 0.3, 0.3, 0.1, 0.15)]
            v = [rng.uniform(-3, 3) for _ in range(16)]
            ev.append(des(max(k, 0), *flags, pose=v[0:4], vel=v[4:8], acc=v[8:12], att=v[12:16]))
        elif r < 0.92:
            ev.append(pose(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-math.pi, math.pi)))
        elif r < 0.95:
            ev.append(f'status {int(rng.random() < 0.8)}')
        elif r < 0.97:
            ev.append(f'flying {int(rng.random() < 0.7)}')
        else:
            name = rng.choice(sorted(gains_ok))
            lo, hi = gains_ok[name]
            val = int(rng.random() < 0.5) if name == 'xIntegrator' else round(rng.uniform(lo, hi), 6)
            ev.append(f'gain {name} {val!r}')
    fx['c20_random_long'] = (params(Y6), ev)

    # Parameter cases (both sides must refuse or clamp as specified).
    fx['k13_out_of_range_gain'] = (params(dict(QUAD, uP=3.0)), ['status 1', des(vv=1, vel=(0.5, 0, 0, 0)), est(1, 0, 0, 0, 0), est(2, 0, 0, 0, 0)])
    missing = dict(QUAD)
    del missing['max_roll']
    fx['p01_missing_max_roll'] = (params(missing), [est(1, 0, 0, 0, 0)])
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
            path = out / f'{name}.{ext}'
            path.write_text(text)
            digests[path.name] = hashlib.sha256(text.encode()).hexdigest()
    if '--write-lock' in argv:
        lock = Path(argv[argv.index('--write-lock') + 1])
        lock.write_text(json.dumps({'_comment': 'SHA-256 of the controller fixtures (baseline/control/fixtures.py)',
                                    'files': digests}, indent=1, sort_keys=True) + '\n')
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
