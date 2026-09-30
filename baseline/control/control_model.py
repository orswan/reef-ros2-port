"""Independent, step-wise re-derivation of the reef_control equations (P06).

Written from docs/CONTROL_CHAIN.md (sections 3-5), not from the C++ sources
of the original or the port. For each event k it takes the reference
harness state after event k-1 (a row of control_columns.txt), applies the
documented operations for event k, and returns the predicted row k. The
gain configuration (which the rows do not hold completely, e.g. the
integrator switches) is tracked by the model from the parameter file and
the gain events.

IEEE semantics are kept where Python would raise: division by zero and
overflow in exp follow C (inf/nan), and C++ std::min/std::max pass a NaN
first argument through.
"""
import math

import numpy as np

# Gains.cfg of reef_control 12237b76: name -> (default, min, max); transcribed
# independently of the harness stand-in and of the port's table.
CFG = {
    'uP': (0, 0, 2), 'uI': (0, 0, 1), 'uD': (0, 0, 0.5),
    'vP': (0, 0, 2), 'vI': (0, 0, 1), 'vD': (0, 0, 0.5),
    'wP': (0, 0, 2), 'wI': (0, 0, 1), 'wD': (0, 0, 0.5), 'uvtau': (0, 0, 1),
    'kp': (0, 0, 1), 'deadzone': (0, 0, 0.5), 'max_vel': (0, 0, 3), 'center_point': (0, 0, 2.0), 'alpha': (0, 0, 2),
    'dP': (0, 0, 5), 'dI': (0, 0, 1), 'dD': (0, 0, 0.5), 'nedtau': (0, 0, 1),
    'yawP': (0, 0, 2), 'yawI': (0, 0, 1), 'yawD': (0, 0, 0.5), 'yawtau': (0, 0, 1),
    'yawRateP': (0, 0, 2), 'yawRateI': (0, 0, 1), 'yawRateD': (0, 0, 0.5), 'yawRatetau': (0, 0, 1),
    'max_u': (0, 0, 2.5), 'max_v': (0, 0, 2.5), 'max_w': (0, 0, 2.5), 'max_d': (0, 0, 2.0),
    'max_n': (0, 0, 1.5), 'max_e': (0, 0, 1.5), 'max_yaw_rate': (0, 0, 0.5),
}
CFG_BOOL = {'xIntegrator': True, 'uIntegrator': True}
PIDS = ('d', 'w', 'yaw', 'u', 'v')


def div(a, b):
    with np.errstate(all='ignore'):
        return float(np.float64(a) / np.float64(b))


def cexp(x):
    with np.errstate(all='ignore'):
        return float(np.exp(np.float64(x)))


def cmax(a, b):   # std::max(a, b): (a < b) ? b : a
    return b if a < b else a


def cmin(a, b):   # std::min(a, b): (b < a) ? b : a
    return b if b < a else a


def f32(x):
    with np.errstate(all='ignore'):
        return float(np.float32(x))


def duration(s1, n1, s0, n0):
    """ROS 1 (Time - Time).toSec(): difference in ns, split with C truncation,
    normalized to nsec in [0, 1e9), then sec + 1e-9 * nsec."""
    d = (s1 * 1_000_000_000 + n1) - (s0 * 1_000_000_000 + n0)
    q = abs(d) // 1_000_000_000
    sec = q if d >= 0 else -q
    nsec = d - sec * 1_000_000_000
    if nsec < 0:
        nsec += 1_000_000_000
        sec -= 1
    return float(sec) + 1e-9 * float(nsec)


def load_params(text):
    """Parameter server as the original reads it: private '~' names; roscpp
    getParam(double) accepts integers, getParam(bool) only booleans."""
    store = {}
    for line in text.splitlines():
        if not line or line.startswith('#'):
            continue
        name, kind, value = line.split()
        store[name] = (kind, value)
    return store


def get_double(store, name):
    if name not in store:
        return None
    kind, v = store[name]
    return float(v) if kind in ('double', 'int') else None


def get_bool(store, name):
    if name not in store:
        return None
    kind, v = store[name]
    return int(v) != 0 if kind == 'bool' else None


def initial_config(store):
    """dynamic_reconfigure: defaults, then the node's parameters, then clamped."""
    cfg = {}
    for n, (d, lo, hi) in CFG.items():
        v = get_double(store, '~' + n)
        v = float(d) if v is None else v
        cfg[n] = min(max(v, lo), hi)
    for n, d in CFG_BOOL.items():
        v = get_bool(store, '~' + n)
        cfg[n] = d if v is None else v
    return cfg


def apply_gains(row, cfg):
    """gainsCallback: set the lookup-table values and the five PIDs."""
    xi = 1.0 if cfg['xIntegrator'] else 0.0
    ui = 1.0 if cfg['uIntegrator'] else 0.0
    row.update(kp=cfg['kp'], deadzone=cfg['deadzone'], vel_max=cfg['max_vel'], x_0=cfg['center_point'],
               alpha=cfg['alpha'], sigma=0.1)
    gains = {'d': (cfg['dP'], xi * cfg['dI'], cfg['dD'], cfg['nedtau'], cfg['max_d'], -0.7),
             'yaw': (cfg['yawP'], cfg['yawI'], cfg['yawD'], cfg['yawtau'], 2.0, -2.0),
             'u': (cfg['uP'], ui * cfg['uI'], cfg['uD'], cfg['uvtau'], cfg['max_u'], -cfg['max_u']),
             'v': (cfg['vP'], ui * cfg['vI'], cfg['vD'], cfg['uvtau'], cfg['max_v'], -cfg['max_v']),
             'w': (cfg['wP'], ui * cfg['wI'], cfg['wD'], cfg['uvtau'], cfg['max_w'], -cfg['max_w'])}
    for n, (p, i, d, tau, mx, mn) in gains.items():
        row.update({f'{n}_kp': p, f'{n}_ki': i, f'{n}_kd': d, f'{n}_tau': tau, f'{n}_max': mx, f'{n}_min': mn})


def initial_row(columns, store, cfg):
    row = {c: 0 for c in columns}
    row['st_qw'] = 0.0   # ROS 1 quaternion default: all zero
    row['max_roll'] = get_double(store, '~max_roll')
    row['max_pitch'] = get_double(store, '~max_pitch')
    row['max_yaw_rate'] = get_double(store, '~max_yaw_rate')
    row['face_target'] = int(bool(get_bool(store, 'face_target')))
    row['fly_fixed_wing'] = int(bool(get_bool(store, 'fly_fixed_wing')))
    apply_gains(row, cfg)
    return row


def pid(r, n, xc, x, dt):
    """SimplePID::computePID(x_c, x, dt): band-limited derivative of the
    state, integrator, output clamp, back-calculation test (K1-K3)."""
    kp, ki, kd, tau = r[f'{n}_kp'], r[f'{n}_ki'], r[f'{n}_kd'], r[f'{n}_tau']
    mx, mn = r[f'{n}_max'], r[f'{n}_min']
    integ, diff, ls = r[f'{n}_integrator'], r[f'{n}_differentiator'], r[f'{n}_last_state']
    if dt > 0.0:
        diff = div(2 * tau - dt, 2 * tau + dt) * diff + div(2, 2 * tau + dt) * (x - ls)
        xdot = diff
    else:
        xdot = 0.0
    ls = x
    e = xc - x
    p = e * kp
    i = 0.0
    d = 0.0
    out = None
    if dt == 0.0 or not math.isfinite(e):
        le, ls, out = e, x, 0.0
    else:
        if kd > 0.0:
            d = kd * xdot
        if ki > 0.0:
            integ += e * dt
            i = ki * integ
        le, ls = e, x
        u = p + d + i
        us = mx if u > mx else (mn if u < mn else u)
        if u != us and abs(i) > abs(u - p + d) and ki > 0.0:
            integ = div(us - p + d, ki)
        out = us
    r.update({f'{n}_integrator': integ, f'{n}_differentiator': diff, f'{n}_last_error': le, f'{n}_last_state': ls})
    return out


def step(prev, kind, tok, cfg):
    """Predicted row after one event; cfg is updated for gain events."""
    r = dict(prev)
    if kind == 'des':
        r['ds_sec'], r['ds_nsec'] = int(tok[0]), int(tok[1])
        for i, flag in enumerate(('attitude_valid', 'position_valid', 'velocity_valid', 'acceleration_valid', 'altitude_only')):
            r['ds_' + flag] = int(int(tok[2 + i]) != 0)
        vals = [float(x) for x in tok[7:23]]
        for j, v in enumerate(('pose', 'vel', 'acc', 'att')):
            for a, ax in enumerate(('x', 'y', 'z', 'yaw')):
                r[f'ds_{v}_{ax}'] = vals[4 * j + a]
    elif kind == 'status':
        r['armed'] = int(int(tok[0]) != 0)
        r['initialized'] = r['armed']
    elif kind == 'flying':
        r['is_flying'] = int(int(tok[0]) != 0)
        r['initialized'] = int(r['is_flying'] and r['armed'])
    elif kind == 'pose':
        x, y, qx, qy, qz, qw = (float(t) for t in tok)
        r.update(st_px=x, st_py=y, st_qx=qx, st_qy=qy, st_qz=qz, st_qw=qw)
    elif kind == 'gain':
        name, v = tok[0], float(tok[1])
        if name in CFG:
            d, lo, hi = CFG[name]
            cfg[name] = min(max(v, lo), hi)
        else:
            cfg[name] = v != 0
        apply_gains(r, cfg)
    elif kind == 'est':
        s, n = int(tok[0]), int(tok[1])
        z, zd, vx, vy = (float(t) for t in tok[2:6])
        r.update(st_sec=s, st_nsec=n, st_pz=z, st_vz=zd, st_vx=vx, st_vy=vy)
        dt = duration(s, n, r['prev_sec'], r['prev_nsec'])
        r['dt'] = dt
        r['prev_sec'], r['prev_nsec'] = s, n
        if dt > 0.0000001:
            control(r, dt)
    return r


def control(r, dt):
    if not r['initialized']:
        for n in PIDS:
            r[f'{n}_integrator'] = 0.0
    qx, qy, qz, qw = r['st_qx'], r['st_qy'], r['st_qz'], r['st_qw']
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    r['current_yaw'] = yaw
    r['ds_vel_z'] = pid(r, 'd', r['ds_pose_z'], r['st_pz'], dt)
    r['ds_acc_z'] = pid(r, 'w', r['ds_vel_z'], r['st_vz'], dt)
    if r['ds_position_valid']:
        if r['face_target']:
            r['ds_pose_yaw'] = r['theta'] + yaw
        r['ds_vel_yaw'] = pid(r, 'yaw', r['ds_pose_yaw'], yaw, dt)
        lookup(r, yaw)
        r['ds_velocity_valid'] = 1
    if r['ds_velocity_valid']:
        r['ds_acc_x'] = pid(r, 'u', r['ds_vel_x'], r['st_vx'], dt)
        r['ds_acc_y'] = pid(r, 'v', r['ds_vel_y'], r['st_vy'], dt)
    for c in list(r):
        if c.startswith('ds_'):
            r['cs_' + c[3:]] = r[c]
    r['n_controller_state'] += 1
    r['phi_desired'] = r['ds_acc_y']
    r['theta_desired'] = -r['ds_acc_x']
    r['thrust'] = -r['ds_acc_z']
    r['cmd_mode'] = 2
    r['cmd_F'] = f32(cmin(cmax(r['thrust'], 0.0), 1.0))
    if not r['ds_attitude_valid'] and not r['ds_altitude_only']:
        r['cmd_ignore'] = 0
        r['cmd_x'] = f32(cmin(cmax(r['phi_desired'], -1.0 * r['max_roll']), r['max_roll']))
        r['cmd_y'] = f32(cmin(cmax(r['theta_desired'], -1.0 * r['max_pitch']), r['max_pitch']))
        r['cmd_z'] = f32(cmin(cmax(r['ds_vel_yaw'], -1.0 * r['max_yaw_rate']), r['max_yaw_rate']))
    elif r['ds_altitude_only']:
        r['cmd_ignore'] = 7
    else:
        r['cmd_ignore'] = 0
        r['cmd_x'] = f32(r['ds_att_x'])
        r['cmd_y'] = f32(r['ds_att_y'])
        r['cmd_z'] = f32(r['ds_att_yaw'])
    r['n_command'] += 1


def lookup(r, yaw):
    ex = r['ds_pose_x'] - r['st_px']
    ey = r['ds_pose_y'] - r['st_py']
    dist = math.sqrt(ex * ex + ey * ey)
    if dist < r['deadzone']:
        req = 0.0
    else:
        req = div(r['vel_max'] * 1, 1 + r['kp'] * cexp(div(-(dist - r['x_0']), r['alpha'])))
    theta = math.atan2(ey, ex) - yaw
    r['theta'] = theta
    if r['fly_fixed_wing']:
        m = cexp(div(-(r['ds_vel_yaw'] * r['ds_vel_yaw']), 2 * r['sigma'] * r['sigma']))
        m = cmin(m + 0.1, 1.0)
    else:
        m = 1.0
    r['ds_vel_x'] = m * req * math.cos(theta)
    r['ds_vel_y'] = m * req * math.sin(theta)
