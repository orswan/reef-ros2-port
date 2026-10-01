"""Independent, step-wise re-derivation of rgbd_to_velocity (P08).

Written from docs/VISION.md section 2, not from the C++ sources of the
original or the port. For each odometry message it takes the reference row
after the previous message (rgbd_columns.txt), applies the documented
steps, and returns the predicted row. Uses numpy (IEEE semantics: a NaN
propagates; division by a large DT is ordinary).
"""
import math

import numpy as np


def rot(v, w):
    """C = I - 2 w [v]x + 2 [v]x^2 for a quaternion with vector part v and scalar w."""
    s = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], float)
    return np.eye(3) - 2 * w * s + 2 * s @ s


M = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)          # camera level -> NED level
FLIP = np.diag([-1.0, -1.0, 1.0])                                    # camera frame -> image frame


def step(prev, tok):
    """Predicted row after one 'odom sec nsec px py pz qx qy qz qw' message."""
    r = dict(prev)
    sec, nsec = int(float(tok[0])), int(float(tok[1]))
    p = np.array([float(x) for x in tok[2:5]])
    q = [float(x) for x in tok[5:9]]
    t = float(sec) + 1e-9 * float(nsec)
    r['counter'] = prev['counter'] + 1
    r['current_time_stamp'] = t
    dt = t - prev['previous_time_stamp']
    r['DT'] = dt
    if not dt >= 1.0 / 30.0:   # gate (Q1, Q8); NaN DT is rejected too
        return r
    v, w = np.array(q[:3]), q[3]
    r['beta_x'], r['beta_y'], r['beta_z'], r['beta_0'] = v[0], v[1], v[2], w
    c_init_cam = rot(v, w)
    yaw213 = math.atan2(c_init_cam[2, 0], c_init_cam[2, 2])
    c_level = np.array([[math.cos(yaw213), 0, -math.sin(yaw213)], [0, 1, 0],
                        [math.sin(yaw213), 0, math.cos(yaw213)]])
    for i in range(3):
        for j in range(3):
            r[f'C_init_cam_{i}{j}'] = c_level[i, j]
    c_init_ned_level = M @ c_level
    qb = [prev[f'q_b2c_{i}'] for i in range(4)]
    c_body_cam = rot(np.array(qb[:3]), qb[3])
    c_lb = c_body_cam.T @ FLIP @ c_init_cam @ c_init_ned_level.T
    r['pitch'] = -math.asin(c_lb[0, 2])
    r['roll'] = math.atan2(c_lb[0, 2], c_lb[2, 2])     # sic (Q5)
    yaw321 = math.atan2(c_lb[0, 1], c_lb[0, 0])
    r['yaw'] = yaw321
    c_body_level = np.array([[math.cos(yaw321), math.sin(yaw321), 0], [-math.sin(yaw321), math.cos(yaw321), 0],
                             [0, 0, 1]])
    p_prev = np.array([prev[f'prev_pos_{a}'] for a in 'xyz'])
    v_prev = np.array([prev[f'prev_vel_{a}'] for a in 'xyz'])
    with np.errstate(all='ignore'):
        v_est = (p - p_prev) * (1 / dt)
        alpha = prev['alpha']
        v_filt = alpha * v_est + (1 - alpha) * v_prev
    for a, i in zip('xyz', range(3)):
        r[f'cur_pos_{a}'] = p[i]
        r[f'est_vel_{a}'] = v_est[i]
        r[f'filt_vel_{a}'] = v_filt[i]
        r[f'prev_pos_{a}'] = p[i]
        r[f'prev_vel_{a}'] = v_filt[i]
    r['previous_time_stamp'] = t
    # Init-frame message: published before the covariances are set (Q6).
    r['init_sec'], r['init_nsec'] = sec, nsec
    r['init_vx'], r['init_vy'], r['init_vz'] = v_filt
    r['init_cov0'], r['init_cov7'], r['init_cov14'] = prev['msg_cov0'], prev['msg_cov7'], prev['msg_cov14']
    for i in range(3):
        r[f'init_S_up_{i}'] = prev[f'S_up_{i}']
        r[f'init_S_lo_{i}'] = prev[f'S_lo_{i}']
    R = c_body_level @ c_init_ned_level
    with np.errstate(all='ignore'):
        v_body = R @ v_filt
    xc, yc = prev['x_vel_cov'], prev['y_vel_cov']
    cov_body = R @ np.diag([xc, yc, 0.0]) @ R.T
    for i in range(3):
        for j in range(3):
            r[f'cov_body_{i}{j}'] = cov_body[i, j]
    sz = math.sqrt(max(cov_body[2, 2], 0.0)) if cov_body[2, 2] >= 0 else float('nan')
    for a, i in zip('xyz', range(3)):
        r[f'body_vel_{a}'] = v_body[i]
    r['msg_sec'], r['msg_nsec'] = sec, nsec
    r['msg_vx'], r['msg_vy'], r['msg_vz'] = v_body
    r['msg_cov0'], r['msg_cov7'], r['msg_cov14'] = xc, yc, cov_body[2, 2]
    bounds = (math.sqrt(xc), math.sqrt(yc), sz)     # unrotated x/y (Q4)
    for i in range(3):
        r[f'S_up_{i}'] = v_body[i] + 3 * bounds[i]
        r[f'S_lo_{i}'] = v_body[i] - 3 * bounds[i]
    r['n_init'] = prev['n_init'] + 1
    r['n_body'] = prev['n_body'] + 1
    r['counter'] = 0
    return r


def initial_row(columns, params):
    r = {c: 0.0 for c in columns}
    r.update(n_init=0, n_body=0, counter=0, alpha=params.get('alpha', 1.0),
             x_vel_cov=params.get('x_vel_covariance', 0.01), y_vel_cov=params.get('y_vel_covariance', 0.01))
    for i, v in enumerate(params.get('body_to_camera_quat', [0.0] * 4)):
        r[f'q_b2c_{i}'] = v
    for a, v in zip('xyz', params.get('body_to_camera_trans', [0.0] * 3)):
        r[f't_b2c_{a}'] = v
    return r


def read_params(text):
    p = {}
    for line in text.splitlines():
        if not line or line.startswith('#'):
            continue
        name, kind, *vals = line.split()
        v = [float(x) for x in vals]
        p[name] = v if kind == 'list' else v[0]
    return p
