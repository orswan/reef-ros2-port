"""Independent, step-wise re-derivation of the REEF estimator equations.

Written from the equations in docs/BASELINE_DECISION.md (sections 3-6), not
from the C++ sources and not from any port. For each event k it takes the
reference harness state after event k-1, applies the documented operations
for event k, and returns the predicted state after event k. That prediction
is compared with the reference row k.

Schedule decisions that depend on internal counters are taken from the
reference row and checked separately:
- takeoff/landing transitions (the accelerometer-variance detector);
- the landing-reset counter.
Everything numeric is recomputed here.
"""
import math

import numpy as np

F32 = np.float32


def quat_to_c_ned_to_body(qx, qy, qz, qw):
    """World-to-body rotation: the transpose of the standard body-to-world matrix R(q)."""
    x, y, z, w = qx, qy, qz, qw
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return r.T


def kf_gain(P, H, R):
    return P @ H.T @ np.linalg.inv(H @ P @ H.T + R)


def full_update(x, P, H, z, R):
    K = kf_gain(P, H, R)
    return x + K @ (z - H @ x), (np.eye(len(x)) - K @ H) @ P


def partial_update(x, P, H, z, R, beta):
    """Partial-update Schmidt-Kalman filter (Brink): per-state blend with weights beta."""
    xp, Pp = full_update(x, P, H, z, R)
    alpha = 1.0 - beta
    gam = np.diag(alpha)
    return alpha * x + beta * xp, gam @ (P - Pp) @ gam + Pp


class Model:
    """Per-variant semantics (docs/BASELINE_DECISION.md, section 4)."""

    def __init__(self, variant, params):
        self.variant = variant
        p = params
        self.est_dt = p.get('estimator_dt', 0.002)
        self.partial = p['enable_partial_update']
        self.xy_Q = np.array(p['xy_Q']).reshape(6, 6) * self.est_dt ** 2
        self.xy_x0 = np.array(p['xy_x0'], float)
        self.xy_P0 = np.array(p['xy_P0']).reshape(6, 6)
        self.xy_beta = np.array(p['xy_beta'], float)
        self.z_Q0 = np.diag(p['z_Q'])
        self.z_x0 = np.array(p['z_x0'], float)
        self.z_P0 = np.array(p['z_P0']).reshape(3, 3)
        self.z_P0_fly = np.array(p['z_P0_flying']).reshape(3, 3)
        self.z_R0 = np.array(p['z_R0'], float).reshape(1, 1)
        self.z_R_fly = np.array(p['z_R_flying'], float).reshape(1, 1)
        self.z_beta = np.array(p['z_beta'], float)
        self.H_xy = np.hstack([np.eye(2), np.zeros((2, 4))])
        self.H_z = np.array([[1.0, 0.0, 0.0]])
        self.gate = {'range': p['mahalanobis_d_sonar'], 'mocap_z': p['mahalanobis_d_mocap_z'],
                     'mocap_xy': p['mahalanobis_d_mocap_velocity'], 'rgbd': p['mahalanobis_d_rgbd_velocity']}
        self.gated = variant == 'master'   # the simulation branch disabled all four chi-square gates

    # --- Z model -----------------------------------------------------------
    def z_matrices(self, dt_measured):
        if self.variant == 'master':
            dt = dt_measured                   # rebuilt every propagation from IMU stamps
            F = np.array([[1, dt, 0], [0, 1, -dt], [0, 0, 1]], float)
            Q = self.z_Q0 * dt
        else:
            dt = 0.002                         # fixed in the ZEstimator constructor
            F = np.array([[1, dt, dt * dt * 0.5], [0, 1, dt], [0, 0, 1]], float)
            Q = self.z_Q0 * self.est_dt        # scaled once with estimator_dt
        B = np.array([[0.0], [dt], [0.0]])
        G = np.array([[0, 0], [1, 0], [0, 1]], float)
        return F, B, G, Q

    # --- XY EKF propagation --------------------------------------------------
    def xy_propagate(self, x, P, C, f_body, g, bias_z_ned, dt):
        roll = math.atan2(C[1, 2], C[2, 2])
        pitch = -math.asin(C[0, 2])
        pe, re = pitch - x[2], roll - x[3]
        cp, sp, cr, sr = math.cos(pe), math.sin(pe), math.cos(re), math.sin(re)
        cbl = np.array([[cp, 0, -sp], [sr * sp, cr, sr * cp], [cr * sp, -sr, cr * cp]])
        bz_body = (C @ np.array([0.0, 0.0, float(F32(bias_z_ned))]))[2]   # passed as float32
        inp = f_body + C @ np.array([0.0, 0.0, g]) + np.array([x[4], x[5], bz_body])
        nl = cbl.T @ inp
        x = x.copy()
        x[0] += nl[0] * dt
        x[1] += nl[1] * dt
        xc, yc, zc = inp
        dC = np.array([[sp * xc - sr * cp * yc - cr * cp * zc, -cr * sp * yc + sr * sp * zc],
                       [0.0, sr * yc + cr * zc]])
        F = np.zeros((6, 6))
        F[0:2, 2:4] = dC
        F[0:2, 4:6] = cbl[0:2, 0:2].T
        G = np.zeros((6, 6))
        G[0:2, 0:2] = cbl[0:2, 0:2].T
        G[2:4, 2:4] = np.eye(2)
        G[4:6, 4:6] = np.eye(2)
        P = P + (F @ P.T + P @ F.T + G @ self.xy_Q @ G.T) * dt
        return x, P


# --------------------------------------------------------------------------
# Step-wise verification of a reference run
# --------------------------------------------------------------------------
RTOL, ATOL = 1e-12, 1e-14   # fixed before measurement; see BASELINE_DECISION.md section 8


def to_sec(t_ns):
    """ROS 1 Time::toSec(): (double)sec + 1e-9 * (double)nsec."""
    return float(t_ns // 1_000_000_000) + 1e-9 * float(t_ns % 1_000_000_000)


def row_state(r):
    return (np.array([r['z'], r['zdot'], r['zbias']]),
            np.array([[r[f'zP{i}{j}'] for j in range(3)] for i in range(3)]),
            np.array([r[k] for k in ('vx', 'vy', 'pitch_bias', 'roll_bias', 'ax_bias', 'ay_bias')]),
            np.array([[r[f'xyP{i}{j}'] for j in range(6)] for i in range(6)]))


class Stats:
    def __init__(self):
        self.worst = 0.0          # max |pred - ref| / (ATOL + RTOL |ref|); <= 1 passes
        self.worst_where = ''
        self.steps = 0
        self.gate_checks = 0
        self.mismatches = []      # discrete disagreements

    def compare(self, name, pred, ref, where):
        pred, ref = np.asarray(pred, float), np.asarray(ref, float)
        both_nan = np.isnan(pred) & np.isnan(ref)
        same_inf = np.isinf(pred) & np.isinf(ref) & (np.sign(pred) == np.sign(ref))
        ok_special = both_nan | same_inf
        with np.errstate(invalid='ignore'):
            ratio = np.where(ok_special, 0.0, np.abs(pred - ref) / (ATOL + RTOL * np.abs(ref)))
        ratio = np.where(np.isnan(ratio), np.inf, ratio)
        m = float(np.max(ratio)) if ratio.size else 0.0
        if m > self.worst:
            self.worst, self.worst_where = m, f'{where}: {name}'


def verify_run(variant, params, events, rows):
    """Return Stats for one reference run (events and rows aligned 1:1)."""
    mdl = Model(variant, params)
    st = Stats()
    acc_n, acc_sum, g = 0, np.zeros(3), None
    last_t = None
    switch_on = False
    ch = int(params.get('mocap_override_channel', 4))
    enable = {k: params.get(k, True) for k in ('enable_mocap_xy', 'enable_mocap_z', 'enable_rgbd', 'enable_sonar')}
    # Event 0 has no predecessor row; it can only be an IMU sample (initialization).
    if events and events[0][0] == 'imu' and rows[0]['delivered']:
        acc_n, acc_sum, last_t = 1, np.array(events[0][2:5]), events[0][1]
    for k in range(1, len(rows)):
        p, c, ev = rows[k - 1], rows[k], events[k]
        where = f'row {k} ({ev[0]} t={ev[1]})'
        xz, Pz, xxy, Pxy = row_state(p)
        zR, xyR = p['zR'], np.diag([p['xyR00'], p['xyR11']])
        z_flag, xy_flag = bool(p['z_flag']), bool(p['xy_flag'])
        z_meas, xy_meas = p['z_meas'], np.array([p['xy_meas0'], p['xy_meas1']])
        use_xy, use_z = bool(p['use_mocap_xy']), bool(p['use_mocap_z'])
        typ = ev[0]
        if not c['delivered']:
            typ = 'undelivered'

        if typ == 'imu':
            f = np.array(ev[2:5]); q = ev[5:9]
            if acc_n < 20:
                acc_n += 1; acc_sum += f; last_t = ev[1]
                if acc_n == 20:
                    mean = acc_sum / 20.0
                    g = 9.81 if variant == 'master' else math.sqrt(mean @ mean)
            else:
                dt = to_sec(ev[1]) - to_sec(last_t)
                last_t = ev[1]
                C = quat_to_c_ned_to_body(*q)
                xxy, Pxy = mdl.xy_propagate(xxy, Pxy, C, f, g, xz[2], dt)
                F, B, G, Q = mdl.z_matrices(dt)
                u = (C.T @ f)[2] + g
                xz = F @ xz + (B * u).ravel()
                Pz = F @ Pz @ F.T + G @ Q @ G.T
                n_prop = int(p['n_prop']) + 1
                if not p['takeoff'] and n_prop >= 10:         # landing reset
                    xxy, Pxy = mdl.xy_x0.copy(), mdl.xy_P0.copy()
                    Pz = mdl.z_P0.copy(); xz[1] = mdl.z_x0[1]
                    n_prop = 0
                if xy_flag:
                    if mdl.partial:
                        xxy, Pxy = partial_update(xxy, Pxy, mdl.H_xy, xy_meas, xyR, mdl.xy_beta)
                        # master clears the flag only after a FULL update (xyz_estimator.cpp
                        # e4179f48 lines 214-222); the simulation branch always clears it.
                        xy_flag = variant == 'master'
                    else:
                        xxy, Pxy = full_update(xxy, Pxy, mdl.H_xy, xy_meas, xyR)
                        xy_flag = False
                if z_flag:
                    zRm = np.array([[zR]])
                    if mdl.partial:
                        xz, Pz = partial_update(xz, Pz, mdl.H_z, np.array([z_meas]), zRm, mdl.z_beta)
                    else:
                        xz, Pz = full_update(xz, Pz, mdl.H_z, np.array([z_meas]), zRm)
                    z_flag = False
                if c['takeoff'] != p['takeoff']:            # setTakeoffState (after updates)
                    if c['takeoff']:
                        zR = mdl.z_R_fly[0, 0]; Pz = mdl.z_P0_fly.copy()
                    else:
                        zR = mdl.z_R0[0, 0]; Pz = mdl.z_P0.copy(); xz = mdl.z_x0.copy()
                st.steps += 1
                st.compare('u', u, c['u'], where)
                st.compare('z_dt', dt if variant == 'master' else dt, c['z_dt'], where)
                if n_prop != int(c['n_prop']):
                    st.mismatches.append(f'{where}: n_prop {n_prop} != {c["n_prop"]}')
        elif typ == 'range':
            r32 = F32(ev[2]); mx32 = F32(ev[3])
            accept = False
            if not use_z and r32 <= mx32:
                accept = True
                if mdl.gated:
                    meas = -float(F32(r32))          # chi2Accept(float): range(0) = -range_measurement
                    d2 = (meas - xz[0]) ** 2 / (Pz[0, 0] + zR)
                    accept = not (d2 > mdl.gate['range'])
                st.gate_checks += 1
            if accept:
                z_meas, z_flag = -float(r32), True
        elif typ == 'mocap_pose':
            if use_z:
                accept = True
                if mdl.gated:
                    d2 = (float(F32(ev[2])) - xz[0]) ** 2 / (Pz[0, 0] + zR)
                    accept = not (d2 > mdl.gate['mocap_z'])
                st.gate_checks += 1
                if accept:
                    z_meas, z_flag = ev[2], True
        elif typ in ('mocap_twist', 'rgbd'):
            is_mocap = typ == 'mocap_twist'
            active = use_xy if is_mocap else (not use_xy and params.get('enable_measurements', True))
            if active:
                zv = np.array(ev[2:4])
                accept = True
                if mdl.gated:
                    S = mdl.H_xy @ Pxy @ mdl.H_xy.T + xyR          # uses the current (previous) R
                    inn = zv - mdl.H_xy @ xxy
                    d2 = float(inn @ np.linalg.inv(S) @ inn)
                    accept = not (d2 > mdl.gate['mocap_xy' if is_mocap else 'rgbd'])
                st.gate_checks += 1
                if accept:
                    xy_meas, xy_flag = zv, True
                    xyR = np.diag([ev[4], ev[5]])
        elif typ == 'rc':
            v = ev[2 + ch]
            if not switch_on and v > 1500:
                use_xy = use_xy or enable['enable_mocap_xy']
                use_z = use_z or enable['enable_mocap_z']
                switch_on = True
            elif switch_on and v <= 1500:
                if enable['enable_rgbd']:
                    use_xy = False
                if enable['enable_sonar']:
                    use_z = False
                switch_on = False

        # Compare everything predicted with the reference row.
        rz, rPz, rxy, rPxy = row_state(c)
        st.compare('z state', xz, rz, where)
        st.compare('z P', Pz, rPz, where)
        st.compare('xy state', xxy, rxy, where)
        st.compare('xy P', Pxy, rPxy, where)
        st.compare('z R', zR, c['zR'], where)
        st.compare('z meas', z_meas, c['z_meas'], where)
        st.compare('xy meas', xy_meas, [c['xy_meas0'], c['xy_meas1']], where)
        st.compare('xy R', np.diag(xyR), [c['xyR00'], c['xyR11']], where)
        for name, pred, ref in (('z_flag', z_flag, c['z_flag']), ('xy_flag', xy_flag, c['xy_flag']),
                                ('use_mocap_xy', use_xy, c['use_mocap_xy']),
                                ('use_mocap_z', use_z, c['use_mocap_z'])):
            if bool(pred) != bool(ref):
                st.mismatches.append(f'{where}: {name} predicted {bool(pred)}, reference {bool(ref)}')
    return st
