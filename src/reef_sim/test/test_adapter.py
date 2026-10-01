"""REEF adapter conversions (reef_sim/reef_adapter.py)."""
import math
import random

import numpy as np

from reef_sim.reef_adapter import MAX_EXTRAPOLATION_NS, Adapter, flu_to_frd, ned_frd_from_enu_flu, slerp

T = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
B = np.diag([1.0, -1.0, -1.0])


def R_of(q):
    """Rotation matrix of a (w, x, y, z) quaternion, independent of the adapter code."""
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def random_q(rng):
    q = [rng.gauss(0, 1) for _ in range(4)]
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def test_orientation_matches_T_R_B():
    rng = random.Random(3)
    for _ in range(200):
        q = random_q(rng)
        np.testing.assert_allclose(R_of(ned_frd_from_enu_flu(q)), T @ R_of(q) @ B, atol=1e-12)


def test_level_facing_east_is_yaw_90_in_ned():
    q = ned_frd_from_enu_flu((1.0, 0.0, 0.0, 0.0))   # FLU aligned with ENU: x forward = east
    R = R_of(q)
    np.testing.assert_allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-12)   # forward = east (NED y)
    np.testing.assert_allclose(R @ [0, 0, 1], [0, 0, 1], atol=1e-12)   # body down = world down


def test_specific_force_at_rest_becomes_minus_g_frd():
    assert flu_to_frd((0.0, 0.0, 9.81)) == (0.0, 0.0, -9.81)


def test_slerp_endpoints_and_midpoint():
    a, b = (1.0, 0.0, 0.0, 0.0), (math.cos(0.5), 0.0, 0.0, math.sin(0.5))   # 0 and 1 rad about z
    np.testing.assert_allclose(slerp(a, b, 0.0), a, atol=1e-12)
    np.testing.assert_allclose(slerp(a, b, 1.0), b, atol=1e-12)
    np.testing.assert_allclose(slerp(a, b, 0.5), (math.cos(0.25), 0, 0, math.sin(0.25)), atol=1e-12)


def test_imu_is_converted_immediately_with_interpolation_or_extrapolation():
    ad = Adapter()
    assert ad.on_imu(5, 'early') == [] and ad.dropped_early == 1        # no truth yet
    ad.on_truth(10, (1.0, 0.0, 0.0, 0.0))
    assert ad.on_imu(12, 'one') == [] and ad.dropped_early == 2         # one truth sample is not enough
    yaw = 0.2
    ad.on_truth(20, (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))   # ENU yaw rate 0.02 rad per ns unit
    [(t, p, q_mid)] = ad.on_imu(15, 'inside')                            # between samples: slerp
    [(_, _, q_ext)] = ad.on_imu(30, 'after')                             # 10 after the last: extrapolated
    def yaw_of(q):   # NED yaw of the FRD body
        w, x, y, z = q
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    base = yaw_of(ned_frd_from_enu_flu((1.0, 0.0, 0.0, 0.0)))
    assert abs((base - yaw_of(q_mid)) - 0.1) < 1e-12                    # ENU yaw +0.1 = NED yaw -0.1
    assert abs((base - yaw_of(q_ext)) - 0.4) < 1e-12                    # linear in time: 0.2 at 20, 0.4 at 30
    [(_, _, q_far)] = ad.on_imu(20 + MAX_EXTRAPOLATION_NS + 1, 'far')  # too far: last sample
    assert abs((base - yaw_of(q_far)) - 0.2) < 1e-12

def test_body_level_velocity_matches_independent_matrices():
    from reef_sim.reef_adapter import body_level_velocity
    rng = random.Random(5)
    for _ in range(200):
        q = random_q(rng)
        v = (rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-1, 1))
        M = T @ R_of(q) @ B                       # R_NED<-FRD
        psi = math.atan2(M[1, 0], M[0, 0])
        Rz = np.array([[math.cos(psi), -math.sin(psi), 0], [math.sin(psi), math.cos(psi), 0], [0, 0, 1]])
        expected = Rz.T @ (T @ (R_of(q) @ v))     # body FLU -> ENU -> NED -> body-level
        np.testing.assert_allclose(body_level_velocity(q, v), expected[:2], atol=1e-12)


def test_body_level_velocity_known_cases():
    from reef_sim.reef_adapter import body_level_velocity
    east = (1.0, 0.0, 0.0, 0.0)                    # FLU aligned with ENU: facing east, yaw 90 deg in NED
    np.testing.assert_allclose(body_level_velocity(east, (1.0, 0.0, 0.0)), (1.0, 0.0), atol=1e-12)   # forward
    np.testing.assert_allclose(body_level_velocity(east, (0.0, 1.0, 0.0)), (0.0, -1.0), atol=1e-12)  # FLU left = right -1
    # pitched 20 deg nose up (about body y), flying forward along the body x axis:
    # only the horizontal part of the velocity remains
    th = math.radians(20)
    q = (math.cos(-th / 2), 0.0, math.sin(-th / 2), 0.0)     # ENU: nose up is negative rotation about y
    vx, vy = body_level_velocity(q, (1.0, 0.0, 0.0))
    np.testing.assert_allclose((vx, vy), (math.cos(th), 0.0), atol=1e-12)


def test_velocity_noise_is_reproducible():
    from reef_sim.reef_adapter import velocity_observation
    a = velocity_observation(123456789, (1.0, 0.0, 0.0, 0.0), (0.3, 0.0, 0.0))
    b = velocity_observation(123456789, (1.0, 0.0, 0.0, 0.0), (0.3, 0.0, 0.0))
    c = velocity_observation(123456790, (1.0, 0.0, 0.0, 0.0), (0.3, 0.0, 0.0))
    assert a == b and a != c and a[2] == 0.02 ** 2


def test_mocap_pose_is_ned_with_frd_attitude():
    """P07b idealized mocap: ENU (x east, y north, z up) -> NED (north, east,
    down); a vehicle facing east (ENU yaw 0) has NED heading +pi/2."""
    import math

    from reef_sim.reef_adapter import mocap_pose
    (n, e, d), q = mocap_pose((2.0, 3.0, 1.5), (1.0, 0.0, 0.0, 0.0))
    assert (n, e, d) == (3.0, 2.0, -1.5)
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    assert abs(yaw - math.pi / 2) < 1e-12 and abs(roll) < 1e-12
