"""REEF adapter conversions (reef_sim/reef_adapter.py)."""
import math
import random

import numpy as np

from reef_sim.reef_adapter import Adapter, flu_to_frd, ned_frd_from_enu_flu, slerp

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


def test_imu_waits_for_truth_and_keeps_order():
    ad = Adapter()
    assert ad.on_imu(5, 'early') == [] and ad.dropped_early == 1   # before any truth
    assert ad.on_truth(10, (1.0, 0.0, 0.0, 0.0)) == []
    assert ad.on_imu(12, 'a') == [] and ad.on_imu(14, 'b') == []   # held: no truth after them yet
    out = ad.on_truth(20, (1.0, 0.0, 0.0, 0.0))
    assert [p for _, p, _ in out] == ['a', 'b']
    out = ad.on_imu(20, 'c')                                        # exactly at a truth stamp
    assert [p for _, p, _ in out] == ['c']
