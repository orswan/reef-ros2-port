"""Unit tests for reef_sim.geometry with hand-derived expected values."""
import math
import unittest

import numpy as np

from reef_sim.geometry import downward_ray_range, quat_to_matrix

OFFSET = (0.0, 0.0, -0.055)   # sensor 5.5 cm below the body origin (FLU), as configured


def quat_axis_angle(axis, angle):
    ax = np.asarray(axis, float) / np.linalg.norm(axis)
    s = math.sin(angle / 2)
    return (ax[0] * s, ax[1] * s, ax[2] * s, math.cos(angle / 2))


class RotationTests(unittest.TestCase):

    def test_yaw_90_maps_x_to_y(self):
        R = quat_to_matrix(*quat_axis_angle((0, 0, 1), math.pi / 2))
        np.testing.assert_allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-12)

    def test_orthonormal_for_arbitrary_unnormalized_quaternion(self):
        R = quat_to_matrix(0.3, -1.2, 0.5, 2.0)   # deliberately not unit length
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
        self.assertAlmostEqual(np.linalg.det(R), 1.0, places=12)

    def test_zero_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            quat_to_matrix(0, 0, 0, 0)


class RangeTests(unittest.TestCase):

    def test_level_is_height_of_sensor(self):
        # body origin 1.0 m up, level: sensor at 0.945 m, beam straight down
        r = downward_ray_range((0, 0, 1.0), np.eye(3), OFFSET)
        self.assertAlmostEqual(r, 0.945, places=12)

    def test_pitch_30_is_slant_range_not_vertical(self):
        # sensor height = 1 - 0.055 cos(t); slant = height / cos(t) = 1/cos(t) - 0.055
        t = math.radians(30)
        R = quat_to_matrix(*quat_axis_angle((0, 1, 0), t))
        r = downward_ray_range((0, 0, 1.0), R, OFFSET)
        self.assertAlmostEqual(r, 1 / math.cos(t) - 0.055, places=12)
        self.assertAlmostEqual(r, 1.0997005383792515, places=12)

    def test_horizontal_position_does_not_matter_on_flat_ground(self):
        a = downward_ray_range((0, 0, 2.0), np.eye(3), OFFSET)
        b = downward_ray_range((123.0, -45.0, 2.0), np.eye(3), OFFSET)
        self.assertEqual(a, b)

    def test_beam_at_or_above_horizon_has_no_return(self):
        for angle in (math.pi / 2, math.pi):   # rolled 90 deg (horizontal) and upside down
            R = quat_to_matrix(*quat_axis_angle((1, 0, 0), angle))
            self.assertEqual(downward_ray_range((0, 0, 1.0), R, OFFSET), math.inf)


if __name__ == '__main__':
    unittest.main()
