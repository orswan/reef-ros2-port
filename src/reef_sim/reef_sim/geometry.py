"""Small rotation and range-geometry helpers (numpy only)."""
import numpy as np


def quat_to_matrix(x, y, z, w):
    """Rotation matrix R (body -> world) from a unit quaternion (x, y, z, w)."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        raise ValueError('zero quaternion')
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def downward_ray_range(position, rotation, sensor_offset, ground_z=0.0):
    """Distance along the body -Z axis from the sensor to the plane z = ground_z.

    position:      body origin in world (ENU), shape (3,)
    rotation:      body -> world rotation matrix, shape (3, 3)
    sensor_offset: sensor origin in the body frame (FLU), shape (3,)

    Returns +inf if the beam does not point toward the ground (no return).
    """
    origin = np.asarray(position) + rotation @ np.asarray(sensor_offset)
    beam = rotation @ np.array([0.0, 0.0, -1.0])
    if beam[2] >= -1e-9:
        return float('inf')
    return float((ground_z - origin[2]) / beam[2])
