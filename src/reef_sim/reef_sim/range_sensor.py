"""IDEALIZED downward range sensor, derived from simulation truth.

This is not a Gazebo sensor and has no beam width, multipath, or latency. For
each truth odometry sample (decimated to rate_hz in sim time) it intersects
the sensor's beam with flat ground at world z = 0:

    origin = p + R * sensor_offset         (world, ENU)
    beam   = R * (0, 0, -1)                (body -Z, i.e. straight down when level)
    range  = (0 - origin.z) / beam.z       (exact slant range, so tilt is included)

Output follows REP 117 for sensor_msgs/Range: -inf if the true range is below
min_range, +inf if it is above max_range or the beam points at or above the
horizon. In-range readings get Gaussian noise (noise_std) drawn from a
generator keyed by (seed, stamp), so the noise of each sample is reproducible,
and are clamped to [min_range, max_range]. The stamp is the truth sample's sim time.

Subscribes: /x3/truth/odom (nav_msgs/Odometry)   [truth input]
Publishes:  /x3/range      (sensor_msgs/Range)   [idealized measurement]
"""
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range

from reef_sim.geometry import downward_ray_range, quat_to_matrix


class IdealRangeSensor(Node):

    def __init__(self):
        super().__init__('range_sensor')
        self.rate_hz = self.declare_parameter('rate_hz', 20.0).value
        self.offset = np.array(self.declare_parameter('sensor_offset', [0.0, 0.0, -0.055]).value, float)
        self.min_range = self.declare_parameter('min_range', 0.20).value
        self.max_range = self.declare_parameter('max_range', 7.65).value
        self.noise_std = self.declare_parameter('noise_std', 0.01).value
        self.seed = int(self.declare_parameter('seed', 42).value)
        self.frame_id = self.declare_parameter('frame_id', 'x3/range_link').value
        self.period_ns = int(1e9 / self.rate_hz)
        self.next_ns = None
        self.pub = self.create_publisher(Range, '/x3/range', 10)
        self.create_subscription(Odometry, '/x3/truth/odom', self.on_truth, 50)
        self.get_logger().info(
            f'idealized range: offset {self.offset.tolist()} m, limits [{self.min_range}, '
            f'{self.max_range}] m, noise {self.noise_std} m, seed {self.seed}, {self.rate_hz} Hz')

    def on_truth(self, odom):
        t_ns = odom.header.stamp.sec * 1_000_000_000 + odom.header.stamp.nanosec
        if self.next_ns is None:
            self.next_ns = t_ns
        if t_ns < self.next_ns:
            return
        # Keep a fixed sim-time grid even if a sample is late.
        while self.next_ns <= t_ns:
            self.next_ns += self.period_ns
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        true_range = downward_ray_range(
            (p.x, p.y, p.z), quat_to_matrix(q.x, q.y, q.z, q.w), self.offset)
        if true_range < self.min_range:
            value = -math.inf
        elif true_range > self.max_range:
            value = math.inf
        else:
            rng = np.random.default_rng([self.seed, t_ns])
            noisy = true_range + (rng.normal(0.0, self.noise_std) if self.noise_std > 0 else 0.0)
            value = min(max(noisy, self.min_range), self.max_range)
        msg = Range()
        msg.header.stamp = odom.header.stamp
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND   # nominal: stands in for REEF's sonar
        msg.field_of_view = 0.0                 # idealized single ray
        msg.min_range = float(self.min_range)
        msg.max_range = float(self.max_range)
        msg.range = float(value)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = IdealRangeSensor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
