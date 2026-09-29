"""Add reproducible Gaussian noise to Gazebo's noise-free IMU.

Gazebo's own sensor noise draws from gz-math's global random generator, which
other systems (for example OdometryPublisher) use concurrently in PostUpdate.
Its noise sequence therefore differs from run to run even with `gz sim --seed`.
Here each sample's noise comes from a generator keyed by (seed, stamp), so
the noise of every sample is reproducible and independent of message drops.

Subscribes: /x3/sim/imu_noise_free (sensor_msgs/Imu)  [simulator-internal]
Publishes:  /x3/imu                (sensor_msgs/Imu)  [measurement]

The output keeps the input stamp and frame (x3/base_link, FLU). It fills the
diagonal covariances from the configured noise, and marks orientation as not
provided (orientation_covariance[0] = -1, REP 145; quaternion all zeros).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuNoise(Node):

    def __init__(self):
        super().__init__('imu_noise')
        self.acc_std = float(self.declare_parameter('accel_noise_std', 0.02).value)
        self.gyr_std = float(self.declare_parameter('gyro_noise_std', 0.002).value)
        self.seed = int(self.declare_parameter('seed', 7).value)
        self.pub = self.create_publisher(Imu, '/x3/imu', 50)
        self.create_subscription(Imu, '/x3/sim/imu_noise_free', self.on_imu, 50)
        self.get_logger().info(
            f'IMU noise: accel {self.acc_std} m/s^2, gyro {self.gyr_std} rad/s, seed {self.seed}')

    def on_imu(self, msg):
        t_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        n = np.random.default_rng([self.seed, t_ns]).normal(0.0, 1.0, 6)
        out = Imu()
        out.header = msg.header
        a, w = msg.linear_acceleration, msg.angular_velocity
        out.linear_acceleration.x = a.x + self.acc_std * n[0]
        out.linear_acceleration.y = a.y + self.acc_std * n[1]
        out.linear_acceleration.z = a.z + self.acc_std * n[2]
        out.angular_velocity.x = w.x + self.gyr_std * n[3]
        out.angular_velocity.y = w.y + self.gyr_std * n[4]
        out.angular_velocity.z = w.z + self.gyr_std * n[5]
        for i in (0, 4, 8):
            out.linear_acceleration_covariance[i] = self.acc_std ** 2
            out.angular_velocity_covariance[i] = self.gyr_std ** 2
        # Orientation not provided (REP 145). The quaternion is zeroed so that it
        # cannot be mistaken for an identity (level) attitude.
        out.orientation.w = 0.0
        out.orientation_covariance[0] = -1.0
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ImuNoise()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
