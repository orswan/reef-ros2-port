"""Adapter from the X3 simulation topics to the REEF estimator inputs (P04/P05).

Live nodes: x3_imu_adapter (C++, reef_x3_adapter: the IMU path below) and
this reef_adapter (velocity observations, labels). The offline replay uses
the Python functions of this module for both.

    /x3/imu (FLU specific force, no orientation)  +  /x3/truth/odom (TRUTH)
        -> /x3/reef/imu/data   sensor_msgs/Imu, body FRD, orientation of FRD in NED
    /x3/range (idealized, derived from truth) is read by the estimator directly
        (remapped; unchanged, since P05; earlier /x3/reef/sonar)
    /x3/truth/odom (TRUTH)
        -> /x3/reef/mocap_velocity/body_level_frame   geometry_msgs/TwistWithCovarianceStamped:
           IDEALIZED simulated velocity observation (P05): the truth velocity in
           REEF's body-level frame plus white noise (velocity_noise_std, keyed by
           (velocity_seed, stamp)), one per truth sample (100 Hz), covariance[0]
           and [7] = noise variance. Not RGB-D odometry.
    /x3/reef/input_labels      std_msgs/String (transient local): what is idealized

IDEALIZED INPUTS: the orientation in /x3/reef/imu/data is the truth attitude
(there is no attitude estimator in this project), interpolated (slerp) to
each IMU stamp; the range is the idealized range of X3_SCENARIO.md section 4.
Specific force and angular rate are the measured /x3/imu values, converted
FLU -> FRD (x, -y, -z). Ranges are passed through unchanged, including REP 117
+/-inf, so the estimator's own rejection rules apply (master rejects -inf
through its chi-square gate, BASELINE_DECISION.md section 4.8). No tilt
compensation is applied (as in master; correction C5 is deferred).

Each IMU message is converted immediately (no waiting, P05): its attitude is
the truth attitude slerped between the two truth samples around its stamp
when both have arrived, otherwise extrapolated (slerp beyond the last
sample) from the last two truth samples by at most MAX_EXTRAPOLATION_NS,
otherwise the latest truth sample. IMU messages before the second truth
sample are dropped. (P04 held each IMU until the next truth sample, which
added up to 10 ms of latency.)

Conversions (X3_SCENARIO.md section 5): R_NED<-FRD = T R_ENU<-FLU B with
T = [[0,1,0],[1,0,0],[0,0,-1]] and B = diag(1,-1,-1); as quaternions
q_NED<-FRD = q_T * q_ENU<-FLU * q_B.
"""
import math
from collections import deque

LABEL = ('IDEALIZED INPUTS: imu/data orientation = TRUTH attitude from /x3/truth/odom '
         '(slerp to the IMU stamp); sonar = idealized range derived from truth (/x3/range); '
         'mocap_velocity/body_level_frame = TRUTH velocity in the body-level frame plus white noise '
         '(simulated velocity observation, not RGB-D odometry). '
         'Specific force and angular rate are the simulated IMU measurement (FLU -> FRD).')

VELOCITY_NOISE_STD = 0.02   # m/s per axis (ACCEPTANCE.md 4d)
VELOCITY_SEED = 11

S = math.sqrt(0.5)
Q_T = (0.0, S, S, 0.0)      # (w, x, y, z): ENU -> NED, 180 deg about (1, 1, 0)/sqrt(2)
Q_B = (0.0, 1.0, 0.0, 0.0)  # FRD -> FLU, 180 deg about x


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def ned_frd_from_enu_flu(q_wxyz):
    """Orientation of the FRD body in NED from the orientation of FLU in ENU."""
    q = qmul(qmul(Q_T, q_wxyz), Q_B)
    if q[0] < 0:   # canonical sign (w >= 0); the rotation is the same
        q = tuple(-c for c in q)
    return q


def slerp(a, b, s):
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0:
        b, dot = tuple(-c for c in b), -dot
    if dot > 0.9995:
        q = tuple(x + s * (y - x) for x, y in zip(a, b))
    else:
        th = math.acos(dot)
        sa, sb = math.sin((1 - s) * th) / math.sin(th), math.sin(s * th) / math.sin(th)
        q = tuple(sa * x + sb * y for x, y in zip(a, b))
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def flu_to_frd(v):
    return (v[0], -v[1], -v[2])


def rotate(q_wxyz, v):
    """R(q) v for a unit quaternion (w, x, y, z)."""
    w, x, y, z = q_wxyz
    r = ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
         (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
         (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))
    return tuple(sum(r[i][k] * v[k] for k in range(3)) for i in range(3))


def body_level_velocity(q_enu_flu, v_flu):
    """Truth velocity in REEF's body-level frame (NED rotated by yaw).

    v_ENU = R_ENU<-FLU v_FLU (odometry twist is in the body FLU frame);
    v_NED = (v_ENU.y, v_ENU.x, -v_ENU.z); yaw psi of the FRD body in NED;
    v_level = R_z(psi)^T v_NED. Returns (vx, vy) forward/right, horizontal.
    """
    ve = rotate(q_enu_flu, v_flu)
    vn = (ve[1], ve[0], -ve[2])
    w, x, y, z = ned_frd_from_enu_flu(q_enu_flu)
    psi = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    c, s = math.cos(psi), math.sin(psi)
    return (c * vn[0] + s * vn[1], -s * vn[0] + c * vn[1])


def velocity_noise(t_ns, seed=VELOCITY_SEED, std=VELOCITY_NOISE_STD):
    import numpy as np
    return tuple(np.random.default_rng([seed, t_ns]).normal(0.0, std, 2))


def velocity_observation(t_ns, q_enu_flu, v_flu, std=VELOCITY_NOISE_STD, seed=VELOCITY_SEED):
    """(vx, vy, variance) of the simulated observation at t_ns."""
    vx, vy = body_level_velocity(q_enu_flu, v_flu)
    nx, ny = velocity_noise(t_ns, seed, std)
    return vx + nx, vy + ny, std * std


MAX_EXTRAPOLATION_NS = 20_000_000


def mocap_pose(p_enu, q_enu_flu_wxyz):
    """IDEALIZED mocap pose (P07b position mode): truth position in NED and the
    truth attitude of FRD in NED, as a motion-capture system would report it
    in REEF's convention (x north, y east, z down)."""
    x, y, z = p_enu
    return (y, x, -z), ned_frd_from_enu_flu(q_enu_flu_wxyz)


MOCAP_LABEL = ('IDEALIZED mocap pose on /x3/reef/pose_stamped: TRUTH position (NED) and TRUTH attitude '
               '(FRD in NED) from /x3/truth/odom, no noise or latency (P07b position mode).')


class TruthInterpolator:
    """Truth attitude samples (t_ns, q_ENU<-FLU as w, x, y, z), interpolated on request."""

    def __init__(self, keep=16):   # 160 ms of truth at 100 Hz is plenty for IMU interpolation
        self.samples = deque(maxlen=keep)

    def add(self, t_ns, q_wxyz):
        if self.samples and t_ns <= self.samples[-1][0]:
            return
        self.samples.append((t_ns, q_wxyz))

    def latest(self):
        return self.samples[-1][0] if self.samples else None

    def earliest(self):
        return self.samples[0][0] if self.samples else None

    def at(self, t_ns):
        """q_NED<-FRD at t_ns, or None before the second truth sample.

        Inside the held samples: slerp between the neighbours. After the last
        sample: extrapolated from the last two (at most MAX_EXTRAPOLATION_NS
        beyond the last sample, else the last sample itself)."""
        if len(self.samples) < 2 or t_ns < self.samples[0][0]:
            return None
        if t_ns > self.samples[-1][0]:
            (t0, q0), (t1, q1) = self.samples[-2], self.samples[-1]
            if t_ns - t1 > MAX_EXTRAPOLATION_NS:
                return ned_frd_from_enu_flu(q1)
            return ned_frd_from_enu_flu(slerp(q0, q1, (t_ns - t0) / (t1 - t0)))
        # search from the newest sample (IMU stamps are near the latest truth)
        for k in range(len(self.samples) - 1, 0, -1):
            prev, cur = self.samples[k - 1], self.samples[k]
            if prev[0] <= t_ns <= cur[0]:
                if t_ns == cur[0]:
                    return ned_frd_from_enu_flu(cur[1])
                s = (t_ns - prev[0]) / (cur[0] - prev[0])
                return ned_frd_from_enu_flu(slerp(prev[1], cur[1], s))
        return ned_frd_from_enu_flu(self.samples[0][1])   # t_ns == oldest stamp


class Adapter:
    """Transport-independent adapter state; used by the node and the offline converter."""

    def __init__(self):
        self.truth = TruthInterpolator()
        self.pending = deque()   # (t_ns, payload)
        self.dropped_early = 0

    def on_truth(self, t_ns, q_wxyz):
        """Returns the IMU payloads that can now be emitted, with their attitude."""
        self.truth.add(t_ns, q_wxyz)
        return self._flush()

    def on_imu(self, t_ns, payload):
        q = self.truth.at(t_ns)
        if q is None:
            self.dropped_early += 1
            return []
        return [(t_ns, payload, q)]

    def _flush(self):
        return []   # nothing is held (kept so on_truth has one return type)


def main(args=None):
    """Live node: simulated velocity observations and the input labels.

    Since P05 the latency-critical IMU conversion runs in C++
    (reef_x3_adapter/x3_imu_adapter, same algorithm as Adapter above, which the
    offline replay still uses), and the estimator reads /x3/range directly.
    """
    import rclpy
    from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    class ReefAdapter(Node):
        def __init__(self):
            super().__init__('reef_adapter')
            self.vel_std = float(self.declare_parameter('velocity_noise_std', VELOCITY_NOISE_STD).value)
            self.vel_seed = int(self.declare_parameter('velocity_seed', VELOCITY_SEED).value)
            self.vel_pub = self.create_publisher(
                TwistWithCovarianceStamped, '/x3/reef/mocap_velocity/body_level_frame', 50)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.label_pub = self.create_publisher(String, '/x3/reef/input_labels', latched)
            self.label_pub.publish(String(data=LABEL))
            self.create_subscription(Odometry, '/x3/truth/odom', self.on_truth, 50)
            self.get_logger().warn(LABEL)
            # P07b: idealized mocap pose (position mode), off by default.
            self.mocap_pub = None
            if self.declare_parameter('publish_mocap_pose', False).value:
                self.mocap_pub = self.create_publisher(PoseStamped, '/x3/reef/pose_stamped', 50)
                self.label_pub.publish(String(data=LABEL + ' ' + MOCAP_LABEL))
                self.get_logger().warn(MOCAP_LABEL)
            # TEST HOOK (P07b), off by default: "velocity_drop <seconds>" on
            # /x3/test/fault drops the velocity observations stamped within the
            # next <seconds> of sim time.
            self.drop_until_ns = 0
            if self.declare_parameter('test_hooks', False).value:
                self.create_subscription(String, '/x3/test/fault', self.on_fault, 10)
                self.get_logger().warn('TEST HOOKS ENABLED (/x3/test/fault)')

        def on_fault(self, m):
            parts = m.data.split()
            if len(parts) == 2 and parts[0] == 'velocity_drop':
                self.drop_until_ns = self.get_clock().now().nanoseconds + int(float(parts[1]) * 1e9)
                self.get_logger().warn(f'TEST HOOK: dropping velocity observations for {parts[1]} s (sim)')

        def on_truth(self, m):
            o, v = m.pose.pose.orientation, m.twist.twist.linear
            t = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
            if self.mocap_pub is not None:
                pos = m.pose.pose.position
                (nx, ny, nz), (qw, qx, qy, qz) = mocap_pose((pos.x, pos.y, pos.z), (o.w, o.x, o.y, o.z))
                ps = PoseStamped()
                ps.header.stamp = m.header.stamp
                ps.header.frame_id = 'mocap_ned'
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = nx, ny, nz
                ps.pose.orientation.w, ps.pose.orientation.x = qw, qx
                ps.pose.orientation.y, ps.pose.orientation.z = qy, qz
                self.mocap_pub.publish(ps)
            if t < self.drop_until_ns:
                return   # test hook: deliberate loss of the velocity observations
            vx, vy, var = velocity_observation(t, (o.w, o.x, o.y, o.z), (v.x, v.y, v.z),
                                               self.vel_std, self.vel_seed)
            tw = TwistWithCovarianceStamped()
            tw.header.stamp = m.header.stamp
            tw.header.frame_id = 'x3/body_level'
            tw.twist.twist.linear.x, tw.twist.twist.linear.y = vx, vy
            tw.twist.covariance[0] = tw.twist.covariance[7] = var
            self.vel_pub.publish(tw)

    rclpy.init(args=args)
    node = ReefAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
