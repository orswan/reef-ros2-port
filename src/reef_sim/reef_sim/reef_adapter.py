"""Adapter from the X3 simulation topics to the REEF estimator inputs (P04).

    /x3/imu (FLU specific force, no orientation)  +  /x3/truth/odom (TRUTH)
        -> /x3/reef/imu/data   sensor_msgs/Imu, body FRD, orientation of FRD in NED
    /x3/range (idealized, derived from truth)
        -> /x3/reef/sonar      sensor_msgs/Range, unchanged
    /x3/reef/input_labels      std_msgs/String (transient local): what is idealized

IDEALIZED INPUTS: the orientation in /x3/reef/imu/data is the truth attitude
(there is no attitude estimator in this project), interpolated (slerp) to
each IMU stamp; the range is the idealized range of X3_SCENARIO.md section 4.
Specific force and angular rate are the measured /x3/imu values, converted
FLU -> FRD (x, -y, -z). Ranges are passed through unchanged, including REP 117
+/-inf, so the estimator's own rejection rules apply (master rejects -inf
through its chi-square gate, BASELINE_DECISION.md section 4.8). No tilt
compensation is applied (as in master; correction C5 is deferred).

An IMU message is held until a truth sample at or after its stamp has
arrived, so every IMU gets an interpolated attitude; messages are emitted in
arrival order. IMU messages older than the first truth sample are dropped.

Conversions (X3_SCENARIO.md section 5): R_NED<-FRD = T R_ENU<-FLU B with
T = [[0,1,0],[1,0,0],[0,0,-1]] and B = diag(1,-1,-1); as quaternions
q_NED<-FRD = q_T * q_ENU<-FLU * q_B.
"""
import math
from collections import deque

LABEL = ('IDEALIZED INPUTS: imu/data orientation = TRUTH attitude from /x3/truth/odom '
         '(slerp to the IMU stamp); sonar = idealized range derived from truth (/x3/range). '
         'Specific force and angular rate are the simulated IMU measurement (FLU -> FRD).')

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


class TruthInterpolator:
    """Truth attitude samples (t_ns, q_ENU<-FLU as w, x, y, z), interpolated on request."""

    def __init__(self, keep=400):
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
        """q_NED<-FRD at t_ns, or None if t_ns is outside the samples held."""
        if not self.samples or t_ns < self.samples[0][0] or t_ns > self.samples[-1][0]:
            return None
        prev = self.samples[0]
        for cur in self.samples:
            if cur[0] >= t_ns:
                if cur[0] == t_ns or cur is prev:
                    return ned_frd_from_enu_flu(cur[1])
                s = (t_ns - prev[0]) / (cur[0] - prev[0])
                return ned_frd_from_enu_flu(slerp(prev[1], cur[1], s))
            prev = cur
        return None


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
        first = self.truth.earliest()
        if first is None or t_ns < first:
            self.dropped_early += 1
            return []
        self.pending.append((t_ns, payload))
        return self._flush()

    def _flush(self):
        out = []
        latest = self.truth.latest()
        while self.pending and latest is not None and self.pending[0][0] <= latest:
            t_ns, payload = self.pending.popleft()
            q = self.truth.at(t_ns)
            if q is None:   # fell out of the held window (should not happen)
                self.dropped_early += 1
                continue
            out.append((t_ns, payload, q))
        return out


def main(args=None):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, Range
    from std_msgs.msg import String

    class ReefAdapter(Node):
        def __init__(self):
            super().__init__('reef_adapter')
            self.adapter = Adapter()
            self.imu_pub = self.create_publisher(Imu, '/x3/reef/imu/data', 50)
            self.range_pub = self.create_publisher(Range, '/x3/reef/sonar', 10)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.label_pub = self.create_publisher(String, '/x3/reef/input_labels', latched)
            self.label_pub.publish(String(data=LABEL))
            self.create_subscription(Imu, '/x3/imu', self.on_imu, 50)
            self.create_subscription(Odometry, '/x3/truth/odom', self.on_truth, 50)
            self.create_subscription(Range, '/x3/range', self.range_pub.publish, 10)
            self.get_logger().warn(LABEL)

        @staticmethod
        def ns(stamp):
            return stamp.sec * 1_000_000_000 + stamp.nanosec

        def on_truth(self, m):
            o = m.pose.pose.orientation
            self.emit(self.adapter.on_truth(self.ns(m.header.stamp), (o.w, o.x, o.y, o.z)))

        def on_imu(self, m):
            self.emit(self.adapter.on_imu(self.ns(m.header.stamp), m))

        def emit(self, items):
            for _, m, q in items:
                out = Imu()
                out.header.stamp = m.header.stamp
                out.header.frame_id = 'x3/base_link_frd'
                a, w = m.linear_acceleration, m.angular_velocity
                out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = \
                    flu_to_frd((a.x, a.y, a.z))
                out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = \
                    flu_to_frd((w.x, w.y, w.z))
                out.linear_acceleration_covariance = m.linear_acceleration_covariance
                out.angular_velocity_covariance = m.angular_velocity_covariance
                out.orientation.w, out.orientation.x, out.orientation.y, out.orientation.z = q
                out.orientation_covariance[0] = 0.0   # idealized: truth (see input_labels)
                self.imu_pub.publish(out)

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
