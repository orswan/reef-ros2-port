"""Bounded X3 flight scenario: publishes velocity commands phase by phase.

All phase timing uses simulation time (use_sim_time). Wall-clock limits apply
only to startup and to detecting a stalled simulation.

Before commanding, it waits (bounded) for /clock and /x3/truth/odom. It
requires exactly one publisher on each required stream (a second one would be
a foreign simulation) and, if record_topics is set, a subscriber (the
recorder) on each of those topics.

Publishes: /x3/cmd_vel (geometry_msgs/Twist), /x3/scenario/phase
(std_msgs/String, transient local; one message per phase change).
Writes result_file (JSON): status, phase boundaries in sim time, final truth.
Exit status: 0 completed, 3 startup timeout or stall, 4 unexpected publishers.
"""
import json
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

REQUIRED_PUBLISHED = ['/clock', '/x3/truth/odom', '/x3/imu']


class ScenarioRunner(Node):

    def __init__(self):
        super().__init__('scenario_runner')
        p = self.declare_parameter
        self.rate_hz = p('cmd_rate_hz', 20.0).value
        self.startup_timeout = p('startup_timeout_s', 90.0).value
        self.stall_timeout = p('stall_timeout_s', 20.0).value
        self.result_file = p('result_file', '').value
        self.require_range = p('require_range', True).value
        self.record_topics = list(p('record_topics', ['']).value)
        # Extra streams that must have exactly one publisher before the flight
        # starts (the REEF adapter and estimator in --estimator runs).
        self.required_extra = [t for t in p('required_extra', ['']).value if t]
        names = list(p('phase_names', ['']).value)
        cols = {k: list(p(f'phase_{k}', [0.0]).value)
                for k in ('durations', 'vx', 'vy', 'vz', 'yaw_rate')}
        if not names or any(len(v) != len(names) for v in cols.values()):
            raise ValueError('phase_* parameters must be non-empty arrays of equal length')
        self.phases = [dict(name=n, duration=float(cols['durations'][i]), vx=float(cols['vx'][i]),
                            vy=float(cols['vy'][i]), vz=float(cols['vz'][i]),
                            yaw_rate=float(cols['yaw_rate'][i])) for i, n in enumerate(names)]
        self.cmd_pub = self.create_publisher(Twist, '/x3/cmd_vel', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.phase_pub = self.create_publisher(String, '/x3/scenario/phase', latched)
        self.truth = None
        self.create_subscription(Odometry, '/x3/truth/odom', self.on_truth, 10)

    def on_truth(self, msg):
        self.truth = msg

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_until(self, pred, wall_limit):
        deadline = time.monotonic() + wall_limit
        while not pred():
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    def startup(self):
        required = list(REQUIRED_PUBLISHED) + (['/x3/range'] if self.require_range else []) \
            + self.required_extra
        if not self.spin_until(lambda: self.now_s() > 0 and self.truth is not None, self.startup_timeout):
            return 3, 'no sim clock or truth odometry within startup timeout'
        ok = self.spin_until(lambda: all(self.count_publishers(t) >= 1 for t in required), 20.0)
        counts = {t: self.count_publishers(t) for t in required}
        if not ok:
            return 3, f'required publishers missing: {counts}'
        if any(c != 1 for c in counts.values()):
            return 4, f'expected exactly one publisher per stream, got {counts}'
        # Only topics that have a publisher can be subscribed by the recorder; a
        # missing stream is left for the bag analysis to report.
        recorded = [t for t in self.record_topics if t and self.count_publishers(t) > 0]
        if recorded and not self.spin_until(
                lambda: all(self.count_subscribers(t) >= 1 for t in recorded), 20.0):
            return 3, 'recorder did not subscribe to ' + str(
                [t for t in recorded if self.count_subscribers(t) < 1])
        return 0, 'ok'

    def publish_cmd(self, ph):
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = ph['vx'], ph['vy'], ph['vz']
        cmd.angular.z = ph['yaw_rate']
        self.cmd_pub.publish(cmd)

    def fly(self):
        period = 1.0 / self.rate_hz
        t0 = self.now_s()
        boundaries, t_start = [], t0
        for ph in self.phases:
            self.phase_pub.publish(String(data=ph['name']))
            t_end = t_start + ph['duration']
            self.get_logger().info(
                f"phase {ph['name']}: {ph['duration']:.1f} s, v=({ph['vx']}, {ph['vy']}, {ph['vz']})")
            next_cmd = t_start
            last_sim, last_wall = self.now_s(), time.monotonic()
            while True:
                now = self.now_s()
                if now >= t_end:
                    break
                if now > last_sim:
                    last_sim, last_wall = now, time.monotonic()
                elif time.monotonic() - last_wall > self.stall_timeout:
                    return 3, f'sim time stalled at {now:.3f} s', boundaries
                if now >= next_cmd:
                    self.publish_cmd(ph)
                    next_cmd += period
                    if next_cmd < now:
                        next_cmd = now + period
                rclpy.spin_once(self, timeout_sec=0.005)
            boundaries.append(dict(ph, t_start=t_start, t_end=t_end))
            t_start = t_end
        stop = dict(name='end', duration=0.0, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0)
        for _ in range(3):
            self.publish_cmd(stop)
        self.phase_pub.publish(String(data='end'))
        return 0, 'completed', boundaries

    def write_result(self, code, status, boundaries=()):
        truth = None
        if self.truth is not None:
            pos = self.truth.pose.pose.position
            truth = dict(t=self.truth.header.stamp.sec + self.truth.header.stamp.nanosec * 1e-9,
                         x=pos.x, y=pos.y, z=pos.z)
        result = dict(exit_code=code, status=status, phases=list(boundaries), final_truth=truth,
                      scenario_duration_s=sum(p['duration'] for p in self.phases))
        if self.result_file:
            with open(self.result_file, 'w') as f:
                json.dump(result, f, indent=2)
        level = self.get_logger().info if code == 0 else self.get_logger().error
        level(f'scenario {status} (exit {code})')


def main():
    rclpy.init()
    node = ScenarioRunner()
    code = 130
    try:
        code, status = node.startup()
        boundaries = []
        if code == 0:
            code, status, boundaries = node.fly()
        node.write_result(code, status, boundaries)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.write_result(130, 'interrupted')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
