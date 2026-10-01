"""reef_control_node and reef_control_sink as processes (launch_testing).

- A missing required limit makes the controller exit 1 before publishing,
  naming the parameter.
- The sink refuses hardware:=true with exit 2 (NOT IMPLEMENTED).
- Over DDS (own namespace), estimates produce commands stamped with the
  estimate stamps, mode 2, throttle in [0, 1]; the sink's trace records them
  as offboard, then an offboard timeout once the estimates stop.
- At the end launch stops the nodes with SIGINT; they must exit 0.
"""
import csv
import os
import tempfile
import time
import unittest
from pathlib import Path

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from reef_msgs.msg import DesiredState, XYZEstimate
from rosflight_msgs.msg import Command, Status

CONFIG = Path(get_package_share_directory('reef_control')) / 'config' / 'reef_control_quad.yaml'
NS = f'/reef_ctl_lt_{os.getpid()}'
TMP = Path(tempfile.mkdtemp(prefix='reef_ctl_lt_'))
TRACE = TMP / 'trace.csv'


@pytest.mark.launch_test
def generate_test_description():
    good = launch_ros.actions.Node(package='reef_control', executable='reef_control_node', namespace=NS,
                                   parameters=[str(CONFIG)], output='screen')
    sink = launch_ros.actions.Node(package='reef_control', executable='reef_control_sink', namespace=NS,
                                   parameters=[{'trace_file': str(TRACE)}], output='screen')
    bad = launch_ros.actions.Node(package='reef_control', executable='reef_control_node', namespace=NS + '_bad',
                                  parameters=[{'max_pitch': 0.25, 'max_yaw_rate': 2.0}], output='screen')
    hw = launch_ros.actions.Node(package='reef_control', executable='reef_control_sink', namespace=NS + '_hw',
                                 parameters=[{'hardware': True}], output='screen')
    return launch.LaunchDescription([good, sink, bad, hw, launch_testing.actions.ReadyToTest()]), \
        {'good': good, 'sink': sink, 'bad': bad, 'hw': hw}


class TestLive(unittest.TestCase):

    def test_refusals(self, proc_info, proc_output, bad, hw):
        proc_info.assertWaitForShutdown(process=bad, timeout=30)
        launch_testing.asserts.assertExitCodes(proc_info, [1], process=bad)
        proc_output.assertWaitFor('max_roll is required', process=bad, timeout=5)
        proc_info.assertWaitForShutdown(process=hw, timeout=30)
        launch_testing.asserts.assertExitCodes(proc_info, [2], process=hw)
        proc_output.assertWaitFor('NOT IMPLEMENTED', process=hw, timeout=5)

    def test_commands_and_trace(self):
        rclpy.init()
        try:
            node = rclpy.create_node('reef_ctl_lt_driver', namespace=NS)
            got = []
            node.create_subscription(Command, 'command', got.append, 10)
            status = node.create_publisher(Status, 'status', 10)
            desired = node.create_publisher(DesiredState, 'desired_state', 10)
            est = node.create_publisher(XYZEstimate, 'xyz_estimate', 10)
            end = time.time() + 20
            while time.time() < end and (est.get_subscription_count() == 0 or desired.get_subscription_count() == 0
                                         or status.get_subscription_count() < 2):
                rclpy.spin_once(node, timeout_sec=0.1)
            s = Status()
            s.armed = True
            d = DesiredState()
            d.pose.z = -1.0
            d.velocity_valid = True
            d.velocity.x = 0.2
            for _ in range(10):
                status.publish(s)
                desired.publish(d)
                rclpy.spin_once(node, timeout_sec=0.05)
            # Lockstep: one estimate, then wait for its command. The node's
            # subscription keeps only the last message (ROS 1 queue size 1),
            # so a burst under load (colcon runs tests in parallel) may
            # legitimately drop estimates; lockstep removes that timing.
            sent = set()
            for k in range(50):
                m = XYZEstimate()
                m.header.stamp.sec = 1000
                m.header.stamp.nanosec = 4_000_000 * k
                m.z_plus.z = -0.8
                est.publish(m)
                sent.add((1000, 4_000_000 * k))
                end = time.time() + 1.0
                while time.time() < end and not any(
                        (c.header.stamp.sec, c.header.stamp.nanosec) == (1000, 4_000_000 * k) for c in got[-3:]):
                    rclpy.spin_once(node, timeout_sec=0.01)
            self.assertEqual(len({(c.header.stamp.sec, c.header.stamp.nanosec) for c in got} & sent), 50)
            for c in got:
                self.assertIn((c.header.stamp.sec, c.header.stamp.nanosec), sent)
                self.assertEqual(c.mode, Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE)
                self.assertTrue(0.0 <= c.u[3] <= 1.0)
            self.assertLess(got[-1].u[1], 0.0)   # forward velocity request: nose down
            time.sleep(0.5)                      # > 100 ms without commands
        finally:
            rclpy.shutdown()
        rows = list(csv.DictReader(open(TRACE)))
        events = [r['event'] for r in rows]
        self.assertIn('armed', events)
        cmd = [r for r in rows if r['event'] == 'command']
        self.assertGreaterEqual(len(cmd), 50)
        self.assertTrue(all(r['src_F'] == 'offboard' and r['motors'] == 'offboard throttle' for r in cmd[-5:]))
        self.assertIn('offboard_timeout', events[events.index('command'):])
        last = rows[-1]
        self.assertEqual((last['event'], last['src_F']), ('offboard_timeout', 'rc'))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_sigint_exit(self, proc_info, good, sink):
        launch_testing.asserts.assertExitCodes(proc_info, [0], process=good)
        launch_testing.asserts.assertExitCodes(proc_info, [0], process=sink)
