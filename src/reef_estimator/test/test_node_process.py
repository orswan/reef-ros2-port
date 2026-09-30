"""reef_estimator_node as a process (launch_testing).

- An invalid parameter file makes the node exit with status 1 before
  publishing, naming the parameter.
- Over DDS, a lockstep IMU/range stream produces xyz_estimate values equal to
  the core's for the same events (reef_estimator_event_replay, core mode).
- Output QoS as seen by a subscriber: reliable, transient local.
- At the end of the test launch stops the node with SIGINT; it must exit 0
  (before launch would escalate to SIGTERM after 5 s).
"""
import csv
import os
import subprocess
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
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from reef_msgs.msg import XYZEstimate
from sensor_msgs.msg import Imu, Range

CONFIG = Path(get_package_share_directory('reef_estimator')) / 'config'
REPLAY = Path(get_package_prefix('reef_estimator')) / 'lib' / 'reef_estimator' / 'reef_estimator_event_replay'
NS = f'/reef_lt_{os.getpid()}'
TMP = Path(tempfile.mkdtemp(prefix='reef_lt_'))
T0 = 1_000_000_000
DT = 2_000_000


def events():
    """IMU at 500 Hz (level; vibration from sample 20), range 0.30 m + small steps every 10th."""
    out = []
    for k in range(200):
        t = T0 + k * DT
        if k % 10 == 0:
            out.append(('range', t, 0.30 + 0.001 * (k % 7)))
        vib = (1.5 if k % 2 else -1.5) if k >= 20 else 0.0
        out.append(('imu', t, -9.81 + vib))
    return out


def write_inputs():
    """Harness-format parameters (master + simulation) and events for the replay driver."""
    merged = {}
    for f in ('estimator_master.yaml', 'simulation.yaml'):
        merged.update(yaml.safe_load((CONFIG / f).read_text())['/**']['ros__parameters'])
    lines = []
    for k, v in sorted(merged.items()):
        if isinstance(v, bool):
            lines.append(f'{k} bool {"true" if v else "false"}')
        elif isinstance(v, list):
            lines.append(f'{k} list ' + ' '.join(repr(float(x)) for x in v))
        else:
            lines.append(f'{k} double {float(v)!r}')
    (TMP / 'params.params').write_text('\n'.join(lines) + '\n')
    ev = []
    for typ, t, val in events():
        ev.append(f'range {t} {val!r} 7.65' if typ == 'range' else f'imu {t} 0.0 0.0 {val!r} 0.0 0.0 0.0 1.0')
    (TMP / 'stream.events').write_text('\n'.join(ev) + '\n')
    bad = {'/**': {'ros__parameters': {'z_Q': [0.03, 0.0001, 0.1]}}}
    (TMP / 'bad.yaml').write_text(yaml.safe_dump(bad))


@pytest.mark.launch_test
def generate_test_description():
    write_inputs()
    master, sim = str(CONFIG / 'estimator_master.yaml'), str(CONFIG / 'simulation.yaml')
    good = launch_ros.actions.Node(package='reef_estimator', executable='reef_estimator_node',
                                   namespace=NS, parameters=[master, sim], output='screen')
    bad = launch_ros.actions.Node(package='reef_estimator', executable='reef_estimator_node',
                                  namespace=NS + '_bad', parameters=[master, str(TMP / 'bad.yaml')],
                                  output='screen')
    return launch.LaunchDescription([good, bad, launch_testing.actions.ReadyToTest()]), \
        {'good': good, 'bad': bad}


class TestNode(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('reef_lt_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, done, timeout=10.0):
        end = time.monotonic() + timeout
        while not done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        return done()

    def test_invalid_parameters_exit_1(self, proc_info, proc_output, bad):
        proc_info.assertWaitForShutdown(process=bad, timeout=20)
        proc_output.assertWaitFor('z_Q has 3 values', process=bad, timeout=5)
        self.assertEqual(self.node.count_publishers(NS + '_bad/xyz_estimate'), 0)

    def test_transport_matches_core(self, good):
        expected_csv = TMP / 'expected.csv'
        subprocess.run([str(REPLAY), str(TMP / 'params.params'), str(TMP / 'stream.events'),
                        str(expected_csv)], check=True)
        with open(expected_csv) as fh:
            rows = list(csv.DictReader(fh))
        expected = {}
        last = 0
        for r in rows:   # value published after each IMU that produced an estimate
            if r['type'] == 'imu' and int(r['n_published']) > last:
                last = int(r['n_published'])
                expected[int(r['t_ns'])] = (float(r['z']), float(r['zdot']))
        self.assertGreater(len(expected), 150)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        got, echoes = {}, set()
        self.node.create_subscription(
            XYZEstimate, NS + '/xyz_estimate',
            lambda m: got.__setitem__(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec,
                                      (m.z_plus.z, m.z_plus.z_dot)), latched)
        self.node.create_subscription(
            Range, NS + '/sonar_ned',
            lambda m: echoes.add(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec), 10)
        imu_pub = self.node.create_publisher(Imu, NS + '/imu/data', 10)
        range_pub = self.node.create_publisher(Range, NS + '/sonar', 10)
        # Both directions must be matched: our publishers to the node, and the
        # node's echo/output publishers to our subscriptions (best effort drops
        # messages sent before discovery completes).
        self.assertTrue(self.spin_until(lambda: imu_pub.get_subscription_count() == 1
                                        and range_pub.get_subscription_count() == 1
                                        and self.node.count_publishers(NS + '/sonar_ned') == 1
                                        and self.node.count_publishers(NS + '/xyz_estimate') == 1), 'discovery')
        self.spin_until(lambda: False, timeout=0.5)

        for typ, t, val in events():
            stamp = rclpy.time.Time(nanoseconds=t).to_msg()
            if typ == 'range':
                m = Range()
                m.header.stamp, m.range, m.max_range = stamp, float(val), 7.65
                # Re-send if not echoed within 1 s (a best-effort drop). A repeated
                # range before the next IMU step is gated against the same state
                # and stores the same value, so z and z_dot are unaffected.
                for _ in range(10):
                    range_pub.publish(m)
                    if self.spin_until(lambda: t in echoes, timeout=1.0):
                        break
                self.assertIn(t, echoes, f'range {t} not processed')
            else:
                m = Imu()
                m.header.stamp = stamp
                m.linear_acceleration.z = float(val)
                m.orientation.w = 1.0
                imu_pub.publish(m)
                if t in expected:
                    self.assertTrue(self.spin_until(lambda: t in got), f'no estimate for {t}')
                else:
                    time.sleep(0.02)   # initialization: no output to wait for
        self.assertEqual(sorted(got), sorted(expected))
        mismatched = [t for t in expected if got[t] != expected[t]]
        self.assertEqual(mismatched, [], 'transported values differ from the core')

    def test_output_qos(self, good):
        self.assertTrue(self.spin_until(lambda: len(self.node.get_publishers_info_by_topic(
            NS + '/xyz_estimate')) == 1))
        for topic in ('/xyz_estimate', '/is_flying_reef'):
            info = self.node.get_publishers_info_by_topic(NS + topic)[0].qos_profile
            self.assertEqual(info.reliability, ReliabilityPolicy.RELIABLE, topic)
            self.assertEqual(info.durability, DurabilityPolicy.TRANSIENT_LOCAL, topic)


@launch_testing.post_shutdown_test()
class TestExit(unittest.TestCase):

    def test_exit_codes(self, proc_info, good, bad):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0], process=good)
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[1], process=bad)
