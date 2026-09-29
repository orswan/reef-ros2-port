#!/usr/bin/env python3
"""Assert that ROS 2 receives advancing simulation time.

Checks both the raw clock messages and a use_sim_time node clock. With
--topic, the node's /clock (including its time source) is remapped to that
topic, so check_clock_demo.sh can observe only its own per-run bridge.

Exit codes: 0 pass, 1 fail, 130/143 interrupted by SIGINT/SIGTERM.
"""
import argparse
import signal
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

interrupted = 0


def to_sec(t):
    return t.sec + t.nanosec * 1e-9


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('duration', nargs='?', type=float, default=5.0,
                   help='measurement window in wall seconds')
    p.add_argument('first_msg_timeout', nargs='?', type=float, default=60.0,
                   help='wall seconds to wait for the first clock message')
    p.add_argument('--topic', default='/clock', help='clock topic to observe')
    p.add_argument('--publisher-node', default=None,
                   help='require exactly one publisher on the topic, with this node name')
    return p.parse_args()


def run(args):
    node = Node('reef_clock_check',
                cli_args=['--ros-args', '-r', f'/clock:={args.topic}'],
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    stamps = []
    node.create_subscription(
        Clock, '/clock', lambda m: stamps.append(to_sec(m.clock)), qos_profile_sensor_data)
    topic = node.resolve_topic_name('/clock')

    def spin_for(pred, seconds):
        deadline = time.monotonic() + seconds
        while not pred() and not interrupted and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

    spin_for(lambda: stamps, args.first_msg_timeout)
    if interrupted:
        return interrupted
    if not stamps:
        print(f'FAIL no message on {topic} within {args.first_msg_timeout:.0f}s wall time')
        return 1

    node_t0 = node.get_clock().now().nanoseconds * 1e-9
    wall0 = time.monotonic()
    n0 = len(stamps)
    spin_for(lambda: False, args.duration)
    if interrupted:
        return interrupted
    wall = time.monotonic() - wall0
    node_t1 = node.get_clock().now().nanoseconds * 1e-9
    pubs = node.get_publishers_info_by_topic(topic)

    window = stamps[n0 - 1:]
    sim_dt = window[-1] - window[0]
    monotonic = all(b >= a for a, b in zip(window, window[1:]))
    msgs = len(window) - 1
    pub_names = sorted(f'{p.node_namespace.rstrip("/")}/{p.node_name}' for p in pubs)
    print(f'topic            : {topic}')
    print(f'publishers       : {pub_names}')
    print(f'wall window      : {wall:.2f} s')
    print(f'clock messages   : {msgs} ({msgs / wall:.1f} Hz)')
    print(f'clock sim time   : {window[0]:.3f} -> {window[-1]:.3f} s (dt {sim_dt:.3f} s)')
    print(f'real-time factor : {sim_dt / wall:.2f}')
    print(f'node sim clock   : {node_t0:.3f} -> {node_t1:.3f} s (use_sim_time=True)')
    print(f'monotonic        : {monotonic}')

    ok = msgs >= 2 and sim_dt > 0 and monotonic and node_t1 > node_t0 > 0
    if args.publisher_node is not None:
        owned = len(pubs) == 1 and pubs[0].node_name == args.publisher_node
        if not owned:
            print(f'FAIL expected exactly one publisher named {args.publisher_node!r}')
        ok = ok and owned
    print('PASS sim time advances in ROS 2' if ok else 'FAIL sim time did not advance')
    node.destroy_node()
    return 0 if ok else 1


def main():
    args = parse_args()

    def on_signal(signum, _frame):
        global interrupted
        interrupted = 128 + signum

    # Own handlers (not rclpy's) so interruption ends the loop with a clear status.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    try:
        status = run(args)
    except (KeyboardInterrupt, ExternalShutdownException):
        status = interrupted or 130
    if status >= 128:
        print(f'INTERRUPTED by signal {status - 128}')
    rclpy.try_shutdown()
    return status


if __name__ == '__main__':
    sys.exit(main())
