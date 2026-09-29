#!/usr/bin/env python3
"""Assert that ROS 2 receives advancing simulation time on /clock.

Checks both the raw /clock messages and a use_sim_time node clock.
Exit code 0 on success, 1 on failure.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def to_sec(t):
    return t.sec + t.nanosec * 1e-9


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    first_msg_timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0

    rclpy.init()
    node = Node('reef_clock_check',
                parameter_overrides=[Parameter('use_sim_time', value=True)])
    stamps = []
    node.create_subscription(
        Clock, '/clock', lambda m: stamps.append(to_sec(m.clock)), qos_profile_sensor_data)

    deadline = time.monotonic() + first_msg_timeout
    while not stamps and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not stamps:
        print(f'FAIL no /clock message within {first_msg_timeout:.0f}s wall time')
        return 1

    node_t0 = node.get_clock().now().nanoseconds * 1e-9
    wall0 = time.monotonic()
    n0 = len(stamps)
    while time.monotonic() - wall0 < duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    wall = time.monotonic() - wall0
    node_t1 = node.get_clock().now().nanoseconds * 1e-9

    window = stamps[n0 - 1:]
    sim_dt = window[-1] - window[0]
    monotonic = all(b >= a for a, b in zip(window, window[1:]))
    msgs = len(window) - 1
    print(f'wall window      : {wall:.2f} s')
    print(f'/clock messages  : {msgs} ({msgs / wall:.1f} Hz)')
    print(f'/clock sim time  : {window[0]:.3f} -> {window[-1]:.3f} s (dt {sim_dt:.3f} s)')
    print(f'real-time factor : {sim_dt / wall:.2f}')
    print(f'node sim clock   : {node_t0:.3f} -> {node_t1:.3f} s (use_sim_time=True)')
    print(f'monotonic        : {monotonic}')

    ok = msgs >= 2 and sim_dt > 0 and monotonic and node_t1 > node_t0 > 0
    print('PASS sim time advances in ROS 2' if ok else 'FAIL sim time did not advance')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
