#!/usr/bin/env python3
"""List the publishers of /clock in the current ROS domain (JSON on stdout).

    clock_sources.py [--wait S]

Waits S seconds (default 2) for discovery, then prints
{"count": N, "publishers": ["/ns/node", ...]}. Uses rclpy directly, not the
ros2 CLI daemon (whose cached graph can be stale). Exit 0.
"""
import argparse
import json
import time

import rclpy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wait', type=float, default=2.0)
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('reef_clock_sources')
    end = time.monotonic() + a.wait
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    info = node.get_publishers_info_by_topic('/clock')
    names = sorted(f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in info)
    print(json.dumps({'count': len(info), 'publishers': names}))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
