#!/usr/bin/env python3
"""List the publishers of ROS topics in the current domain (JSON on stdout).

    topic_sources.py [--wait S] TOPIC [TOPIC ...]

Waits S seconds (default 2) for discovery, then prints
{"TOPIC": ["/ns/node", ...], ...}. Uses rclpy directly, not the ros2 CLI
daemon (whose cached graph can be stale). Exit 0.
"""
import argparse
import json
import time

import rclpy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--wait', type=float, default=2.0)
    ap.add_argument('topics', nargs='+')
    a = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('reef_topic_sources')
    end = time.monotonic() + a.wait
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    out = {}
    for t in a.topics:
        info = node.get_publishers_info_by_topic(t)
        out[t] = sorted(f"{i.node_namespace.rstrip('/')}/{i.node_name}" for i in info)
    print(json.dumps(out))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
