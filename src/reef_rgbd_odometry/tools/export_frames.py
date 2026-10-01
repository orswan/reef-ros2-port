#!/usr/bin/env python3
"""Export paired RGB-D frames from a camera bag for vo_replay (P08 development tool).

    export_frames.py CAMERA_BAG OUT_DIR

CAMERA_BAG: RUN_DIR/camera_bag (run_x3_scenario.sh --vision with
REEF_X3_RECORD_CAMERA=1). Pairs RGB and depth by identical stamps, converts
RGB to grey with OpenCV's weights (as the node: cv::COLOR_RGB2GRAY), writes
meta.txt, stamps.txt, grey.u8, depth.f32.
"""
import sys
from pathlib import Path

import numpy as np


def main(argv):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, Image
    bag, out = Path(argv[1]), Path(argv[2])
    out.mkdir(parents=True, exist_ok=True)
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    rgb, dep, info = {}, {}, None
    while r.has_next():
        topic, raw, _ = r.read_next()
        if topic == '/x3/camera/camera_info' and info is None:
            info = deserialize_message(raw, CameraInfo)
            continue
        if topic not in ('/x3/camera/image', '/x3/camera/depth'):
            continue
        m = deserialize_message(raw, Image)
        t = m.header.stamp.sec * 10**9 + m.header.stamp.nanosec
        (rgb if topic == '/x3/camera/image' else dep)[t] = m
    import cv2
    stamps = sorted(set(rgb) & set(dep))
    with open(out / 'grey.u8', 'wb') as g, open(out / 'depth.f32', 'wb') as d:
        for t in stamps:
            a, b = rgb[t], dep[t]
            img = np.frombuffer(bytes(a.data), np.uint8).reshape(a.height, a.width, 3)
            g.write(cv2.cvtColor(img, cv2.COLOR_RGB2GRAY).tobytes())
            d.write(np.frombuffer(bytes(b.data), np.float32).reshape(b.height, b.width).tobytes())
    (out / 'stamps.txt').write_text(''.join(f'{t}\n' for t in stamps))
    k = info.k
    (out / 'meta.txt').write_text(f'{len(stamps)} {info.width} {info.height} {k[0]} {k[4]} {k[2]} {k[5]}\n')
    print(f'{len(stamps)} paired frames ({len(rgb)} RGB, {len(dep)} depth) -> {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
