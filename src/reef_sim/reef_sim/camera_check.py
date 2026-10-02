"""Live RGB-D camera interface check (P08, ACCEPTANCE.md `vision`, camera interface).

Runs beside a vision simulation. During a static window (the vehicle on the
ground, sim time [t0, t1]) it takes the last RGB, depth, and camera-info
messages and the TRUTH pose (scoring only) and checks:
  intrinsics   fx = fy = w / (2 tan(hfov / 2)), principal point at the image
               centre within 0.5 px, no distortion
  projection   the red target sphere (known world position) is found in the
               image (red pixels nearer than the wall) and its centroid is
               within 1 px of the pinhole projection of its true centre
               through the true camera pose (optical frame: x right, y down,
               z forward)
  depth        metres, planar (along the optical axis): the wall's depth at
               the image centre equals its truth distance within 0.01 m
  invalid      pixels outside the clip range (+-inf or NaN) are counted
               (reported, not a check; "never used" is checked on the odometry)
  stamps       RGB and depth stamps equal; spacing = 1/rate quantized to the
               physics step (sim time)
Writes <out>/camera_check.json and sample frames (.npy). Truth is used only
here, never by any estimator or controller input.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image

T = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)


def rot_flu_to_enu(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class CameraCheck(Node):

    def __init__(self):
        super().__init__('camera_check')
        p = self.declare_parameter
        self.out = Path(p('output_dir', '').value)
        self.t0, self.t1 = p('window_start', 2.0).value, p('window_end', 4.5).value
        self.width, self.height = p('width', 320).value, p('height', 240).value
        self.hfov = p('hfov', 1.0471975511965976).value
        self.rate = p('rate', 15.0).value
        self.mount_flu = np.array(p('mount_flu', [0.0, 0.0, 0.0]).value, float)
        self.target = np.array(p('target', [0.0, 0.0, 0.0]).value, float)
        self.wall_x = p('wall_x', 4.0).value
        self.physics_step = p('physics_step', 0.002).value
        self.img = self.dep = self.info = self.truth = None
        self.stamps = []
        self.depth_stamps = []
        self.done = False
        self.subs = [
            self.create_subscription(Image, '/x3/camera/image', self.on_img, 5),
            self.create_subscription(Image, '/x3/camera/depth', self.on_dep, 5),
            self.create_subscription(CameraInfo, '/x3/camera/camera_info', lambda m: setattr(self, 'info', m), 5),
            self.create_subscription(Odometry, '/x3/truth/odom', lambda m: setattr(self, 'truth', m), 10)]

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_img(self, m):
        self.img = m
        self.stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)

    def on_dep(self, m):
        self.dep = m
        self.depth_stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        if not self.done and self.now() >= self.t1 and self.img is not None and self.info is not None \
                and self.truth is not None:
            self.done = True
            self.evaluate()

    def evaluate(self):
        r = {'checks': []}

        def check(name, ok, detail):
            r['checks'].append(dict(name=name, ok=bool(ok), detail=detail))
            if ok:   # separate call sites: rclpy rejects changing the severity of one call site
                self.get_logger().info(f'PASS {name}: {detail}')
            else:
                self.get_logger().error(f'FAIL {name}: {detail}')
        img, dep, info = self.img, self.dep, self.info
        I = np.frombuffer(bytes(img.data), dtype=np.uint8).reshape(img.height, img.width, 3)
        D = np.frombuffer(bytes(dep.data), dtype=np.float32).reshape(dep.height, dep.width).astype(float)
        np.save(self.out / 'camera_sample_rgb.npy', I)
        np.save(self.out / 'camera_sample_depth.npy', D)
        fx_exp = self.width / (2 * math.tan(self.hfov / 2))
        k = list(info.k)
        ok = (abs(k[0] - fx_exp) < 1e-6 and abs(k[4] - fx_exp) < 1e-6 and abs(k[2] - self.width / 2) <= 0.5
              and abs(k[5] - self.height / 2) <= 0.5 and all(v == 0 for v in info.d) and img.encoding == 'rgb8'
              and dep.encoding == '32FC1' and (img.width, img.height) == (self.width, self.height))
        check('intrinsics and encodings', ok, f'fx {k[0]:.4f} fy {k[4]:.4f} (expected {fx_exp:.4f}), cx {k[2]} cy {k[5]}, '
              f'distortion {list(info.d)}, {img.encoding}/{dep.encoding} {img.width}x{img.height}')
        # True camera pose: base_link (truth) + mount (FLU); camera axes = body axes (forward-looking).
        tp = self.truth.pose.pose
        R = rot_flu_to_enu((tp.orientation.x, tp.orientation.y, tp.orientation.z, tp.orientation.w))
        cam = np.array([tp.position.x, tp.position.y, tp.position.z]) + R @ self.mount_flu
        rel_body = R.T @ (self.target - cam)                       # FLU
        x_opt, y_opt, z_opt = -rel_body[1], -rel_body[2], rel_body[0]
        u_exp, v_exp = k[2] + k[0] * x_opt / z_opt, k[5] + k[4] * y_opt / z_opt
        wall_depth = (R.T @ (np.array([self.wall_x, cam[1], cam[2]]) - cam))[0]
        red = (I[:, :, 0] > 200) & (I[:, :, 1] < 60) & (I[:, :, 2] < 60) & (D < wall_depth - 0.25)
        ys, xs = np.nonzero(red)
        if len(xs):
            # pixel-centre convention: pixel (i, j) covers [j, j+1) x [i, i+1); its centre is j + 0.5
            u, v = xs.mean() + 0.5, ys.mean() + 0.5
            err = math.hypot(u - u_exp, v - v_exp)
            check('projection of the target through the true pose (optical frame)', err <= 1.0,
                  f'measured ({u:.2f}, {v:.2f}) vs projected ({u_exp:.2f}, {v_exp:.2f}): error {err:.2f} px '
                  f'({len(xs)} target pixels)')
        else:
            err = None
            check('projection of the target through the true pose (optical frame)', False, 'target not found')
        c = D[self.height // 2, self.width // 2]
        check('depth in metres, planar: wall at the image centre', math.isfinite(c) and abs(c - wall_depth) <= 0.01,
              f'{c:.4f} m vs truth {wall_depth:.4f} m')
        n = D.size
        r['invalid'] = dict(posinf=int(np.isposinf(D).sum()), neginf=int(np.isneginf(D).sum()), nan=int(np.isnan(D).sum()),
                            fraction=float((~np.isfinite(D)).sum() / n))
        self.get_logger().info(f"REPORTED invalid depth pixels: +inf {r['invalid']['posinf']}, -inf "
                               f"{r['invalid']['neginf']}, NaN {r['invalid']['nan']} of {n} (-inf = nearer than the "
                               'near clip, +inf = beyond the far clip)')
        st = np.array([t for t in self.stamps if self.t0 <= t <= self.t1])   # the static window (startup frames are irregular)
        dt = np.diff(st)
        steps = dt / self.physics_step
        rgb_set = set(self.stamps)
        # the RGB frame of the newest depth frame may still be in transit: pair up to the newest RGB stamp
        paired = [t for t in self.depth_stamps if self.t0 <= t <= min(self.t1, max(self.stamps))]
        unpaired = [t for t in paired if t not in rgb_set]
        steps_ok = bool(np.all(np.abs(steps - np.round(steps)) < 1e-3))
        spacing_ok = bool(np.all(np.abs(dt - 1 / self.rate) <= self.physics_step + 1e-9))
        r['stamp_detail'] = dict(paired=len(paired), unpaired=len(unpaired), quantized=steps_ok, spacing=spacing_ok)
        ok = (paired and not unpaired and len(dt) > 5 and np.all(np.abs(steps - np.round(steps)) < 1e-3)
              and np.all(np.abs(dt - 1 / self.rate) <= self.physics_step + 1e-9))
        check('stamps: every depth frame has an RGB frame with the same stamp; spacing 1/rate quantized to the '
              'physics step (sim time)', ok, f'{len(paired) - len(unpaired)}/{len(paired)} depth frames paired; quantized {steps_ok}; spacings {sorted(set(np.round(dt, 4)))} s '
              f'(1/rate {1 / self.rate:.4f})')
        r.update(projection_error_px=err, wall_depth=float(c), wall_truth=float(wall_depth),
                 fx=k[0], cx=k[2], cy=k[5], frame_id=img.header.frame_id)
        r['estimator_inputs'] = _estimator_inputs(self)
        r['ok'] = all(ch['ok'] for ch in r['checks'])
        (self.out / 'camera_check.json').write_text(json.dumps(r, indent=1))


def _pubs(node, topic):
    return sorted(f'{i.node_namespace.rstrip("/")}/{i.node_name}' for i in node.get_publishers_info_by_topic(topic))


def _estimator_inputs(node):
    try:
        subs = node.get_subscriber_names_and_types_by_node('reef_estimator', '/x3/reef')
    except Exception as e:  # noqa: BLE001  (node not present)
        return {'error': str(e)}
    return {t: _pubs(node, t) for t, _ in subs if t != '/parameter_events'}


def main():
    rclpy.init()
    node = CameraCheck()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
        # The check is complete: stop receiving images and stop following sim time (with use_sim_time the
        # node processes every /clock message, about 500 Hz; together that cost about a core of Python
        # for the rest of the run, competing with the control loop, P08). The node stays up until shutdown.
        for sub in node.subs:
            node.destroy_subscription(sub)
        node.subs = []
        node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, False)])
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
