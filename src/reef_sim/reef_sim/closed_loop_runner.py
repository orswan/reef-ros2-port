"""Closed-loop X3 scenario (P07): setpoints for reef_control and arming of the
stand-in low-level loop, phase by phase in simulation time.

Simulation time is read from the stamps of /x3/truth/odom (100 Hz), not from
/clock: the node runs without use_sim_time, because following /clock (about
500 Hz) cost a Python node about half a core and competed with the control
loop (P08). Phase changes therefore land within one truth sample (10 ms) of
their scheduled times; the recorded boundaries are the scheduled times.

REEF is IN the control loop here: the stand-in (reef_fc_standin, a
development tool) drives the motors from reef_control's commands, and
reef_control uses only the REEF estimate and these setpoints. Nothing
publishes /x3/cmd_vel and the world has no stock controller.

Before arming it waits (bounded) for /clock, truth odometry, and exactly one
publisher on each required stream; if record_topics is set, also for the
recorder's subscriptions. Phases are parallel arrays: name, duration,
armed, z (REEF NED altitude setpoint, m), vx, vy (body-level, m/s; y right),
yaw_rate (rad/s), and optionally (P07b) mode ('velocity' or 'position'), px,
py (mocap NED position setpoint, m), heading (rad), setpoint (false: publish
no setpoint in that phase), fault (a fault injected at the phase start):
  imu_drop S | range_drop S | velocity_drop S | controller_exit | standin_exit
      -> published on /x3/test/fault for the labelled test hooks
  estimator_reset  -> the estimator's ~/reset service
  pause S          -> Gazebo world paused for S wall seconds, then resumed
Every fault and its outcome is written to the result (faults).

Publishes: /x3/reef/desired_state (reef_msgs/DesiredState, 50 Hz),
/x3/fc/arm (std_msgs/Bool, 10 Hz), /x3/scenario/phase (std_msgs/String,
transient local). Writes result_file (JSON).
Exit status: 0 completed, 3 startup timeout or stall, 4 unexpected publishers.
"""
import json
import os
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from reef_msgs.msg import DesiredState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

REQUIRED = ['/clock', '/x3/truth/odom', '/x3/imu', '/x3/range', '/x3/reef/xyz_estimate',
            '/x3/reef/command', '/x3/fc/motor_speed', '/x3/reef/status']
FORBIDDEN = ['/x3/cmd_vel']   # the stock controller's command: must have no publisher


class ClosedLoopRunner(Node):

    def __init__(self):
        super().__init__('closed_loop_runner')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('closed_loop_runner reads sim time from /x3/truth/odom stamps; run it without use_sim_time')
        p = self.declare_parameter
        self.setpoint_rate = p('setpoint_rate_hz', 50.0).value
        self.startup_timeout = p('startup_timeout_s', 90.0).value
        self.stall_timeout = p('stall_timeout_s', 20.0).value
        self.result_file = p('result_file', '').value
        self.record_topics = [t for t in p('record_topics', ['']).value if t]
        names = list(p('phase_names', ['']).value)
        cols = {k: list(p(f'phase_{k}', [0.0]).value) for k in ('durations', 'z', 'vx', 'vy', 'yaw_rate')}
        armed = list(p('phase_armed', [False]).value)
        if not names or any(len(v) != len(names) for v in list(cols.values()) + [armed]):
            raise ValueError('phase_* parameters must be non-empty arrays of equal length')
        n = len(names)

        def optional(key, default):
            v = list(p(f'phase_{key}', [default]).value)
            if v == [default] and n != 1:
                v = [default] * n
            if len(v) != n:
                raise ValueError(f'phase_{key} must have {n} entries')
            return v
        mode, px, py = optional('mode', 'velocity'), optional('px', 0.0), optional('py', 0.0)
        heading, setpoint, fault = optional('heading', 0.0), optional('setpoint', True), optional('fault', 'none')
        self.phases = [dict(name=nm, duration=float(cols['durations'][i]), armed=bool(armed[i]),
                            z=float(cols['z'][i]), vx=float(cols['vx'][i]), vy=float(cols['vy'][i]),
                            yaw_rate=float(cols['yaw_rate'][i]), mode=mode[i], px=float(px[i]), py=float(py[i]),
                            heading=float(heading[i]), setpoint=bool(setpoint[i]),
                            fault='' if fault[i] == 'none' else fault[i]) for i, nm in enumerate(names)]
        if any(ph['mode'] not in ('velocity', 'position') for ph in self.phases):
            raise ValueError("phase_mode entries must be 'velocity' or 'position'")
        self.world = p('world_name', 'x3_closed_loop').value
        self.fault_pub = self.create_publisher(String, '/x3/test/fault', 10)
        self.reset_client = self.create_client(Trigger, '/x3/reef/reef_estimator/reset')
        self.faults = []
        self.desired_pub = self.create_publisher(DesiredState, '/x3/reef/desired_state', 10)
        self.arm_pub = self.create_publisher(Bool, '/x3/fc/arm', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.phase_pub = self.create_publisher(String, '/x3/scenario/phase', latched)
        self.truth = None
        self.create_subscription(Odometry, '/x3/truth/odom', self.on_truth, 10)

    def on_truth(self, msg):
        self.truth = msg

    def now_s(self):
        """Simulation time [s]: the stamp of the latest truth odometry (0 before the first)."""
        if self.truth is None:
            return 0.0
        return self.truth.header.stamp.sec + self.truth.header.stamp.nanosec * 1e-9

    def spin_until(self, pred, wall_limit):
        deadline = time.monotonic() + wall_limit
        while not pred():
            if time.monotonic() > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    def startup(self):
        if not self.spin_until(lambda: self.now_s() > 0 and self.truth is not None, self.startup_timeout):
            return 3, 'no sim clock or truth odometry within startup timeout'
        ok = self.spin_until(lambda: all(self.count_publishers(t) >= 1 for t in REQUIRED), 30.0)
        counts = {t: self.count_publishers(t) for t in REQUIRED}
        if not ok:
            return 3, f'required publishers missing: {counts}'
        if any(c != 1 for c in counts.values()):
            return 4, f'expected exactly one publisher per stream, got {counts}'
        stock = {t: self.count_publishers(t) for t in FORBIDDEN}
        if any(stock.values()):
            return 4, f'stock-controller command has publishers: {stock}'
        # Discovery of subscriptions and node names can lag the publisher
        # counts: read the graph until it is complete (bounded).
        self.graph = self.read_graph()
        deadline = time.monotonic() + 20.0
        while not self.graph_complete(self.graph) and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            self.graph = self.read_graph()
        recorded = [t for t in self.record_topics if self.count_publishers(t) > 0]
        if recorded and not self.spin_until(lambda: all(self.count_subscribers(t) >= 1 for t in recorded), 20.0):
            return 3, 'recorder did not subscribe to ' + str([t for t in recorded if self.count_subscribers(t) < 1])
        return 0, 'ok'

    def read_graph(self):
        """The controller's subscriptions and who publishes each, plus the
        publishers of the motor command and of the stock controller's topic."""
        def pubs(topic):
            return sorted(f'{i.node_namespace.rstrip("/")}/{i.node_name}' for i in self.get_publishers_info_by_topic(topic))
        g = {'controller_inputs': {}, 'motor_command_publishers': pubs('/x3/fc/motor_speed'),
             'stock_command_publishers': pubs('/x3/cmd_vel')}
        try:
            subs = self.get_subscriber_names_and_types_by_node('reef_control_pid', '/x3/reef')
        except Exception as e:  # node not found
            g['error'] = str(e)
            return g
        for topic, _ in subs:
            g['controller_inputs'][topic] = pubs(topic)
        return g

    @staticmethod
    def graph_complete(g):
        inputs = g.get('controller_inputs', {})
        names = [n for pubs in inputs.values() for n in pubs] + g.get('motor_command_publishers', [])
        return ('/x3/reef/xyz_estimate' in inputs and '/x3/reef/desired_state' in inputs
                and inputs['/x3/reef/xyz_estimate'] and g.get('motor_command_publishers')
                and not any('UNKNOWN' in n for n in names))

    def publish(self, ph):
        if not ph['setpoint']:
            return   # P07b stale-setpoint case: the controller keeps the last one
        d = DesiredState()
        d.header.stamp = self.truth.header.stamp   # sim time (read by no consumer)
        d.pose.z = ph['z']
        if ph['mode'] == 'position':
            d.position_valid = True
            d.pose.x, d.pose.y, d.pose.yaw = ph['px'], ph['py'], ph['heading']
        else:
            d.velocity_valid = True
            d.velocity.x = ph['vx']
            d.velocity.y = ph['vy']
            d.velocity.yaw = ph['yaw_rate']
        self.desired_pub.publish(d)

    def inject(self, ph):
        """Inject the phase's fault (P07b) and record what happened."""
        f = ph['fault']
        ev = dict(phase=ph['name'], fault=f, t_sim=self.now_s(), ok=False, detail='')
        kind = f.split()[0]
        if kind in ('imu_drop', 'range_drop', 'velocity_drop', 'controller_exit', 'standin_exit'):
            subs = self.count_subscribers('/x3/test/fault')
            self.fault_pub.publish(String(data=f))
            ev.update(ok=subs > 0, detail=f'published on /x3/test/fault ({subs} hook subscribers)')
        elif kind == 'estimator_reset':
            if self.reset_client.wait_for_service(timeout_sec=2.0):
                fut = self.reset_client.call_async(Trigger.Request())
                deadline = time.monotonic() + 5.0
                while not fut.done() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.01)
                r = fut.result() if fut.done() else None
                ev.update(ok=bool(r and r.success), detail=r.message if r else 'no response')
            else:
                ev.update(detail='reset service unavailable')
        elif kind == 'pause':
            seconds = float(f.split()[1])
            ok = self.world_control('pause: true')
            t_wall = time.monotonic()
            # The gz CLI takes about a second; drain the /clock messages that
            # were already queued before measuring whether sim time stands still.
            while time.monotonic() - t_wall < 0.5:
                rclpy.spin_once(self, timeout_sec=0.05)
            frozen = self.now_s()
            while time.monotonic() - t_wall < seconds:
                rclpy.spin_once(self, timeout_sec=0.05)
            advanced = self.now_s() - frozen
            ok = self.world_control('pause: false') and ok
            ev.update(ok=ok and advanced < 0.01,
                      detail=f'paused {seconds} s wall; sim time advanced {advanced:.4f} s over the last '
                             f'{seconds - 0.5:.1f} s of it')
        else:
            ev.update(detail='unknown fault')
        self.faults.append(ev)
        (self.get_logger().warn if ev['ok'] else self.get_logger().error)(f"FAULT {f}: {ev['detail']}")

    def world_control(self, req):
        cmd = ['gz', 'service', '-s', f'/world/{self.world}/control', '--reqtype', 'gz.msgs.WorldControl',
               '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=10, env=os.environ.copy())
        except (OSError, subprocess.TimeoutExpired):
            return False
        return r.returncode == 0 and 'true' in r.stdout

    def arm(self, on):
        self.arm_pub.publish(Bool(data=bool(on)))

    def fly(self):
        period = 1.0 / self.setpoint_rate
        boundaries, t_start = [], self.now_s()
        next_arm = t_start
        for ph in self.phases:
            self.phase_pub.publish(String(data=ph['name']))
            t_end = t_start + ph['duration']
            self.get_logger().info(f"phase {ph['name']}: {ph['duration']:.1f} s, armed={ph['armed']}, "
                                   f"mode={ph['mode']}, z={ph['z']}, v=({ph['vx']}, {ph['vy']}), "
                                   f"p=({ph['px']}, {ph['py']}), heading={ph['heading']}, "
                                   f"yaw_rate={ph['yaw_rate']}, setpoint={ph['setpoint']}, fault={ph['fault'] or '-'}")
            if ph['fault']:
                self.inject(ph)
            next_sp = t_start
            last_sim, last_wall = self.now_s(), time.monotonic()
            while True:
                now = self.now_s()
                if now >= t_end:
                    break
                if now > last_sim:
                    last_sim, last_wall = now, time.monotonic()
                elif time.monotonic() - last_wall > self.stall_timeout:
                    return 3, f'sim time stalled at {now:.3f} s', boundaries
                if now >= next_sp:
                    self.publish(ph)
                    next_sp = max(next_sp + period, now)
                if now >= next_arm:
                    self.arm(ph['armed'])
                    next_arm = now + 0.1
                rclpy.spin_once(self, timeout_sec=0.05)   # wakes on the next truth sample (100 Hz)
            boundaries.append(dict(ph, t_start=t_start, t_end=t_end))
            t_start = t_end
        self.phase_pub.publish(String(data='end'))
        return 0, 'completed', boundaries

    def write_result(self, code, status, boundaries=()):
        truth = None
        if self.truth is not None:
            pos = self.truth.pose.pose.position
            truth = dict(t=self.truth.header.stamp.sec + self.truth.header.stamp.nanosec * 1e-9,
                         x=pos.x, y=pos.y, z=pos.z)
        result = dict(exit_code=code, status=status, phases=list(boundaries), final_truth=truth,
                      scenario_duration_s=sum(p['duration'] for p in self.phases), mode='closed_loop',
                      graph=getattr(self, 'graph', None), faults=self.faults)
        if self.result_file:
            with open(self.result_file, 'w') as f:
                json.dump(result, f, indent=2)
        (self.get_logger().info if code == 0 else self.get_logger().error)(f'scenario {status} (exit {code})')


def main():
    rclpy.init()
    node = ClosedLoopRunner()
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
