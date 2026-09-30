"""Validate and plot an X3 scenario recording.

    ros2 run reef_sim analyze_x3_bag <run_dir> [--no-plots]

<run_dir> is a directory written by scripts/run_x3_scenario.sh: bag/,
scenario_result.json, and x3_scenario.yaml (the parameters used). Writes
<run_dir>/analysis/validation.json and PNG plots, and prints a check table.

The checks are consistency and plausibility checks on SYNTHETIC data. They
show that the pipeline and simulation behave as documented; they are not a
validation of any hardware or of the idealized range model's realism.

Exit status: 0 all checks passed, 1 a check failed, 2 the run could not be read.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

G = 9.80665
NOMINAL_HZ = {'/x3/imu': 250.0, '/x3/truth/odom': 100.0}   # set in model.sdf / world
REQUIRED = ['/clock', '/x3/truth/odom', '/x3/imu', '/x3/range', '/x3/cmd_vel', '/x3/scenario/phase']


# ---------------------------------------------------------------- reading
def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def read_bag(bag_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {name: get_message(t) for name, t in types.items()}
    data = {name: [] for name in types}
    while reader.has_next():
        topic, raw, t_bag = reader.read_next()
        data[topic].append((t_bag * 1e-9, deserialize_message(raw, classes[topic])))
    return types, data


def arrays(data):
    out = {}
    odom = data.get('/x3/truth/odom', [])
    out['odom'] = np.array([(stamp(m.header), tb,
                             m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z,
                             m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                             m.pose.pose.orientation.z, m.pose.pose.orientation.w,
                             m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z,
                             m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z)
                            for tb, m in odom]).reshape(-1, 15)
    imu = data.get('/x3/imu', [])
    out['imu'] = np.array([(stamp(m.header), tb,
                            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z,
                            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                            m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w,
                            m.orientation_covariance[0])
                           for tb, m in imu]).reshape(-1, 13)
    rng = data.get('/x3/range', [])
    out['range'] = np.array([(stamp(m.header), tb, m.range, m.min_range, m.max_range)
                             for tb, m in rng]).reshape(-1, 5)
    clk = data.get('/clock', [])
    out['clock'] = np.array([(m.clock.sec + m.clock.nanosec * 1e-9, tb) for tb, m in clk]).reshape(-1, 2)
    cmd = data.get('/x3/cmd_vel', [])
    out['cmd'] = np.array([(tb, m.linear.x, m.linear.y, m.linear.z, m.angular.z)
                           for tb, m in cmd]).reshape(-1, 5)
    out['phase_msgs'] = len(data.get('/x3/scenario/phase', []))
    out['imu_frames'] = sorted({m.header.frame_id for _, m in imu})
    out['odom_frames'] = sorted({(m.header.frame_id, m.child_frame_id) for _, m in odom})
    out['range_frames'] = sorted({m.header.frame_id for _, m in rng})
    return out


# -------------------------------------------------------------- geometry
def rot(q):
    """Body->world rotation matrices from quaternions (N,4) as x,y,z,w (independent of geometry.py)."""
    x, y, z, w = (q[:, i] for i in range(4))
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def expected_slant_range(pos, R, offset):
    """Sensor height above z=0 divided by the cosine between the beam (-Z_body) and world down."""
    height = pos[:, 2] + R[:, 2, :] @ offset          # sensor origin z
    cos_tilt = R[:, 2, 2]                              # (-Z_body) . (-z_world)
    with np.errstate(divide='ignore', invalid='ignore'):
        slant = np.where(cos_tilt > 1e-9, height / cos_tilt, np.inf)
    return slant, height, cos_tilt


# ----------------------------------------------------------------- checks
class Checks:
    def __init__(self):
        self.items = []

    def add(self, name, ok, detail, criterion):
        self.items.append(dict(name=name, ok=bool(ok), detail=detail, criterion=criterion))

    @property
    def ok(self):
        return all(i['ok'] for i in self.items)


def phase_window(phases, name):
    for p in phases:
        if p['name'] == name:
            return p
    return None


def in_window(t, t0, t1):
    return (t >= t0) & (t < t1)


def run_checks(a, types, result, params):
    c = Checks()
    rp = params['range_sensor']['ros__parameters']
    phases = result.get('phases', [])
    duration = result.get('scenario_duration_s', 0.0)

    # 1. Streams present, with counts well above zero (not an exact rate).
    for topic in REQUIRED:
        n = {'/clock': len(a['clock']), '/x3/truth/odom': len(a['odom']), '/x3/imu': len(a['imu']),
             '/x3/range': len(a['range']), '/x3/cmd_vel': len(a['cmd']),
             '/x3/scenario/phase': a['phase_msgs']}.get(topic)
        present = topic in types
        if topic in NOMINAL_HZ:
            need = int(0.5 * NOMINAL_HZ[topic] * duration)
        elif topic == '/x3/range':
            need = int(0.5 * rp['rate_hz'] * duration)
        elif topic == '/x3/scenario/phase':
            need = len(phases)
        else:
            need = 10
        c.add(f'stream {topic}', present and (n or 0) >= need,
              f'{"present" if present else "MISSING"}, {n or 0} msgs', f'>= {need} msgs (50% of nominal)')

    if not c.ok:
        return c   # later checks need the streams

    # 2. Timestamps: finite, non-decreasing, covering the scenario, close to bag (sim) time.
    for key, topic in (('odom', '/x3/truth/odom'), ('imu', '/x3/imu'), ('range', '/x3/range')):
        t, tb = a[key][:, 0], a[key][:, 1]
        mono = bool(np.all(np.diff(t) >= 0))
        span = float(t[-1] - t[0])
        lag = float(np.percentile(np.abs(tb - t), 99))
        c.add(f'timestamps {topic}', np.all(np.isfinite(t)) and mono and span >= 0.9 * duration and lag < 0.5,
              f'monotonic={mono}, span {span:.1f} s, p99 |bag-header| {lag * 1e3:.1f} ms',
              f'monotonic, span >= 90% of {duration:.0f} s, p99 lag < 500 ms')
    ct = a['clock'][:, 0]
    c.add('timestamps /clock', np.all(np.diff(ct) >= 0) and ct[-1] - ct[0] >= 0.9 * duration,
          f'{ct[0]:.2f} -> {ct[-1]:.2f} s', 'advancing over the scenario')

    # 3. Finite, documented frames.
    c.add('finite truth', np.all(np.isfinite(a['odom'][:, 2:])), 'pose and twist', 'no NaN/inf')
    c.add('finite IMU', np.all(np.isfinite(a['imu'][:, 2:8])), 'accel and gyro', 'no NaN/inf')
    c.add('IMU orientation not provided', np.all(a['imu'][:, 12] == -1) and np.all(a['imu'][:, 8:12] == 0),
          'orientation_covariance[0] = -1, zero quaternion', 'REP 145: orientation must not carry truth')
    r = a['range'][:, 2]
    in_rng = np.isfinite(r)
    ok_vals = np.all(~np.isnan(r)) and np.all((r[in_rng] >= rp['min_range']) & (r[in_rng] <= rp['max_range']))
    c.add('range values', ok_vals, f'{in_rng.sum()} in range, {np.sum(r == -np.inf)} -inf, '
          f'{np.sum(r == np.inf)} +inf', 'finite within [min,max], else +-inf (REP 117)')
    c.add('frames', a['imu_frames'] == ['x3/base_link'] and a['range_frames'] == [rp['frame_id']]
          and a['odom_frames'] == [('world', 'x3/base_link')],
          f'imu {a["imu_frames"]}, range {a["range_frames"]}, odom {a["odom_frames"]}',
          'as documented')

    # Truth in world frame.
    to, pos = a['odom'][:, 0], a['odom'][:, 2:5]
    Rw = rot(a['odom'][:, 5:9])
    v_world = np.einsum('nij,nj->ni', Rw, a['odom'][:, 9:12])

    # 4. Response to commands: displacement over each commanded phase plus the
    #    settle/hover that follows it, against the commanded v * T.
    settle = phase_window(phases, 'settle')
    if settle:
        m = in_window(to, settle['t_start'], settle['t_end'])
        c.add('settle: vehicle on ground', m.any() and np.max(pos[m, 2]) < 0.1,
              f'max z {np.max(pos[m, 2]):.3f} m', 'z < 0.10 m')
    for i, p in enumerate(phases):
        cmd = np.array([p['vx'], p['vy'], p['vz']])
        if not np.any(cmd):
            continue
        t_end = phases[i + 1]['t_end'] if i + 1 < len(phases) else p['t_end']
        i0 = np.searchsorted(to, p['t_start'])
        i1 = min(np.searchsorted(to, t_end), len(to) - 1)
        disp = pos[i1] - pos[i0]
        want = cmd * p['duration']
        axis = int(np.argmax(np.abs(want)))
        ratio = disp[axis] / want[axis] if want[axis] else float('nan')
        cross = [disp[k] for k in range(3) if want[k] == 0]
        drift = float(np.max(np.abs(cross))) if cross else 0.0
        c.add(f'response {p["name"]}', 0.7 <= ratio <= 1.3 and drift < 0.3,
              f'disp {np.round(disp, 2).tolist()} m vs cmd*T {want.tolist()} (ratio {ratio:.2f}, '
              f'cross-axis {drift:.2f} m)', 'ratio in [0.7, 1.3], cross-axis < 0.3 m')

    # 5. IMU plausibility.
    ti, acc, gyr = a['imu'][:, 0], a['imu'][:, 2:5], a['imu'][:, 5:8]
    if settle:
        m = in_window(ti, settle['t_start'] + 1.0, settle['t_end'])
        mean, std = acc[m].mean(0), acc[m].std(0)
        err = np.linalg.norm(mean - [0, 0, G])
        ip = params.get('imu_noise', {}).get('ros__parameters', {})
        vib = float(ip.get('vibration_std', 0.0))
        if vib > 0:   # REEF runs: configured vibration (imu_noise.py); expect the combined sigma
            sig = float(np.hypot(float(ip.get('accel_noise_std', 0.02)), vib))
            std_ok, std_rule = bool(np.all(np.abs(std - sig) < 0.3 * sig)), f'std within 30% of {sig:.3f} per axis'
        else:
            std_ok, std_rule = bool(np.all(std < 0.1)), 'std < 0.1 per axis'
        c.add('IMU stationary: specific force = +g on z (FLU)', err < 0.2 and std_ok,
              f'mean {np.round(mean, 3).tolist()} m/s^2, std {np.round(std, 3).tolist()}',
              f'|mean - (0,0,9.807)| < 0.2, {std_rule}')
        gm = np.linalg.norm(gyr[m].mean(0))
        c.add('IMU stationary: gyro ~ 0', gm < 0.02, f'|mean| {gm:.4f} rad/s', '< 0.02 rad/s')
    hov = [p for p in phases if p['name'].startswith('hover')]
    if hov:
        m = np.zeros_like(ti, bool)
        for p in hov:
            m |= in_window(ti, p['t_start'] + 1.0, p['t_end'])
        nm = float(np.linalg.norm(acc[m], axis=1).mean())
        c.add('IMU hover: |specific force| ~ g', 9.3 <= nm <= 10.3, f'{nm:.3f} m/s^2', 'in [9.3, 10.3]')
    c.add('IMU bounded', np.max(np.linalg.norm(acc, axis=1)) < 30 and np.max(np.linalg.norm(gyr, axis=1)) < 5,
          f'max |a| {np.max(np.linalg.norm(acc, axis=1)):.2f} m/s^2, max |w| '
          f'{np.max(np.linalg.norm(gyr, axis=1)):.3f} rad/s', '|a| < 30, |w| < 5')
    w_truth = np.column_stack([np.interp(ti, to, a['odom'][:, 12 + k]) for k in range(3)])
    wr = float(np.sqrt(np.mean(np.sum((gyr - w_truth) ** 2, axis=1))))
    c.add('IMU gyro vs truth body rate', wr < 0.05, f'RMS {wr:.4f} rad/s', 'RMS < 0.05 rad/s (FLU body frame)')

    # 6. Range against geometry recomputed from truth at the same stamps.
    tr = a['range'][:, 0]
    idx = np.clip(np.searchsorted(to, tr), 0, len(to) - 1)
    exact = np.abs(to[idx] - tr) < 1e-6
    slant, height, cos_tilt = expected_slant_range(pos[idx], Rw[idx], np.array(rp['sensor_offset']))
    sig = float(rp['noise_std'])
    both = exact & in_rng & (slant >= rp['min_range']) & (slant <= rp['max_range'])
    res = r[both] - slant[both]
    tol_mean = 3 * max(sig, 1e-4) / math.sqrt(max(both.sum(), 1)) + 1e-4
    std_ok = (0.7 * sig <= res.std() <= 1.3 * sig) if sig > 0 else res.std() < 1e-6
    c.add('range vs slant geometry', both.sum() > 100 and abs(res.mean()) < tol_mean and std_ok,
          f'n={both.sum()}, residual mean {res.mean() * 1e3:.2f} mm, std {res.std() * 1e3:.2f} mm '
          f'(noise_std {sig * 1e3:.1f} mm)', f'|mean| < {tol_mean * 1e3:.2f} mm, std within 30% of noise_std')
    margin = 3 * sig + 1e-6
    below = exact & (slant < rp['min_range'] - margin)
    above = exact & (slant > rp['max_range'] + margin)
    c.add('range out-of-limit encoding', np.all(r[below] == -np.inf) and np.all(r[above] == np.inf),
          f'{below.sum()} expected below min (all -inf: {np.all(r[below] == -np.inf)}), '
          f'{above.sum()} above max', 'below min -> -inf, above max -> +inf')
    tilt = np.degrees(np.arccos(np.clip(cos_tilt, -1, 1)))
    c.add('range geometry note', True,
          f'max tilt {tilt.max():.1f} deg; max slant - vertical height {np.max(slant[both] - height[both]) * 1e3:.1f} mm',
          'informational')

    # 7. Truth self-consistency: odom twist (rotated to world) vs d(position)/dt.
    dt = np.diff(to)
    fd = np.diff(pos, axis=0) / dt[:, None]
    vm = 0.5 * (v_world[1:] + v_world[:-1])
    vr = float(np.sqrt(np.mean(np.sum((fd - vm) ** 2, axis=1))))
    c.add('truth twist vs d(pose)/dt', vr < 0.1, f'RMS {vr:.4f} m/s', '< 0.1 m/s (twist is a smoothed FD)')

    a['_derived'] = dict(v_world=v_world, slant=slant, height=height, tilt=tilt, range_exact=exact)
    return c


# ------------------------------------------------------------------ plots
def plots(a, phases, params, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    d = a['_derived']
    rp = params['range_sensor']['ros__parameters']

    def shade(ax):
        for p in phases:
            moving = any((p['vx'], p['vy'], p['vz']))
            ax.axvspan(p['t_start'], p['t_end'], color='tab:orange' if moving else 'tab:gray',
                       alpha=0.12 if moving else 0.06, lw=0)

    def phase_labels(ax):
        for p in phases:
            ax.text(0.5 * (p['t_start'] + p['t_end']), 1.0, p['name'], rotation=90, fontsize=7,
                    ha='center', va='top', transform=ax.get_xaxis_transform(), color='0.3')

    to = a['odom'][:, 0]
    fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    shade(ax[0])
    ax[0].plot(to, a['odom'][:, 4], label='truth z (base_link origin)')
    ax[0].set_ylabel('altitude [m]')
    ax[0].legend(loc='upper right')
    phase_labels(ax[0])
    for k, n in enumerate('xyz'):
        shade(ax[1]) if k == 0 else None
        ax[1].plot(to, d['v_world'][:, k], label=f'truth v_{n} (world ENU)')
    if len(a['cmd']):
        for k, n in enumerate('xyz'):
            ax[1].step(a['cmd'][:, 0], a['cmd'][:, 1 + k], where='post', ls='--', lw=1,
                       label=f'cmd v_{n} (body FLU)')
    ax[1].set_ylabel('velocity [m/s]')
    ax[1].legend(loc='upper right', ncol=2, fontsize=8)
    shade(ax[2])
    for k, n in enumerate('xy'):
        ax[2].plot(to, a['odom'][:, 2 + k], label=f'truth {n}')
    ax[2].set_ylabel('position [m]')
    ax[2].set_xlabel('sim time [s]')
    ax[2].legend(loc='upper right')
    fig.suptitle('Ground truth (/x3/truth/odom). Orange: commanded motion; gray: settle/hover')
    fig.tight_layout()
    fig.savefig(out / 'truth_altitude_velocity.png', dpi=110)
    plt.close(fig)

    tr, r = a['range'][:, 0], a['range'][:, 2]
    fin = np.isfinite(r)
    fig, ax = plt.subplots(2, 1, figsize=(11, 7), sharex=True, gridspec_kw=dict(height_ratios=[3, 1]))
    shade(ax[0])
    ax[0].plot(tr, d['slant'], 'k-', lw=1, label='expected slant range (truth geometry)')
    ax[0].plot(tr, d['height'], 'c:', lw=1.5, label='sensor height above ground (vertical)')
    ax[0].plot(tr[fin], r[fin], '.', ms=3, label='/x3/range (idealized, noisy)')
    lo = ~fin & (r < 0)
    hi = ~fin & (r > 0)
    ax[0].plot(tr[lo], np.full(lo.sum(), rp['min_range']), 'rv', ms=3, label='-inf (below min_range)')
    if hi.any():
        ax[0].plot(tr[hi], np.full(hi.sum(), rp['max_range']), 'r^', ms=3, label='+inf')
    ax[0].axhline(rp['min_range'], color='r', lw=0.5, ls='--')
    ax[0].set_ylabel('range [m]')
    ax[0].legend(loc='upper right', fontsize=8)
    phase_labels(ax[0])
    both = fin & np.isfinite(d['slant'])
    ax[1].plot(tr[both], (r[both] - d['slant'][both]) * 1e3, '.', ms=2)
    ax[1].set_ylabel('residual [mm]')
    ax[1].set_xlabel('sim time [s]')
    fig.suptitle('Idealized downward range vs geometry from truth '
                 f'(offset {rp["sensor_offset"]} m, noise {rp["noise_std"]} m)')
    fig.tight_layout()
    fig.savefig(out / 'range_vs_geometry.png', dpi=110)
    plt.close(fig)

    ti = a['imu'][:, 0]
    fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for axi in ax:
        shade(axi)
    for k, n in enumerate('xyz'):
        ax[0].plot(ti, a['imu'][:, 2 + k], lw=0.5, label=f'a_{n}')
    ax[0].set_ylabel('specific force [m/s^2]')
    ax[0].legend(loc='right')
    phase_labels(ax[0])
    ax[1].plot(ti, np.linalg.norm(a['imu'][:, 2:5], axis=1), lw=0.5, label='|a|')
    ax[1].axhline(G, color='k', lw=0.8, ls='--', label='g')
    ax[1].set_ylabel('|specific force| [m/s^2]')
    ax[1].legend(loc='right')
    for k, n in enumerate('xyz'):
        ax[2].plot(ti, a['imu'][:, 5 + k], lw=0.5, label=f'w_{n}')
    ax[2].set_ylabel('angular rate [rad/s]')
    ax[2].set_xlabel('sim time [s]')
    ax[2].legend(loc='right')
    fig.suptitle('IMU (/x3/imu, x3/base_link FLU). Gray: stationary/hover; orange: commanded motion')
    fig.tight_layout()
    fig.savefig(out / 'imu.png', dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir', type=Path)
    ap.add_argument('--no-plots', action='store_true')
    args = ap.parse_args()
    run = args.run_dir
    try:
        params = yaml.safe_load((run / 'x3_scenario.yaml').read_text())
        result = json.loads((run / 'scenario_result.json').read_text())
        types, data = read_bag(run / 'bag')
    except Exception as e:  # noqa: BLE001 - report any unreadable input
        print(f'FAIL cannot read run: {e}')
        return 2
    a = arrays(data)
    checks = run_checks(a, types, result, params)
    out = run / 'analysis'
    out.mkdir(exist_ok=True)
    if not args.no_plots and '_derived' in a:
        plots(a, result.get('phases', []), params, out)
    (out / 'validation.json').write_text(json.dumps(
        dict(ok=checks.ok, scenario_status=result.get('status'), checks=checks.items), indent=2))
    for i in checks.items:
        print(f"{'PASS' if i['ok'] else 'FAIL'} {i['name']}: {i['detail']}  [{i['criterion']}]")
    print('ANALYSIS PASSED' if checks.ok else 'ANALYSIS FAILED')
    return 0 if checks.ok else 1


if __name__ == '__main__':
    sys.exit(main())
