"""Offline replay of an X3 recording through the REEF estimator (no ROS graph).

    ros2 run reef_sim x3_reef_offline RUN_DIR [--out DIR] [--params A.yaml B.yaml ...]

Reads RUN_DIR/bag in recorded order, applies the adapter (reef_adapter.py:
FLU -> FRD, truth attitude, range unchanged) exactly as the live node does,
and writes DIR/inputs.events and DIR/params.params in the reference-harness
format. It then runs reef_estimator_event_replay (core mode) and writes
DIR/estimates.csv. Deterministic: the same recording always gives the same
result. No clock is published, so no /clock source can compete.

Default parameters: reef_estimator's config/estimator_master.yaml followed
by config/simulation.yaml (the files the live launch uses).
"""
import argparse
import subprocess
import sys
from pathlib import Path

import yaml

from reef_sim.reef_adapter import LABEL, Adapter, flu_to_frd

INPUT_TOPICS = ('/x3/imu', '/x3/truth/odom', '/x3/range')


def harness_params(files, out):
    merged = {}
    for f in files:
        merged.update(yaml.safe_load(Path(f).read_text())['/**']['ros__parameters'])
    lines = []
    for k, v in sorted(merged.items()):
        if isinstance(v, bool):
            lines.append(f'{k} bool {"true" if v else "false"}')
        elif isinstance(v, (int, float)):
            lines.append(f'{k} double {float(v)!r}')
        elif isinstance(v, str):
            lines.append(f'{k} string {v}')
        else:
            lines.append(f'{k} list ' + ' '.join(repr(float(x)) for x in v))
    out.write_text('\n'.join(lines) + '\n')
    return merged


def bag_to_events(bag_dir, out):
    """Adapter applied to the bag in recorded order; returns counts."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in INPUT_TOPICS if t not in types]
    if missing:
        raise SystemExit(f'bag lacks {missing}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(INPUT_TOPICS)))
    classes = {t: get_message(types[t]) for t in INPUT_TOPICS}
    adapter = Adapter()
    lines, counts = [], {'imu': 0, 'range': 0, 'truth': 0}

    def ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    def emit(items):
        for t, m, q in items:
            a = flu_to_frd((m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z))
            # event quaternion order: x y z w
            lines.append('imu {} {!r} {!r} {!r} {!r} {!r} {!r} {!r}'.format(t, *a, q[1], q[2], q[3], q[0]))
            counts['imu'] += 1

    while reader.has_next():
        topic, data, _ = reader.read_next()
        m = deserialize_message(data, classes[topic])
        if topic == '/x3/truth/odom':
            o = m.pose.pose.orientation
            emit(adapter.on_truth(ns(m.header.stamp), (o.w, o.x, o.y, o.z)))
            counts['truth'] += 1
        elif topic == '/x3/imu':
            emit(adapter.on_imu(ns(m.header.stamp), m))
        else:
            lines.append(f'range {ns(m.header.stamp)} {float(m.range)!r} {float(m.max_range)!r}')
            counts['range'] += 1
    counts['imu_dropped_before_truth'] = adapter.dropped_early
    counts['imu_pending_at_end'] = len(adapter.pending)
    out.write_text(f'# offline adapter replay of {bag_dir}\n# {LABEL}\n' + '\n'.join(lines) + '\n')
    return counts


def main(argv=None):
    from ament_index_python.packages import get_package_prefix, get_package_share_directory

    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('run_dir', type=Path)
    ap.add_argument('--out', type=Path)
    ap.add_argument('--params', nargs='+')
    a = ap.parse_args(argv)
    cfg = Path(get_package_share_directory('reef_estimator')) / 'config'
    params = a.params or [str(cfg / 'estimator_master.yaml'), str(cfg / 'simulation.yaml')]
    out = a.out or a.run_dir / 'reef_offline'
    out.mkdir(parents=True, exist_ok=True)
    harness_params(params, out / 'params.params')
    counts = bag_to_events(a.run_dir / 'bag', out / 'inputs.events')
    replay = Path(get_package_prefix('reef_estimator')) / 'lib' / 'reef_estimator' / 'reef_estimator_event_replay'
    rc = subprocess.run([str(replay), str(out / 'params.params'), str(out / 'inputs.events'),
                         str(out / 'estimates.csv')]).returncode
    (out / 'offline.yaml').write_text(yaml.safe_dump({
        'run_dir': str(a.run_dir), 'parameter_files': params, 'counts': counts,
        'replay_exit': rc, 'labels': LABEL}, sort_keys=False))
    print(f'offline replay: {counts} -> {out / "estimates.csv"} (exit {rc})')
    return rc


if __name__ == '__main__':
    sys.exit(main())
