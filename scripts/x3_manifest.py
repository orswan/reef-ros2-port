#!/usr/bin/env python3
"""Write or finalize <run_dir>/manifest.yaml for an X3 scenario recording.

    x3_manifest.py start  <run_dir> key=value ...   # before the flight
    x3_manifest.py finish <run_dir> key=value ...   # after it (adds an outcome section)

Records the scenario, source revision, model provenance, software versions,
parameters, seeds, and isolation settings needed to interpret or reproduce
the recording. Values given as key=value are stored as strings.
"""
import datetime
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def run(*cmd):
    try:
        return subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=20).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ''


def dpkg_version(pkg):
    return run('dpkg-query', '-W', '-f=${Version}', pkg) or None


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def controller_label(extra):
    # What commands the motors. In a closed-loop run the stock controller is
    # absent from the world (x3_closed_loop*.sdf) and REEF flies the vehicle.
    if extra.get('closed_loop') == 'true':
        return ('REEF controller -> STAND-IN low-level loop (development tool, not ROSflight); '
                'the stock gz-sim MulticopterVelocityControl is absent from this world')
    return 'gz-sim MulticopterVelocityControl, fed by SIMULATION TRUTH (not REEF)'


def start(run_dir, extra):
    params = yaml.safe_load((run_dir / 'x3_scenario.yaml').read_text())
    asset_manifests = {}
    for m in sorted((ROOT / 'src' / 'reef_sim' / 'assets').glob('*.json')):
        d = json.loads(m.read_text())
        asset_manifests[d['name']] = {
            'fuel_version': d['fuel_version'], 'source_url': d['source_url'],
            'archive_sha256': d['archive_sha256'], 'license': d['license']['name'],
            'manifest': str(m.relative_to(ROOT)), 'manifest_sha256': sha256(m)}
    world = 'src/reef_sim/worlds/' + extra.pop('world', 'x3_flight.sdf')
    vehicle = 'src/reef_sim/models/' + extra.pop('vehicle_model', 'reef_x3') + '/model.sdf'
    image_pkgs = Path('/etc/reef-image-packages.txt')
    dirty = run('git', 'status', '--porcelain')   # includes untracked, excludes ignored
    manifest = {
        'scenario': 'reef_sim x3_scenario',
        'run_id': run_dir.name,
        'created_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'),
        'source': {
            'git_commit': run('git', 'rev-parse', 'HEAD'),
            'git_branch': run('git', 'rev-parse', '--abbrev-ref', 'HEAD'),
            'uncommitted_changes': dirty.splitlines() if dirty else [],
        },
        'environment': {
            'container': 'reef_ros2_dev image' if image_pkgs.exists() else 'other (e.g. ros2_novnc_container)',
            'image_package_list_sha256': sha256(image_pkgs) if image_pkgs.exists() else None,
            'gz_sim': run('gz', 'sim', '--versions'),
            'ros_gz_sim': dpkg_version('ros-jazzy-ros-gz-sim'),
            'ros_gz_bridge': dpkg_version('ros-jazzy-ros-gz-bridge'),
            'rosbag2': dpkg_version('ros-jazzy-rosbag2'),
        },
        'model': {
            'world': world,
            'world_sha256': sha256(ROOT / world),
            'vehicle_model': vehicle,
            'vehicle_model_sha256': sha256(ROOT / vehicle),
            'third_party_assets': asset_manifests,
            'asset_verification': extra.pop('asset_verification', 'unknown'),
            'controller': controller_label(extra),
        },
        'seeds': {
            'imu_noise': params['imu_noise']['ros__parameters']['seed'],
            'range_noise': params['range_sensor']['ros__parameters']['seed'],
            'note': 'noise per sample is keyed by (seed, sample timestamp); Gazebo adds no noise',
        },
        'parameters_file': 'x3_scenario.yaml',
        'parameters': params,
        'run': extra,
        'recording': {'bag': 'bag/', 'storage': 'mcap', 'clock': 'sim time (recorded with --use-sim-time)'},
    }
    (run_dir / 'manifest.yaml').write_text(yaml.safe_dump(manifest, sort_keys=False))


def finish(run_dir, extra):
    path = run_dir / 'manifest.yaml'
    manifest = yaml.safe_load(path.read_text())
    result_file = run_dir / 'scenario_result.json'
    result = json.loads(result_file.read_text()) if result_file.exists() else {}
    bag = run_dir / 'bag'
    extra['scenario_status'] = result.get('status', 'no result')
    extra['bag_bytes'] = sum(p.stat().st_size for p in bag.glob('*')) if bag.exists() else 0
    extra['finished_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds')
    manifest['outcome'] = extra
    path.write_text(yaml.safe_dump(manifest, sort_keys=False))


def main():
    if len(sys.argv) < 3 or sys.argv[1] not in ('start', 'finish'):
        print(__doc__)
        return 2
    run_dir = Path(sys.argv[2])
    extra = dict(a.split('=', 1) for a in sys.argv[3:])
    (start if sys.argv[1] == 'start' else finish)(run_dir, extra)
    return 0


if __name__ == '__main__':
    sys.exit(main())
