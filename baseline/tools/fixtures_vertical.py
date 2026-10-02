#!/usr/bin/env python3
"""Vertical-estimator fixtures for the P04 port comparison (v01-v10).

    fixtures_vertical.py OUT_DIR

Same conventions and flight model as fixtures.py (P02). These add the vertical
cases the P02 set does not cover: descent and landing, missing and invalid
range data, repeated measurements, IMU timestamp anomalies, a tilted descent,
and the vertical filter disabled. Each stream is fed unchanged to the
reference harness and to the port. Deterministic: no randomness.

Timestamp anomalies are applied after the events are sorted, so an event
with an earlier stamp stays at its position in the stream (as a late message
would arrive).
"""
import hashlib
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import fixtures as f  # noqa: E402  (imported after the sys.path insert above)

H = f.T_HOVER
SONAR_DT = 0.02


def on_sonar_tick(t):
    return abs(t / SONAR_DT - round(t / SONAR_DT)) < 1e-6


def near(t, t0):
    return abs(t - t0) < 1e-9


def write(s, out, edit=None):
    """Scenario.write with an optional edit of the sorted event list."""
    s.events.sort(key=lambda e: (e[0], e[1]))
    lines = [e[2] for e in s.events]
    if edit:
        lines = edit(lines)
    (out / f'{s.name}.events').write_text(f'# {s.name}: {s.doc}\n' + '\n'.join(lines) + '\n')
    (out / f'{s.name}.truth.csv').write_text(
        't_ns,h_up,v_up,vx,vy,roll,pitch\n' + '\n'.join(','.join(repr(x) for x in r) for r in s.truth) + '\n')


def imu_index(lines, t):
    """Index of the IMU event at time t (seconds after T0)."""
    ns = f.T0 + round(t * 1e9)
    for i, line in enumerate(lines):
        if line.startswith('imu ') and int(line.split()[1]) == ns:
            return i
    raise KeyError(t)


def restamp(line, ns):
    parts = line.split()
    parts[1] = str(ns)
    return ' '.join(parts)


DESCENT = [(2.0, 0.0, 0.0), (1.0, -0.5, 0.0), (1.8, 0.0, 0.0), (1.0, 0.5, 0.0), (3.0, 0.0, 0.0)]


def scenarios():
    out = []   # (scenario, edit, param_set)

    out.append((f.flight('v01_descent_landing',
                         'takeoff, hover 2 s, descend 1.40 m at up to 0.5 m/s to the ground (range 0.12 m), 3 s on the ground',
                         DESCENT), None, 'default'))

    out.append((f.flight('v02_sonar_dropout', 'hover; no range messages from T_HOVER+1 s to T_HOVER+3 s',
                         [(6.0, 0.0, 0.0)],
                         sonar_override=lambda t, r: None if H + 1.0 <= t < H + 3.0 else r), None, 'default'))

    def invalid(t, r):
        for dt_, val in ((1.0, 8.0), (1.5, float('nan')), (2.0, 0.0), (2.5, -0.5), (3.0, f.MAX_RANGE)):
            if near(t, H + dt_):
                return val
        return r
    out.append((f.flight('v03_range_invalid',
                         'hover; range 8.0 (> max_range), NaN, 0.0, -0.5, and exactly max_range at T_HOVER+1/1.5/2/2.5/3 s',
                         [(5.0, 0.0, 0.0)], sonar_override=invalid), None, 'default'))

    def repeated(s, t, tr):
        if t >= H and on_sonar_tick(t):
            s.range(t, tr[0])                           # exact duplicate (same stamp and value)
            if H + 1.0 <= t < H + 2.0:
                s.range(t + 0.001, tr[0] + 0.05)        # a second, different range before the next IMU
    out.append((f.flight('v04_repeated_range',
                         'hover; every range duplicated from T_HOVER; from T_HOVER+1 s to +2 s a second range (+0.05 m) 1 ms later',
                         [(4.0, 0.0, 0.0)], mocap=repeated), None, 'default'))

    def duplicate_imu(lines):
        i = imu_index(lines, H + 1.0)
        return lines[:i + 1] + [lines[i]] + lines[i + 1:]
    out.append((f.flight('v05_imu_duplicate_stamp', 'hover; the IMU message at T_HOVER+1 s is delivered twice (dt = 0)',
                         [(3.0, 0.0, 0.0)]), duplicate_imu, 'default'))

    def backward_imu(lines):
        i = imu_index(lines, H + 1.0)
        late = restamp(lines[i], f.T0 + round((H + 1.0 - 0.003) * 1e9))
        return lines[:i + 1] + [late] + lines[i + 1:]
    out.append((f.flight('v06_imu_backward_stamp',
                         'hover; after the IMU at T_HOVER+1 s, an IMU stamped 3 ms earlier (negative dt, then dt = 5 ms)',
                         [(3.0, 0.0, 0.0)]), backward_imu, 'default'))

    def imu_gap(lines):
        lo, hi = f.T0 + round((H + 1.0) * 1e9), f.T0 + round((H + 1.5) * 1e9)
        return [ln for ln in lines if not (ln.startswith('imu ') and lo <= int(ln.split()[1]) < hi)]
    out.append((f.flight('v07_imu_gap', 'hover; no IMU from T_HOVER+1 s to T_HOVER+1.5 s while ranges continue (dt = 0.502 s)',
                         [(4.0, 0.0, 0.0)]), imu_gap, 'default'))

    def mocap_z(s, t, tr):
        z = -tr[0]
        if near(t, H + 1.5):
            z -= 1.0                                    # outlier
        s.mocap_pose(t, z)
        if H + 1.0 <= t < H + 2.0:
            s.mocap_pose(t, z)                          # duplicate
        if near(t, H + 2.5):
            s.mocap_pose(t, z - 1e-9)                   # differs from z only below float32 resolution
    out.append((f.flight('v08_mocap_z_repeated_outlier',
                         'mocap z (no sonar); duplicates from T_HOVER+1 s to +2 s, a 1 m outlier at +1.5 s, a sub-float32 change at +2.5 s',
                         [(4.0, 0.0, 0.0)], sonar=False, mocap=mocap_z), None, 'mocap_z'))

    out.append((f.flight('v09_tilt_descent',
                         'hover, then roll 15 deg while descending 0.9 m (slant range, no tilt compensation)',
                         [(1.0, 0.0, 0.0), (1.0, -0.5, 0.0), (0.8, 0.0, 0.0), (1.0, 0.5, 0.0), (2.0, 0.0, 0.0)],
                         roll=lambda t: math.radians(15) if t >= H + 1.0 else 0.0), None, 'default'))

    out.append((f.flight('v10_z_disabled', 'takeoff and hover with enable_z = false (propagation only, no updates)',
                         [(4.0, 0.0, 0.0)]), None, 'z_disabled'))
    return out


def node_settings(set_name):
    if set_name == 'z_disabled':
        return {**f.node_settings('default'), 'enable_z': False}
    return f.node_settings(set_name)


def main():
    out = Path(sys.argv[1])
    (out / 'params').mkdir(parents=True, exist_ok=True)
    master = f.shipped_params('master')
    param_set = {}
    for s, edit, set_name in scenarios():
        write(s, out, edit)
        param_set[s.name] = set_name
    for set_name in sorted(set(param_set.values())):
        f.write_params(out / 'params' / f'common_{set_name}_master.params', {**master, **node_settings(set_name)})
    files = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest()
             for p in sorted(out.rglob('*')) if p.is_file() and p.name != 'manifest.json'}
    (out / 'manifest.json').write_text(json.dumps({'param_set_for_scenario': param_set, 'files': files}, indent=2))
    print(f'wrote {len(param_set)} vertical scenarios to {out}')


if __name__ == '__main__':
    main()
