# R1 review packet: REEF estimator port (P02–P05)

Prepared 2026-09-30 on branch `p05-horizontal-estimator`. Purpose: let an
independent reviewer confirm that the ROS 2 port reproduces the selected
original estimator, and decide on the correction candidates C1–C6. Every
claim below links to its evidence; commands are in §7. Container commands
run from `/root/ros2_ws/reef_ros2` in either project container.

## 1. Baseline decision (what the port must reproduce)

| Item | Value | Where |
|---|---|---|
| Selected original | `reef_estimator` master `e4179f48c3f26e22bd1366b71ee1e117ce2f5f7f` (2021-03-08) with `reef_msgs` `7fb63ff9` | [BASELINE_DECISION.md §1–3](../BASELINE_DECISION.md) |
| Comparison reference kept | simulation branch `95987b51` (used only as a negative control) | same |
| Specification of the baseline | equations, order of operations, gates, float32 conversions, defaults, with file:line references | BASELINE_DECISION §4 |
| Legacy defects (preserved) | D1–D10; D1 (XY observation re-fused after a partial update) matters most; D10 (invalid parameters) is the only one not preserved (the port rejects invalid parameters) | BASELINE_DECISION §6–7 |
| Corrections | none approved. C1–C6 deferred to R1 by the USER (C6, the takeoff detector threshold, added after P04). C1 is implemented **opt-in, default off** so it can be evaluated | BASELINE_DECISION §7 |
| Scenario assumption | the simulation adds 1.0 m/s² IMU vibration so master's takeoff detector can fire (USER: keep; detector = C6) | ACCEPTANCE §4c |

## 2. Source diff and approved deviations

`baseline/tools/source_diff.sh` writes the unified diff of every ported file
against the original imported by `git subtree` (reef_estimator `c80f824` =
master minus a third-party PDF; reef_msgs `d8e3096` = `7fb63ff`). Summary
(lines added / removed):

| File | + | − | Nature of the change |
|---|---|---|---|
| `estimator.h/.cpp`, `z_estimator.h/.cpp`, `xy_estimator.h/.cpp` | 11 | 16 | include lines only; **filter math unchanged** |
| `xyz_estimator.h/.cpp` | 380 | 369 | ROS types → plain structs with the ROS 1 field types; ROS parameters → `EstimatorParameters`; publishing moved to the node; added counters and opt-in C1; statement order, types, and Eigen expressions kept |
| `sensor_manager.h/.cpp`, `reef_estimator_node.cpp` | 357 | 132 | rclcpp node: subscriptions, publishers, QoS, reset service, jump reset, diagnostics |
| `reef_msgs` helpers | 135 | 274 | unused functions removed; bodies of the used ones unchanged; parameter import split into a ROS-free core and an rclcpp adapter with validation |
| `reef_msgs` messages | 48 | 13 | `Header` → `std_msgs/Header`; three renames forced by ROS 2 naming (`S_upper_bound`, `S_lower_bound`, `P`); comments |

Approved middleware deviations (no effect on any estimate for the same
ordered inputs): BASELINE_DECISION §7 and [INTERFACES.md §3.9](../INTERFACES.md).

## 3. Interface table

| Topic (relative, node `reef_estimator`) | Type | Direction | QoS | Notes |
|---|---|---|---|---|
| `imu/data` | `sensor_msgs/Imu` | in | best effort, depth 10 | FRD specific force; orientation = attitude of body in NED (**external attitude input**; truth in simulation) |
| `sonar` | `sensor_msgs/Range` | in (`enable_sonar`) | best effort, 1 | vertical distance, no tilt compensation (C5) |
| `mocap_ned` | `geometry_msgs/PoseStamped` | in (`enable_mocap_z`) | best effort, 1 | z only, NED |
| `mocap_velocity/body_level_frame` | `geometry_msgs/TwistWithCovarianceStamped` | in (`enable_mocap_xy`) | best effort, 1 | body-level vx, vy; covariance[0], [7] |
| `rgbd_velocity_body_frame` | `reef_msgs/DeltaToVel` | in (`enable_rgbd`, `enable_measurements`) | best effort, 1 | vel field only |
| `rc_raw` | `rosflight_msgs/RCRaw` (upstream v2.0.1) | in (`enable_mocap_switch`) | best effort, 1 | mocap/RGB-D and mocap/sonar switch |
| `xyz_estimate`, `xyz_debug_estimate` | `reef_msgs/*` | out | reliable, transient local, 1 | stamp = IMU stamp; frame_id empty |
| `is_flying_reef` | `std_msgs/Bool` | out | reliable, transient local, 1 | on transitions |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | out | reliable, 10 | timing, observation accounting |
| `~/reset` | `std_srvs/Trigger` | service | — | startup state |

Full contract: [INTERFACES.md §3](../INTERFACES.md); ROS 2 status of the
optional producers (mocap velocity, mocap pose, RGB-D): §3.10 (none ported;
types compatible; RGB-D **not exercised**).

## 4. Fixture provenance

| Set | Generator | Lock | Content |
|---|---|---|---|
| P02 s01–s15 | `baseline/tools/fixtures.py` | `baseline/fixtures.lock.json` | deterministic flights, gates, NaN, outliers, mocap, RC, full update, airborne start |
| P04 v01–v10 | `baseline/tools/fixtures_vertical.py` | `baseline/fixtures_vertical.lock.json` | descent/landing, dropouts, invalid and repeated ranges, IMU timestamp anomalies, tilted descent, z disabled |
| P05 h01–h10 | `baseline/tools/fixtures_horizontal.py` | `baseline/fixtures_horizontal.lock.json` | nonzero yaw (30°, −135°, 90°, 45°, −60°, 20°, 10°, 75°) with tilt, outliers, duplicates, out-of-order, dropout, RC both ways, RGB-D full update, landing reset, `enable_xy` false, RGB-D ignored |
| Legacy helper vectors | `baseline/helper_vectors.sh` | committed file, byte-identical on regeneration | reef_msgs helpers |
| Golden | `baseline/tools/check_baseline.py --update-golden` (P02, from the unmodified original) | `baseline/golden/index.json` | decimated reference outputs |
| Recorded simulation stream | `x3_reef_offline` of an `--estimator` run | regenerated per run | IMU/range/velocity events from the X3 simulation |

The reference harness compiles the pinned originals unmodified
(checksums in `baseline/provenance.json`; adaptations A1–A5 in
`baseline/README.md`).

## 5. Comparison results

Numerical parity (`reef_check.sh baseline`, `check_port.py`):

| Result | Value |
|---|---|
| Streams / events | 50 fixture streams, 282,490 events |
| Values compared | 18,079,360 (every Z and XY state field, full covariances, measurements, R, u, dt, g) — **all bit-identical** |
| Discrete fields, gate values | identical |
| Golden anchor | 15/15 |
| Wrapper (ROS 2 node through callbacks) | 50/50: node state = core; published messages carry that state; fusion counts equal |
| Negatives | simulation revision 2.08e9 × tolerance; port with C1 4.87e8 × tolerance (both detected) |
| Recorded simulation stream | port = original, bit-identical (`reef_check.sh estimator`) |
| Optimization | port built -O2 (default) and -O0: both bit-identical |

Physical plausibility (`reef_check.sh estimator`; idealized inputs, REEF
not in the loop): altitude RMSE 6.8 mm, vertical velocity 0.037 m/s,
horizontal 0.011 m/s per axis, biases ≤ 0.007 rad / 0.011 m/s²,
covariances symmetric and PSD, 100 % within ±3σ (conservative), callback
p99 about 1.5 ms, estimate age p99 8–16 ms (run to run). See [reviews/P05.md](P05.md).

Faults (`reef_check.sh faults`): 35/36 pass. **F11 baseline fails**: after a
5 s velocity dropout during motion, master's D1 keeps the stale observation
fused, the variance stays small, and every later observation is rejected by
the gate (2629/2629); the velocity stays at the stale 0.36 m/s (error
0.51 m/s RMS). With C1 the filter recovers (0.015 m/s). See §6.

## 6. Decisions requested from R1

| ID | Question | Evidence |
|---|---|---|
| C1 | Approve clearing the XY flag after a partial update? | s07 (999 fusions of 1 observation), F1 (no variance growth during a dropout), **F11 (permanent lock-out after a dropout)**; opt-in implementation with regression tests |
| C2 | Sign convention of the Z bias in the XY filter | BASELINE_DECISION D2 |
| C3 | Measured gravity instead of 9.81 | s08 |
| C4 | Airborne initialization | s15 |
| C5 | Tilt-compensated range | s05, v09, X3 slant range |
| C6 | Takeoff detector threshold (vibration-dependent) | P04 characterization; simulation vibration assumption |
| — | Keep the simulation vibration assumption? | ACCEPTANCE §4c |

## 7. Reproduce (container)

```bash
scripts/reef_check.sh baseline      # reference (P02) + port parity (about 15 min)
scripts/reef_check.sh estimator     # tests, simulation vs truth, replays (about 12 min)
scripts/reef_check.sh faults        # F1-F12 (about 12 min)
baseline/tools/source_diff.sh       # build/r1/source_diff.patch and summary
T=$(python3 scripts/colcon_tree.py)
env -i HOME=$HOME PATH=/usr/bin:/bin bash --norc -c "source /opt/ros/jazzy/setup.bash && source $T/install/setup.bash && \
  python3 baseline/tools/check_port.py --port $T/install/reef_estimator/lib/reef_estimator/reef_estimator_event_replay"
```

Discriminating single runs (after `reef_check.sh baseline` built everything):

```bash
build/baseline/master/reef_ref build/baseline/fixtures_horizontal/params/common_default_master.params \
  build/baseline/fixtures_horizontal/h05_mocap_dropout.events /tmp/ref_h05.csv
$T/install/reef_estimator/lib/reef_estimator/reef_estimator_event_replay \
  build/baseline/fixtures_horizontal/params/common_default_master.params \
  build/baseline/fixtures_horizontal/h05_mocap_dropout.events /tmp/port_h05.csv
```
