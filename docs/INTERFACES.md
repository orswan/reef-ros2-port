# Interfaces

Current contracts for the project's command line, simulation topics, and
(planned) REEF estimator. **Implemented** means exercised by a check in this
repository; **PLANNED** or **NOT DEFINED** means no ROS 2 implementation exists.

## 1. Command interface

Container terminal, from `/root/ros2_ws/reef_ros2`. Both wrappers set up
their own environment (`scripts/env.sh`); nothing needs sourcing first.

```bash
scripts/reef_check.sh help
scripts/reef_check.sh env
scripts/reef_check.sh clock [--gui] [--regress]
scripts/reef_check.sh sim-data [--gui] [--regress]
scripts/reef_check.sh baseline [--floor]
scripts/reef_check.sh interfaces
scripts/reef_check.sh estimator
scripts/reef_demo.sh help
scripts/reef_demo.sh stock [--gui]
scripts/reef_demo.sh replay recordings/<run> [--rate R]
scripts/reef_demo.sh estimator [--gui]
scripts/reef_demo.sh estimator --offline recordings/<run>
scripts/reef_demo.sh estimator --replay recordings/<run> [--rate R]
```

Not yet implemented (each says NOT IMPLEMENTED and exits 2):
`reef_check.sh faults|control|vision|release` and
`reef_demo.sh closed-loop|vision`. `reef_check.sh estimator` and
`reef_demo.sh estimator` cover the **vertical** filter only; horizontal
coverage is reported as NOT IMPLEMENTED (milestone P05) and never counted. All demo modes are **simulation
only**; no mode can reach hardware.

### Exit status (wrapper boundary)

| Code | Meaning |
|---|---|
| 0 | PASS: every executed assertion passed (demo: completed) |
| 1 | FAIL: a check executed and failed |
| 2 | BLOCKED (prerequisite missing: assets, display), NOT IMPLEMENTED, or invalid invocation. Nothing was judged |
| 130 / 143 | interrupted by SIGINT / SIGTERM. The signal is forwarded to the running check, which cleans up its own processes |

The wrapped scripts keep their existing codes, which are mapped here:

| Script | Its codes | Wrapper result |
|---|---|---|
| `check_clock_demo.sh` | 0 pass, 1 clock check failed, 2 owned demo failed or bad input, 124 timeout | 0 → PASS; 1, 2, 124 → FAIL (prerequisites are checked by the wrapper first, so 2 here is an executed failure) |
| `run_x3_scenario.sh` | 0 pass, 1 analysis failed, 2 setup or owned simulation failed, 3 scenario runner failed, 124 timeout | 0 → PASS; others → FAIL (assets and display are prechecked; a missing one → BLOCKED 2) |
| `clock_check.py` (negative case) | 1 = no clock, as required | PASS only if it exits exactly 1 |
| `regress_*.sh` | 0 all cases matched, 1 otherwise | 0 → PASS, 1 → FAIL |
| `baseline/tools/check_baseline.py` | 0 all assertions passed, 1 a check failed, 2 sources unavailable | 0 → PASS, 1 → FAIL (sources are prechecked → BLOCKED 2) |
| `scripts/check_vendor.py`, `scripts/check_colcon.py`, `baseline/helper_vectors.sh` | 0 pass, 1 mismatch/failure, 2 invalid environment | 0 → PASS; the tampered-copy negative passes only on exactly 1 |

### Report

Each `reef_check.sh` run ends with:
- **source:** commit and branch, and the number of uncommitted or untracked paths.
- **environment:** dev image package-list hash or "not the dev image", and the gz version.
- **config:** SHA-256 of the configuration and input files of the target.
- **assertions:** each one with PASS/FAIL, its actual and expected exit code, and wall time.
- **sim time:** where relevant, the sim-time span observed or the scenario end time.
- **wall time**, and **artifact** paths: logs in `log/checks/reef_check_<target>_<time>/`,
  recordings in `recordings/`.

## 2. Simulation interfaces (implemented)

### Clock demo (`sim/`)

`/clock` (`rosgraph_msgs/Clock`), bridged from Gazebo. `check_clock_demo.sh`
remaps it to a per-run topic (`/reef_clock_check_<token>/clock`), with a
per-run `GZ_PARTITION` and ROS domain.

### X3 scenario (`src/reef_sim`)

The full specification is in [X3_SCENARIO.md](X3_SCENARIO.md). Summary:

| Topic | Type | Category | Frame | Rate | Time basis |
|---|---|---|---|---|---|
| `/x3/truth/odom` | `nav_msgs/Odometry` | TRUTH (twist: smoothed finite difference) | `world` (ENU) → `x3/base_link` (FLU) | 100 Hz | sim time, header stamp = Gazebo sample time |
| `/x3/imu` | `sensor_msgs/Imu` | measurement; specific force (+g on z at rest); orientation not provided | `x3/base_link` (FLU) | 250 Hz | sim time |
| `/x3/range` | `sensor_msgs/Range` | idealized measurement derived from truth; slant range; REP 117 ±inf | `x3/range_link` | 20 Hz | sim time (the truth sample it was computed from) |
| `/x3/cmd_vel` | `geometry_msgs/Twist` | command to the truth-fed stock controller | body FLU | 20 Hz | sim time |
| `/x3/scenario/phase` | `std_msgs/String` | phase label (transient local) | — | per phase | recorder receive time (sim) |

**Freshness:** no staleness detection is implemented on these topics. Consumers
must compare header stamps with the sim clock themselves. For REEF inputs,
see §3.4 (timestamps and freshness) and §3.7 (health).

**Attitude and yaw sources:** the only attitude or yaw in the simulation is
the **truth** orientation in `/x3/truth/odom`, which is **idealized**. No
attitude estimator is implemented. The IMU deliberately does not carry
orientation.

## 3. REEF estimator node interface (vertical filter implemented in P04)

This is the contract for the ROS 2 `reef_estimator` node
(`reef_estimator_node`, class `SensorManager`). P03 implemented the
messages, the parameter contract, and the configuration files; P04
implemented the node with the **vertical** filter. The horizontal filter is
not ported yet: its inputs are not subscribed and its output fields are
**NaN** (§3.3). Everything marked "horizontal" below is PLANNED (P05). Behaviour is
that of master `e4179f48` ([BASELINE_DECISION.md §4](BASELINE_DECISION.md#4-specification-of-the-baseline-master-e4179f48));
corrections C1–C5 are deferred to R1. Differences from ROS 1 are limited to
middleware and are listed in §3.9.

### 3.1 What REEF estimates, and what it does not

REEF estimates **altitude, vertical velocity, horizontal velocity in the
body-level frame, and sensor biases**. It does **not** estimate horizontal
position, attitude, or yaw. Attitude is an **input** (the IMU orientation).
The outputs are REEF's own messages, which contain only estimated
quantities, plus a few legacy fields that are never set (§3.3). No standard
pose or odometry message is published. If a later milestone adds one (for
example for a controller), fields that REEF does not estimate must not be
filled with fabricated values: use a message whose fields are all estimated
(for example `TwistWithCovarianceStamped` for the body-level velocity), or
mark the missing parts explicitly and document it here.

### 3.2 Frames and units

| Name | Definition |
|---|---|
| NED | local North-East-Down world frame; z positive **down** |
| body (FRD) | Forward-Right-Down vehicle frame |
| body-level | NED rotated by the vehicle yaw ψ: x forward and y right in the horizontal plane, z down |
| C | `quaternion_to_rotation(imu.orientation)` = C_NED→body (transpose of the standard R(q)) |

All quantities are SI: m, m/s, m/s², rad, s. No TF is used or published.
Input `frame_id` values are **not checked** (as in ROS 1); inputs must
already be in the frames listed in §3.4.

### 3.3 Outputs (implemented, P04; horizontal fields NaN until P05)

| Topic | Type | When | QoS |
|---|---|---|---|
| `xyz_estimate` | `reef_msgs/XYZEstimate` | after every IMU propagation (from the 21st IMU message on, §3.7) | reliable, transient local, keep last 1 (was latched, queue 1) |
| `xyz_debug_estimate` | `reef_msgs/XYZDebugEstimate` | same, only if `debug_mode` | same |
| `is_flying_reef` | `std_msgs/Bool` | **only on takeoff and landing transitions** (nothing is published before the first takeoff) | same |
| `sonar_ned` | `sensor_msgs/Range` | every range message, only if `debug_mode` and `enable_sonar`; `range` negated, other fields copied | reliable, volatile, keep last 1 |

Output fields (see the comments in `src/reef_msgs/msg/*.msg`):

| Field | Meaning |
|---|---|
| `header.stamp` | stamp of the IMU message that triggered the step (sensor time; sim time in simulation) |
| `header.frame_id` | **empty**: the message mixes two frames (NED vertical, body-level horizontal) |
| `node_id` | not set (0) |
| `xy_plus.x_dot`, `y_dot` | horizontal velocity, body-level frame [m/s], after the update of this step. **P04: NaN** (not estimated; the horizontal filter is not ported) |
| `z_plus.z` | vertical position, NED [m], relative to the altitude reference: the ground below the range sensor, or the mocap origin when mocap z is used |
| `z_plus.z_dot` | vertical velocity, NED [m/s] (positive down) |
| debug `*_minus` / `*_plus` | state after propagation / after the update of the same step |
| debug `xy_*` | [x_dot, y_dot, pitch_bias, roll_bias, xa_bias, ya_bias] in m/s, rad, m/s²; `sigma_plus/minus` = state ± 3·sqrt(diag P). **P04: all NaN** |
| debug `z_*` | `z`, `z_dot`, `bias` (a = u − b, m/s²), `u` (vertical specific force in NED + 9.81, m/s²), `p` = covariance of [z, z_dot, bias], **row-major 3×3**; `sigma_plus/minus` as above |
| debug `z_*.truth`, `z_error`, `z_dot_error` | **not set (0)**; not measurements and not truth |

`XYZEstimate` carries no covariance; only the debug message does.

### 3.4 Inputs (vertical inputs implemented, P04)

The legacy topic names are kept (relative names, remappable). A topic is
subscribed only if its enable parameter is true.

| Topic (parameter) | Type | Used fields and convention | Subscribed if | QoS |
|---|---|---|---|---|
| `imu/data` | `sensor_msgs/Imu` | `linear_acceleration`: specific force in **body FRD** [m/s²] (level and at rest ≈ (0, 0, −9.81)); `orientation`: attitude of the body in NED (Hamilton x, y, z, w; not normalized by REEF); `header.stamp`: defines dt. Angular velocity and all covariances are ignored | always | best effort, volatile, keep last 10 (ROS 1 queue 10) |
| `sonar` | `sensor_msgs/Range` | `range` [m], used as vertical distance (no tilt compensation, D/C5); accepted only if `range <= max_range`. `min_range`, field of view, and stamp are ignored | `enable_sonar` | best effort, keep last 1 |
| `mocap_ned` (`mocap_pose_topic`) | `geometry_msgs/PoseStamped` | `pose.position.z` only, NED [m] | `enable_mocap_z` | best effort, keep last 1 |
| `mocap_velocity/body_level_frame` (`mocap_twist_topic`) | `geometry_msgs/TwistWithCovarianceStamped` | `twist.twist.linear.x/.y` [m/s], body-level; `twist.covariance[0]`, `[7]` = variances of x and y (row-major 6×6); off-diagonal terms ignored | `enable_mocap_xy` (**P04: never subscribed**, horizontal) | best effort, keep last 1 |
| `rgbd_velocity_body_frame` (`rgbd_twist_topic`) | `reef_msgs/DeltaToVel` | `vel.twist.twist.linear.x/.y` and `vel.twist.covariance[0]`, `[7]` as for mocap; other fields unused | `enable_rgbd` (**P04: never subscribed**, horizontal) | best effort, keep last 1 |
| `rc_raw` | `rosflight_msgs/RCRaw` (upstream v2.0.1) | `values[mocap_override_channel]`, PWM µs (§3.5) | `enable_mocap_switch` | best effort, keep last 1 |

Best effort matches publishers of either reliability. QoS can be changed at
launch through the standard ROS 2 QoS-override parameters
(`qos_overrides.<topic>.subscription.reliability` etc.; every subscription
enables `QosOverridingOptions`).

**Timestamps and freshness (clock policy).** Only IMU stamps are used: dt is
the difference of consecutive IMU stamps, each converted with ROS 1's
`toSec()` = sec + 1e−9 · nanosec (so dt is not always exactly the nominal
period). There is no minimum, maximum, or monotonicity check, as in master:
duplicate, backward, and late stamps are processed as the original did
(fixtures `v05`–`v07`, bit-identical to the reference). The node logs such
steps (throttled) and counts them, without changing the numbers. The node
clock and `use_sim_time` never enter the estimate; they only matter for the
backward-jump reset (§3.7). Measurement stamps are ignored. A measurement is gated when it
arrives and fused at the next IMU propagation, so its latency is not
compensated, and within one IMU period the last accepted measurement of each
kind wins. There is no staleness detection: when a measurement stream stops,
the filter continues on IMU propagation alone.

**Simulation mapping (implemented, P04).** `reef_sim/reef_adapter` publishes
`/x3/reef/imu/data` (the `/x3/imu` measurement converted FLU → FRD, with the
**truth** attitude slerped to each IMU stamp, frame `x3/base_link_frd`) and
`/x3/reef/sonar` (`/x3/range` unchanged, including REP 117 ±inf: master's
`range <= max_range` test drops +inf and NaN, and its χ² gate rejects −inf).
It publishes `/x3/reef/input_labels` (transient local) saying which inputs
are idealized. The estimator runs in namespace `/x3/reef`, so its topics are
`/x3/reef/xyz_estimate` etc. No tilt compensation is applied (as in master;
C5 deferred). See [X3_SCENARIO.md §11](X3_SCENARIO.md).

### 3.5 Measurement selection and the RC switch

At startup: `useMocapXY = enable_mocap_xy && !enable_rgbd` and
`useMocapZ = enable_mocap_z && !enable_sonar`. Then:

| Input | Used when |
|---|---|
| RGB-D velocity | `!useMocapXY`, the parameter `enable_measurements` is true (read at every message, runtime-settable), and the χ² gate accepts |
| mocap velocity | `useMocapXY` and the gate accepts |
| range | `!useMocapZ`, `range <= max_range`, and the gate accepts |
| mocap z | `useMocapZ` and the gate accepts |

**RC switch** (`enable_mocap_switch`, legacy default false; `true` in the
shipped hardware file; **false, explicitly, in `config/simulation.yaml`**).
`rosflight_msgs/RCRaw` has `uint16 values[8]`, the receiver PWM pulse width in
µs; index i is receiver channel i + 1 (both in ROS 1 rosflight `44e5f37e`,
where rosflight_io copies MAVLink `RC_CHANNELS_RAW.chanN_raw`, and in upstream
v2.0.1, which copies `RC_CHANNELS.chanN_raw`). `mocap_override_channel`
(default 4, shipped 6) indexes this array and must be 0–7. Rules, edge
triggered with an internal state that starts "off":

- off and `values[ch] > 1500`: `useMocapXY = true` if `enable_mocap_xy`,
  `useMocapZ = true` if `enable_mocap_z`; state on.
- on and `values[ch] <= 1500`: `useMocapXY = false` if `enable_rgbd`,
  `useMocapZ = false` if `enable_sonar`; state off.

**Missing RC.** With the switch enabled and no `rc_raw` message, the
selection stays at its startup value. If RC messages stop, the last
selection is kept: there is no timeout or failsafe. A default-constructed
message (all 0) reads as "low". MAVLink marks an unused channel as
`UINT16_MAX`, which reads as "high"; check this before enabling the switch
on hardware. Launch files that start `rosflight_io` belong to the separate
hardware integration path (P10+); simulation publishes no `rc_raw`.

### 3.6 Parameters

Implemented in `src/reef_estimator` (P03): declared with descriptions,
read-only except `enable_measurements`, validated at startup. Any invalid
value stops the node with a list of every problem (`reef_msgs::ParameterError`).
Defaults are the legacy code defaults; shipped values are in
`config/estimator_master.yaml`.

| Name | Type | Default | Meaning and validation |
|---|---|---|---|
| `debug_mode` | bool | false | publish `xyz_debug_estimate`, `sonar_ned` |
| `enable_xy`, `enable_z` | bool | true | landing reset and updates of the horizontal / vertical filter (propagation always runs) |
| `enable_mocap_xy`, `enable_rgbd`, `enable_mocap_z`, `enable_sonar` | bool | true | subscriptions and selection (§3.5) |
| `enable_partial_update` | bool | true | β-weighted partial updates instead of full updates |
| `enable_mocap_switch` | bool | false | RC switch (§3.5) |
| `mocap_override_channel` | integer | 4 | 0–7 (descriptor range); a double such as `6.0` is rejected (roscpp silently used the default) |
| `enable_measurements` | bool | true | runtime-settable RGB-D switch |
| `mahalanobis_d_sonar`, `_rgbd_velocity`, `_mocap_z`, `_mocap_velocity` | double | 20 | gate on the squared Mahalanobis distance; > 0, `+inf` disables the gate, NaN rejected; integers accepted |
| `estimator_dt` | double | 0.002 | nominal IMU period [s]: first dt and `xy_Q · dt²`; finite, > 0 |
| `mocap_twist_topic`, `mocap_pose_topic`, `rgbd_twist_topic` | string | legacy names (§3.4) | non-empty |
| `xy_x0` (6×1), `z_x0` (3×1) | double list | **required** | initial states; finite |
| `xy_P0`, `xy_Q` (6×6), `xy_R0` (2×2), `z_P0`, `z_P0_flying` (3×3), `z_Q` (2×2), `z_R0`, `z_R_flying` (1×1) | double list | **required** | covariances: rows·cols values row-major, or n values = diagonal of an n×n matrix; finite, exactly symmetric, diagonal ≥ 0 |
| `xy_beta` (6×1), `z_beta` (3×1) | double list | **required** | partial-update weights in [0, 1] |

Matrix lists may be integer lists (converted exactly). Missing, empty,
wrong-length, non-finite, or wrongly typed values are errors. In ROS 1 a
missing matrix became zeros and a wrong-length one stayed uninitialized (D10).
The original YAML files cannot be loaded by ROS 2 as they are: they lack the
`ros__parameters` structure, and their lists mix integers and floats, which
ROS 2 rejects (characterized in `test_legacy_params.py`).
`use_sim_time` is the standard ROS 2 parameter; it does not affect the
estimate, which uses message stamps only.

### 3.7 Health, initialization, and reset

There is no health topic (none in ROS 1; a diagnostics output may be
proposed after R1). Observable behaviour, as in master, plus two ROS 2
additions (reset service, backward-jump reset) that never run unless
triggered:

| Condition | Behaviour |
|---|---|
| first 20 IMU messages | accelerometer initialization; **no output** (gravity is then fixed at 9.81, D/C3) |
| IMU with NaN acceleration | dropped with an error log; the next dt spans the gap (D9). Orientation is not checked |
| not flying | every 10 propagations a landing reset (if `enable_xy` / `enable_z`): horizontal P = `xy_P0`, state = `xy_x0`; vertical P = `z_P0` and z_dot = `z_x0[1]` (z and bias are kept) |
| takeoff / landing | detected from accelerometer variance and the altitude measurement ([BASELINE_DECISION.md §4.4](BASELINE_DECISION.md#44-initialization)); announced on `is_flying_reef`. Takeoff sets the vertical filter's R = `z_R_flying`, P = `z_P0_flying`; landing sets R = `z_R0` and resets its state to `z_x0`, `z_P0` |
| measurement streams stop | no detection; propagation continues |
| invalid parameters | node exits with status 1 before publishing, naming every invalid parameter (§3.6) |
| `~/reset` (`std_srvs/Trigger`) | returns the estimator to its startup state; the following outputs equal those of a fresh node (tested). If it was flying, `is_flying_reef` publishes `false` |
| ROS time jumps backwards (simulation reset, replay restarted) | the time source flags a reset from its own thread; the next message callback applies it on the executor thread, before that message is processed. Stamps already queued with the old time can be processed first; the ordering of `/clock` against data topics is not defined in ROS 2 |
| launch / shutdown | `ros2 launch reef_estimator reef_estimator.launch.py [overrides_file:=…/simulation.yaml]`; SIGINT exits 0 (launch test) |

### 3.8 Execution model

- **Core without ROS (implemented).** `VerticalEstimator`
  (`reef_estimator_core`) holds the vertical filter and the measurement
  logic. It takes plain structs (IMU sample, range, mocap pose, RC values)
  with the ROS 1 field types and exposes its state; it has no ROS types,
  clocks, or threads. `reef_estimator_event_replay` drives it from event
  files, and `--mode node` drives the ROS node's callbacks with the same
  events as ROS 2 messages (wrapper equivalence).
- **Node.** A single-threaded executor with all subscriptions in one
  mutually exclusive callback group, so callbacks never overlap and all state
  changes happen on one thread, in the order the executor delivers messages.
  No timers: output is driven by IMU messages.
- **Input order.** Per topic, messages arrive in publication order. Across
  topics, the live node processes them in executor order, which is not
  guaranteed to be stamp order (the same class of nondeterminism as the ROS 1
  callback queue). No reorder buffer is added: it would add latency that
  ROS 1 did not have. Because measurements are fused only at the next IMU
  step (§3.4), cross-topic order affects only which state a gate sees and
  which of several measurements in one IMU period is used.
- **Fidelity tests** call the core directly with the fixture events in their
  recorded order (stamp, then a fixed topic order), exactly as the P02
  harness drives the original, so the comparison with the golden output does
  not depend on scheduling.

### 3.9 Differences from ROS 1 (middleware only)

| ROS 1 | ROS 2 |
|---|---|
| `Header` with `seq` | `std_msgs/Header` (no `seq`) |
| `DeltaToVel.S_upper_bound`, `S_lower_bound`, `ZDebugEstimate.P` | `s_upper_bound`, `s_lower_bound`, `p` (ROS 2 naming rule) |
| latched publishers | reliable + transient local, depth 1 |
| private parameters, silent zero/default on bad values | node parameters with descriptors; invalid values stop the node |
| ROS 1 `rosflight_msgs` (`44e5f37e`) | upstream ROS 2 `rosflight_msgs` v2.0.1, vendored unmodified; same `RCRaw` layout |
| callback queue with one spinner | single-threaded executor, one mutually exclusive group |
| (P04) horizontal fields always estimated | NaN until the horizontal filter is ported |
| (P04) no reset | `~/reset` service; reset on a backward ROS time jump |
| (P04) range/mocap rejection logged at every message | throttled to 1 Hz; stamp anomalies logged (throttled) and counted |
