# Acceptance criteria

Criteria are fixed **before** an implementation is scored. They change only
in a separate, explained commit. Lowering a threshold to make a failing result
pass is not allowed. Every criterion names its scenario, seed, units and frame,
time basis (simulation or wall), warm-up, scoring window, and limit.

Three different questions are kept apart:
- **Migration fidelity:** the ROS 2 code reproduces the selected original
  algorithm for the same ordered inputs.
- **Estimator quality:** estimates match the simulated physical state under
  the stated sensor errors.
- **Control quality:** the REEF feedback loop tracks bounded commands and
  handles failures as specified.

General rules:
- **No simulation result implies flight readiness.** Synthetic data, idealized
  sensors, and a truth-fed stock controller validate software behavior only.
- A negative test passes only when it observes the expected nonzero result of
  the rejected case. A process merely starting is never a pass.
- Exact message rates or real-time factors are not acceptance thresholds.
  Slower-than-real-time simulation can be valid. Rates count only as a
  plausibility floor (for example, at least 50 % of nominal).

## 1. `env` (implemented)

| Criterion | Limit |
|---|---|
| ROS distribution | `jazzy` |
| Gazebo | `gz sim` 8.x; `ros_gz_bridge` and `ros_gz_sim` installed |
| Environment hygiene | no `/root/ros2_ws/install` overlay on `AMENT_PREFIX_PATH`; `reference/COLCON_IGNORE` present |
| Display | X server answers on `$DISPLAY` within 5 s; in the dev container a window manager (Fluxbox) is present |

## 2. `clock` (implemented)

Scenario: `sim/worlds/clock_demo.sdf`, 1 ms step, a test-owned session with a
per-run `GZ_PARTITION`, ROS domain, and ROS topic.

| Criterion | Limit |
|---|---|
| Owned sim time reaches ROS 2 | at least 2 messages in a 5 s wall window; strictly increasing overall and monotonic; a `use_sim_time` node clock advances |
| Ownership | exactly one publisher on the per-run topic, named `clock_bridge`; required processes carry the run's `ROS_DOMAIN_ID` and `GZ_PARTITION` |
| Liveness | owned launch or required process exits → FAIL (not a timeout) |
| Negative: no simulator | observer exits 1 within 3 s |
| Regression (`--regress`) | 28/28: foreign simulation plus invalid display → FAIL; SIGINT/SIGTERM/timeout/mid-run failure → documented codes with 0 tagged survivors; long-lived display services unchanged |
| Bounded waits | first-message timeout 60 s; overall deadline startup + window + 15 s |

## 3. `sim-data` (implemented)

Scenario: [X3_SCENARIO.md](X3_SCENARIO.md), `src/reef_sim/config/x3_scenario.yaml`
(46 s of sim time; IMU seed 7, range seed 42). Limits and their rationale are in
[X3_SCENARIO.md §7](X3_SCENARIO.md#acceptance-criteria-and-why). In summary:

| Criterion | Limit |
|---|---|
| Streams present | ≥ 50 % of the nominal count per topic; phase labels, one per phase |
| Timestamps | monotonic, span ≥ 90 % of the scenario, p99 \|bag − header\| < 0.5 s |
| Values and frames | finite; frames exactly as documented; range within limits or ±inf (REP 117); IMU orientation not provided (covariance −1) |
| Vehicle responds | per commanded phase: displacement/(v·T) ∈ [0.7, 1.3], cross-axis < 0.3 m; settle z < 0.10 m |
| IMU | at rest: \|mean − (0, 0, 9.807)\| < 0.2 m/s², σ < 0.1; \|mean gyro\| < 0.02 rad/s; hover \|f\| ∈ [9.3, 10.3]; gyro vs truth rate RMS < 0.05 rad/s |
| Range | residual vs independently recomputed geometry: \|mean\| < 3σ/√n, std within 30 % of σ; below min → −inf |
| Offline | 0 files fetched from Fuel |
| Regression (`--regress`) | 13/13 (including seed reproducibility: same-seed IMU RMS difference < 10 % of σ) |

What this proves: the data pipeline behaves as documented. It says nothing
about REEF, which is not in the loop.

## 4. `baseline` (implemented, P02): migration fidelity

Fixed on 2026-09-30, before any port output exists. Details and
justification are in [BASELINE_DECISION.md §8](BASELINE_DECISION.md#8-numerical-tolerances-fixed-before-any-port-exists).
Scenarios: 15 deterministic fixtures (`baseline/tools/fixtures.py`, locked by
`baseline/fixtures.lock.json`) × 2 parameter sets (common = master's values;
shipped = each revision's own), for master `e4179f48` and sim `95987b51`.

| Criterion | Limit | Measured (P02) |
|---|---|---|
| Reference builds from pinned, checksummed, unmodified sources | all SHA-256 match `baseline/provenance.json` | yes |
| Independent step-wise re-derivation vs reference | \|pred − ref\| ≤ 1e−14 + 1e−12·\|ref\| at every step; every gate decision identical | 0 (bit-identical), 266,796 steps, 41,200 gates |
| Covariance | finite (except the characterized sim `-inf` case); ‖P − Pᵀ‖ ≤ 1e−12‖P‖; min eig ≥ −1e−12‖P‖ | 5.9e−16; 0 |
| Analytic cases (both variants) | hover \|z + h\| ≤ 1 mm and \|ż\| ≤ 1 mm/s; constant-accel climb \|ż_NED + v_up\| ≤ 0.02 m/s; tilt z → −h/cos θ ± 1 mm; mocap velocity ± 0.01 m/s; NaN IMU skipped with next dt = 4 ms; takeoff within 0.1 s of range 0.25 m; RC switch | all pass |
| Characterization (legacy behaviour asserted, not "correct") | bias sign conventions, gravity 9.81 vs measured, 250 Hz velocity ratio (master 1, sim ≈ 2), stale re-fusion (D1), outlier/−inf handling, airborne start (D4) | all as documented |
| Golden stability | decimated outputs within the port tolerance, discrete fields exact | 0; 60/60 bit-identical |
| Negative check | a mutated build (bias coupling −dt → +dt) must fail both the independent check and the golden comparison | detected |
| **Port vs reference (applies from P03/P04)** | continuous: \|port − ref\| ≤ **1e−9 · max_run\|ref_field\|**; discrete (gates, flags, takeoff/landing, reset counter): **exact**; NaN/inf classification identical | floor 2.05e−12 (FMA, `-O3 -march=native`), 0 (`-O0`); master vs sim ≥ 1.08e9× the tolerance |

## 4b. `interfaces` (P03): messages, helpers, parameter validation

Fixed on 2026-09-30 before any P03 code was written (measured column added
afterwards; limits unchanged). Check: `scripts/reef_check.sh interfaces`. Scope: the ROS 2
`reef_msgs` package (the messages and helpers that master `e4179f48` uses) and
the pinned upstream `rosflight_msgs`. There is no estimator node yet, so
nothing here scores estimates. Wall-clock time is not a criterion.

| Criterion | Limit | Measured (P03, 2026-09-30) |
|---|---|---|
| Real packages selected | `colcon list --base-paths src` contains `reef_msgs`, `rosflight_msgs`, `reef_sim`, and no other `rosflight_*` package (no firmware, `rosflight_io`, or simulator) | `reef_estimator`, `reef_msgs`, `reef_sim`, `rosflight_msgs`; no other rosflight package |
| Build | `colcon build --base-paths src` exits 0 for all selected packages | exit 0; no compiler warnings in `reef_msgs` or `reef_estimator` (`-Wall -Wextra -Wpedantic`) |
| Tests executed | `colcon test` and `colcon test-result` exit 0; 0 failures, 0 errors, 0 skipped; `reef_msgs` reports ≥ 1 gtest and ≥ 1 pytest result file, with ≥ 20 test cases in total. An empty test run is a FAIL | `reef_msgs` 49 cases (4 gtest + 2 pytest files), `reef_estimator` 33 (3 gtest + 1 pytest), `reef_sim` 7; 0 failures, errors, skips. An injected failing assertion made `check_colcon.py` exit 1 |
| Upstream RCRaw pin | the vendored `rosflight_msgs` directory's Git tree ID equals upstream `rosflight_ros_pkgs` `v2.0.1` (`cefdb425`) `:rosflight_msgs`, i.e. byte-identical files and modes. A tampered copy must be detected (negative check) | tree `0f469ec1` = upstream `cefdb425:rosflight_msgs` (also the committed tree); one-byte change detected (exit 1, in the wrapper); execute-bit change detected (manual run) |
| RCRaw fields | generated type has exactly `header: std_msgs/Header` and `values: uint16[8]` | yes (`test_rc_raw` static checks, `test_legacy_params`) |
| Message fields | each ported message has the legacy field names, types, and fixed array sizes, except the three renames forced by ROS 2 naming rules (`DeltaToVel.S_upper_bound` → `s_upper_bound`, `S_lower_bound` → `s_lower_bound`, `ZDebugEstimate.P` → `p`) and `Header` → `std_msgs/Header` (no `seq`) | 7 messages match; exactly the 3 renames (`test_messages`) |
| Helper fidelity | the ported helpers give **bit-identical** results (NaN where legacy gives NaN) to the pinned legacy `reef_msgs` sources compiled with the P02 shim, on ≥ 1000 recorded cases each for `quaternion_to_rotation` and `roll_pitch_yaw_from_rotation321`, and on every legal full/diagonal matrix-import and `matrixToArray` case. The recorded vectors come from the legacy code only and are reproducible from the pinned sources | bit-identical: 1520 `quaternion_to_rotation` (Eigen and `geometry_msgs` overloads), 1312 `roll_pitch_yaw_from_rotation321` (signed zeros, NaN, inf), 54 legal matrix imports, 20 `matrixToArray`. Mutations detected: Eigen's algebraically equal `toRotationMatrix().transpose()` (a signed-zero difference) and column-major import |
| ROS independence | the numerical helper library builds and its tests pass with neither rclcpp nor any message package on the include or link path | clean-environment compile and run with only `include/` and Eigen; negative control (`rclcpp.hpp` unreachable) confirmed |
| Parameter acceptance | legal forms load with the legacy meaning: n·m values fill row-major; n values on a square n×n matrix fill the diagonal; integer arrays convert exactly | yes (`test_matrix_operation`, `test_parameters`, `test_ros_parameters`) |
| Parameter rejection (negative) | each of these is rejected with an error naming the parameter and the accepted sizes, and never yields a matrix: missing, empty, wrong length, wrong type (string, bool, string array), non-finite value, `mocap_override_channel` outside 0–7, negative covariance diagonal, asymmetric full covariance | all listed cases rejected with the parameter name and sizes; matrix unchanged (legacy's 12 zero-filled and 12 uninitialized cases now rejected) |
| Legacy YAML | the verbatim master `params/*.yaml` files are characterized under ROS 2's parser (accepted or rejected, with the reason recorded), and the converted ROS 2 file loads to matrices identical to the legacy interpretation of the verbatim file | verbatim files rejected (no `ros__parameters`); wrapped matrix files rejected for mixed int/float lists; wrapped `basic_params` loads with integer gates. Converted file: same keys and values, loads as double arrays |
| Baseline untouched | `baseline/golden/` and `baseline/fixtures.lock.json` unchanged | `git diff 04c9b19 -- baseline/golden baseline/fixtures.lock.json` empty |

## 4c. `estimator` (P04): vertical estimator and ROS 2 wrapper

Fixed on 2026-09-30 before any P04 code was written. Scope: the **vertical**
filter of master `e4179f48` (Z Kalman filter, IMU handling, accelerometer
initialization, landing reset, range and mocap-z updates and gates, RC
switch for z, takeoff/landing detection). The horizontal filter is not
ported; its cases are reported as NOT IMPLEMENTED and never counted as
passed. No corrections (C1–C5 deferred to R1).

### Fidelity: port vs reference (fixture time)

Event streams: the 15 P02 fixtures (master, both parameter kinds) and the P04
vertical fixtures `v01`–`v10` (`baseline/tools/fixtures_vertical.py`, locked
by `baseline/fixtures_vertical.lock.json`). Each stream is fed unchanged to the
reference harness (pinned original, `build_reference.sh master`) and to the
port's event driver, with the same subscription rules.

| Criterion | Limit |
|---|---|
| Continuous Z fields per event (`z, zdot, zbias`, 9 × `zP`, `z_meas`, `zR`, `u`, `z_dt`, `g_init`) | \|port − ref\| ≤ 1e−9 · max_run\|ref_field\| (ACCEPTANCE §4, unchanged); NaN/inf classification identical |
| Discrete fields per event (`z_flag`, `takeoff`, `n_prop`, `use_mocap_z`, `acc_init`, `n_published`, `delivered`) | exact |
| Gate value `maha2` on events where the port evaluated a Z gate | same tolerance as continuous fields |
| Port vs committed P02 golden (decimated rows, Z fields) | same tolerance; discrete exact |
| Negative: port vs the simulation-revision reference (`95987b51`) on `s09` | must **exceed** the tolerance (the comparison can fail) |
| Named vertical cases | stationary `s01`; ascent `s02`, `s03`; descent and landing `v01`; tilt/range geometry `s05`; parameter errors (node test); missing data `v02`, `s11`; range-invalid `v03`, `s10`; repeated measurements `v04`, `v08`; timestamp anomalies `v05`–`v07`; gravity/bias `s04`, `s08`; mocap z `s12`; RC switch `s13`; full update `s14`; z disabled `v10`; 250 Hz `s09`; airborne start `s15` (characterization). Each passes the fidelity rows above |

### ROS 2 wrapper

| Criterion | Limit |
|---|---|
| Wrapper equivalence | every fixture event delivered through the node's message callbacks (ROS types, float32 ranges, `builtin_interfaces/Time` stamps) gives outputs bit-identical to the core, for all vertical-relevant fixtures |
| Live transport | a node started as a process receives IMU/range messages over DDS and publishes `xyz_estimate` whose values equal the core's for the same stream; `is_flying_reef` and outputs use the QoS of INTERFACES.md §3.3 |
| Clock policy | dt from IMU header stamps with the ROS 1 formula (sec + 1e−9 · nanosec); output stamp = triggering IMU stamp; node clock not used for estimation |
| Parameter errors | an invalid parameter file makes the node exit non-zero before publishing, naming the parameter |
| Reset | the reset service returns the core to its startup state: the following outputs equal those of a fresh node on the same stream |
| Shutdown | SIGINT exits 0 within 5 s; no process of the test survives |

### Estimator quality in simulation (sim time; idealized sensors)

Stock-controlled X3 scenario (`x3_scenario.yaml` merged with
`x3_reef_overlay.yaml`, seeds 7/42), truth-fed controller, REEF **not** in
the loop. Inputs: `/x3/imu` converted FLU → FRD, attitude from **truth**
(idealized), range from the idealized sensor (no tilt compensation, as in
master).

**Scenario assumption (added 2026-09-30, before any simulation result was
scored; limits unchanged; kept by the USER on 2026-09-30, with the detector
itself listed as correction candidate C6):** the IMU gets 1.0 m/s² per-axis white "rotor
vibration" (`imu_noise.vibration_std`). The original takeoff detector needs
accelerometer-magnitude variance ≥ 0.5 (m/s²)², which vibration supplied on
the REEF hardware. An offline replay of an existing vibration-free P01
recording showed that the estimator then never declares takeoff and its
landing reset pins ż at 0 for the whole flight. The value was chosen to
exceed the threshold (variance ≈ 1.0), not measured. The vibration-free
behaviour is reported as a characterization, not scored. Truth for scoring: the vertical height of
the range-sensor origin (−z NED) and the vertical velocity from
`/x3/truth/odom`, interpolated to each estimate stamp. **Initialization
interval:** from start until 2 s after REEF declares takeoff; not scored.
Scoring window: from its end to the end of `hover_low`.

| Criterion | Limit |
|---|---|
| Takeoff | declared during `ascend`; no landing declared before the end of `hover_low` |
| Outputs | finite; count ≥ 50 % of IMU messages after initialization |
| Altitude error (z vs −h_sensor) | RMSE ≤ 0.05 m; peak ≤ 0.15 m |
| Vertical velocity error (ż vs −v_up) | RMSE ≤ 0.10 m/s |
| Consistency | fraction of samples within ±3σ (from `p`) reported per phase; no limit (the filter is not claimed to be consistent) |
| Labels | plots and reports state "idealized attitude (truth) and idealized range" and the vibration assumption |
| Characterization (not scored) | the same estimator on a vibration-free recording: takeoff declared or not, ż behaviour |

### Measured (P04, 2026-09-30; limits above unchanged)

Check: `scripts/reef_check.sh estimator` (original container). Details and
commands: [reviews/P04.md](reviews/P04.md).

| Criterion | Measured |
|---|---|
| Fidelity, continuous and discrete Z fields, gate values | 40 streams (15 P02 fixtures × 2 parameter kinds + v01–v10), 209,587 events: **bit-identical** in every compared value; discrete fields and every gate value identical |
| Port vs committed P02 golden | 15/15 match, worst 0 × tolerance |
| Negative (port vs simulation revision, `s09`) | 2.08e9 × tolerance: detected |
| Named vertical cases | all pass (16 case groups; horizontal cases listed as NOT IMPLEMENTED) |
| Wrapper equivalence | 40/40 streams: node state = core state at every event; every published message carries exactly that state, horizontal fields NaN |
| Live transport | launch test: 180 estimates over DDS equal to the core's (exact); `xyz_estimate` and `is_flying_reef` reliable + transient local |
| Clock policy | unit tests (ROS 1 `toSec` formula, output stamp = IMU stamp); v05–v07 bit-identical |
| Parameter errors | process exits 1, message names `z_Q`, no publisher (launch test) |
| Reset | after `reset()` 150 IMU steps give messages equal to a fresh node's; reset service and backward sim-time jump tested |
| Shutdown | SIGINT exit 0 (launch test) |
| Takeoff | declared at 7.77 s, during `ascend` (6.66–11.66 s); no landing |
| Outputs | finite; 9471 estimates for 9472 IMU inputs after initialization |
| Altitude error | RMSE **6.8 mm**, peak 26 mm (per phase 5.4–8.5 mm) |
| Vertical velocity error | RMSE **0.037 m/s**, peak 0.135 m/s (per phase 0.031–0.044 m/s) |
| Consistency (reported) | 100 % of samples within ±3σ in every phase for z and ż: the covariance is conservative (σ_ż ≈ 0.075 m/s vs 0.037 m/s RMSE) |
| Labels | plots and reports carry the idealized-input and vibration labels |
| Characterization, vibration-free (P01 recording, offline replay) | takeoff **never declared**; ż stays within ±0.040 m/s all flight; from ascend + 2 s: altitude RMSE 16 mm, vertical velocity RMSE 0.145 m/s (would fail the limit) |

## 4d. P05: combined (vertical + horizontal) estimator

Fixed on 2026-09-30 before any P05 code was written. Scope: the complete
estimator of master `e4179f48` (horizontal EKF with velocity, attitude-bias,
and accelerometer-bias states; mocap and RGB-D velocity updates; RC switch;
landing reset) plus the vertical filter of §4c. No corrections are approved
(C1–C6 deferred to R1). Numerical parity (`reef_check.sh baseline`) and
physical plausibility (`reef_check.sh estimator`) are judged separately;
fault behaviour is `reef_check.sh faults` (§5).

### Numerical parity (fixture time) — `reef_check.sh baseline`

| Criterion | Limit |
|---|---|
| Streams | 15 P02 fixtures × 2 kinds, v01–v10, and new horizontal fixtures h01–h10 (`fixtures_horizontal.py`, locked), including nonzero yaw and tilt, mocap/RGB-D outliers, duplicates, out-of-order stamps, dropouts, RC switching both ways, full update, landing reset, `enable_xy` false |
| Every state field (Z and XY: state, full P, measurement, R, dt, flags, takeoff, counters, `use_mocap_xy/z`) | \|port − ref\| ≤ 1e−9 · max_run\|ref_field\|; discrete fields exact; gate values within the same tolerance on events where the port evaluated a gate; NaN/inf class identical |
| Golden anchor | port vs the committed P02 golden, all fields, same tolerance |
| Wrapper equivalence | the same events through the node's callbacks as ROS 2 messages: node state = core state; published messages carry exactly that state (now including XY) |
| Negative | port vs simulation revision on `s09` must exceed the tolerance; the port with correction C1 enabled must differ from the reference on `s07` |
| Published messages (added after R1, 2026-09-30) | on every stream, the messages the node publishes (`xyz_estimate` and the `xyz_debug_estimate` sent with it) are bit-identical to what the original published, recorded by the reference harness (A6). Parity runs pin `correction_c1_clear_xy_flag` to false (master semantics) |
| Correction C1, the default since R1 | on every stream, the port's default output equals the independent step-wise model with C1 (independent.py; tolerance as in §4), and that model equals the original with C1 off |

### Frame conversions (independent checks)

| Criterion | Limit |
|---|---|
| Horizontal propagation | one propagation step with known specific force and attitude (yaw 0°, 30°, 90°, −135°; roll and pitch up to ±20°) gives Δv equal to the body-level acceleration × dt, computed independently (Eigen AngleAxis rotations, not the code's formula): error ≤ 1e−12 m/s |
| Simulated velocity observation | the adapter's body-level velocity from truth (yaw 90° and tilted cases) equals an independent computation: ≤ 1e−12 m/s |

### Observation reuse (D1) and correction C1

| Criterion | Limit |
|---|---|
| Baseline | D1 preserved (parity above); the core counts XY fusions per accepted observation and the node reports it |
| ROS layer | never adds fusions: fusion counts through the node = core counts on every stream |
| C1 (**approved at R1, default on**; off = master) | with C1 on: no observation is fused twice (an IMU step fuses exactly when a new observation arrived since the previous step, then clears the flag) on s06/s07/h fixtures; observations superseded by a newer one before the next IMU step are never fused (last one wins, as in master) and are counted; with C1 off: parity unchanged |

### ROS 2 wrapper, QoS, executor, replay isolation

| Criterion | Limit |
|---|---|
| QoS | every publisher/subscriber pair of the REEF graph (adapter, estimator, recorder) is compatible (tested from graph info) |
| Executor | single-threaded executor; callback wall time p99 ≤ 2 ms, max ≤ 20 ms in the simulation run (IMU period 4 ms) |
| Replay isolation | the ROS replay refuses to start (exit 2) when its domain already has a publisher of `/clock` or of any played or REEF input topic; negative cases for a foreign `/clock` and a foreign `/x3/imu` |

### Physical plausibility in simulation — `reef_check.sh estimator`

Same scenario and assumptions as §4c, plus **idealized simulated velocity
observations**: body-level velocity derived from truth, with 0.02 m/s white
noise keyed by (seed, stamp), at 100 Hz, through REEF's mocap-velocity input
(not RGB-D odometry). Initialization interval and scoring window as in §4c.

| Criterion | Limit |
|---|---|
| Vertical | §4c limits unchanged |
| Horizontal velocity, per body-level axis | RMSE ≤ 0.10 m/s; peak ≤ 0.30 m/s |
| Bias plausibility (truth: zero attitude and accel bias) | \|attitude bias\| ≤ 0.05 rad, \|accel bias\| ≤ 0.5 m/s² in the scoring window |
| Finite and covariance | all outputs finite; every published P symmetric (‖P − Pᵀ‖ ≤ 1e−9‖P‖), min eigenvalue ≥ −1e−9‖P‖ |
| Initialization | first estimate after exactly 20 IMU messages; takeoff during `ascend`; the horizontal state before takeoff is reported |
| Timing | estimate age (recorder receive time − stamp, sim time) p99 ≤ 20 ms |
| Recorded-stream parity | the simulation run's inputs as an event stream: port = reference, bit-identical |
| Replays | two offline replays bit-identical; ROS-replay metrics reported next to live |
| Consistency (reported, no limit) | per axis: fraction within ±3σ and mean NEES, with the effective sample size from the error autocorrelation; assumptions stated in the report |

### Measured (P05, 2026-09-30; limits above unchanged)

Evidence and commands: [reviews/P05.md](reviews/P05.md), [reviews/R1_packet.md](reviews/R1_packet.md).

| Criterion | Measured |
|---|---|
| Numerical parity | 50 streams, 282,490 events, 18,079,360 values: **all bit-identical** (built -O2 and -O0); discrete fields and gates identical; golden 15/15 |
| Wrapper equivalence | 50/50 streams, XY message fields and fusion counts included |
| Negatives | simulation revision 2.08e9 × tolerance; C1 on s07 4.87e8 × tolerance |
| Frame conversions | propagation Δv vs independent AngleAxis construction ≤ 1e−12 for yaw 0/30/90/−135/170° with tilt to 20° (a transpose mutation fails the test); adapter body-level velocity vs independent matrices ≤ 1e−12 (200 random attitudes, yaw 90° and tilted known cases) |
| D1 / C1 | D1: s07 1 observation fused 999 times; C1: no observation fused twice on 7 streams (fusions = IMU steps with a new observation; superseded observations counted: 3 at initialization, 104 duplicates in h04) |
| QoS / executor | graph-level QoS test; callback wall time: 27 of 17,390 callbacks > 2 ms (0.16 %), median 0.18 ms, max 6.9 ms |
| Replay isolation | foreign `/clock` and foreign `/x3/imu` both refused (exit 2); during playback each input has only its intended publisher |
| Vertical (sim) | altitude RMSE 6.8 mm, ż RMSE 0.037 m/s |
| Horizontal (sim, idealized velocity observations) | body-level x RMSE 0.0106 m/s (peak 0.035), y 0.0109 m/s (peak 0.044) |
| Bias plausibility | ≤ 0.0066 rad attitude, ≤ 0.011 m/s² accel |
| Covariance | published Z p: max asymmetry 1.5e−16, min eigenvalue > 0; full Z and XY P in the offline core output checked the same way |
| Initialization | first estimate at the 21st IMU message; takeoff at 7.77 s (ascend) |
| Timing | estimate age: dev container at `8bac5a5` **p99 22 ms (FAIL)**, tail from the Python adapter; after moving the IMU adapter to C++ (`3edbd95`) p99 8–10 ms here (limit 20 ms unchanged; the estimator stage adds p99 about 4 ms at the 2 ms clock resolution) |
| Recorded-stream parity, replays | simulation stream port = original, bit-identical; two offline replays byte-identical; ROS replay metrics equal to live within 0.1 mm / 0.1 mm/s RMSE |
| Consistency (reported) | 100 % within ±3σ for z, ż, vx, vy (conservative covariances); NEES and effective sample sizes in the report |

### Measured: faults (2026-09-30)

35 of 36 assertions pass. **F11 (baseline, C1 off) fails**: after the 5 s
simulated velocity dropout during `forward`, all 2629 later observations are
rejected by the gate (the stale observation kept the variance small, D1) and
the horizontal error stays at 0.51 m/s RMS (limit 0.10). With C1 the error
returns to 0.015 m/s. F1 baseline D1 behaviour asserted (1499 re-fusions,
variance not growing); F1/F11 with C1 pass; F2–F10, F12, determinism pass.
This failure is the approved baseline's behaviour and is left failing for
R1 (correction C1); the limit is not changed.

## 5. Future targets: criteria to be fixed before implementation

These are drafts. Items marked **PROPOSED** must be confirmed (or replaced,
with reasons) at the start of their milestone and before any port output is
scored.

### `estimator` (P04–P05): estimator quality

Scenario: the X3 scenario (or its extension), fixed seeds, sim time. Warm-up:
the settle phase plus 2 s after takeoff. Metrics per phase, in the estimator's
documented frame (NED/body-level), after conversion from truth.

- **Altitude and vertical velocity:** fixed for P04 in §4c (the proposed
  0.05 m / 0.15 m peak / 0.10 m/s were confirmed). Rationale: idealized sonar
  σ = 0.01 m and IMU σ_a = 0.02 m/s² should support centimetre-level vertical
  estimates; the margin covers lag during 0.4 m/s ramps and the uncompensated
  slant range (≤ 9 mm at the scenario's tilt).
- **Horizontal velocity:** fixed for P05 in §4d (idealized truth-derived
  velocity observations, labelled as such).
- **Consistency:** finite values; covariance bounds reported. Any statistical
  consistency test (for example NEES) states its assumptions.
- **Timing:** estimate stamp age relative to the input stamp is reported, in
  sim time and in wall time.

### `faults` (P05): fixed on 2026-09-30 before any P05 code was written

`reef_check.sh faults`. Each case states the specified behaviour; a case
passes only when that behaviour is observed (never merely "no crash").

| Case | Specified behaviour (limit) |
|---|---|
| F1 velocity dropout (fixture h05) | **amended 2026-09-30 twice, see notes.** Default configuration (C1 on, approved at R1): no fusion during the dropout, velocity variance grows monotonically. Legacy master (C1 off, characterization, asserted): the last observation is re-fused at every IMU step during the dropout (D1) and the velocity variance does not grow. Both: the first observation after the dropout is accepted |
| F2 outlier observation | rejected by the gate (maha² > limit); state and flags unchanged by it |
| F3 duplicate observation | processed as master (parity); fusion count through the node = core |
| F4 out-of-order measurement stamps | no effect beyond arrival order (measurement stamps unused, as in master; parity) |
| F5 IMU NaN / gap / backward stamp | outputs as master (parity); every output finite |
| F6 range dropout / invalid ranges | z propagates; invalid ranges never gated in (parity v02/v03) |
| F7 start-up | no output for the first 20 IMU messages; landing reset every 10 propagations until takeoff |
| F8 resets | reset service and backward time jump reset (node tests); landing transition resets both filters (parity v01/h-series) |
| F9 parameter failures | node exits 1 naming the parameter, before publishing |
| F10 replay isolation | foreign `/clock` or foreign `/x3/imu` in the replay domain: replay refuses (exit 2) |
| F11 simulated velocity dropout (offline replay of the simulation run with the velocity stream removed for 5 s) | default configuration (C1 on): outputs finite; horizontal variance grows during the dropout; horizontal error ≤ 0.10 m/s RMS from 1 s after the dropout ends (limit unchanged). Legacy master (C1 off, characterization): D1 re-fusion during the dropout and the lock-out after it are reproduced and reported |
| F12 missing IMU stream (offline replay without IMU) | no estimates; the analysis fails (exit 1) |
| Replay determinism | two offline replays bit-identical; ROS replay nondeterminism (delivery order) documented |

Note on F1/F11 (amended 2026-09-30, after the first run of the h05 fixture
and before any faults result was scored): as first written, F1 and F11
required the velocity variance to grow during a dropout. The approved
baseline cannot do that: master re-fuses the last observation at every IMU
step (legacy defect D1), which the USER decided to preserve until R1. The
requirement now applies with correction C1 enabled, and the baseline's D1
behaviour is asserted as a characterization, as the P02 characterization
cases do. Recorded for R1 as evidence for C1.

Note after R1 (2026-09-30): the USER approved correction C1 after the R1
self-review, and it is now the port's default. F1 and F11 are therefore
judged on the default configuration; the legacy (C1 off) runs remain as
characterizations that must still reproduce master's D1 behaviour. F11's
error limit is unchanged; the approved correction, not a changed limit, makes
it pass. Criteria for published messages and for C1 against the independent
model were added (stricter, §4d).

### `control` (P06): controller fidelity — fixed 2026-09-30, before any P06 port code

Specification: [CONTROL_CHAIN.md](CONTROL_CHAIN.md). USER: the port is
strictly faithful and bit-exact; no robustness changes. Reference:
`reef_control` `12237b76`, compiled unmodified in a harness (like
`baseline/`). Fixtures: deterministic event streams (estimate, desired
state, status, `is_flying`, pose, gain change) generated by a script and
locked by SHA-256, plus one stream driven by a recorded X3 estimate run.

| Criterion | Limit |
|---|---|
| Reference build | pinned, checksummed, unmodified sources; adaptations listed in `baseline/README.md` |
| Port core vs reference | every output of every step **bit-identical**: which events produce a step, `command` (mode, ignore, and x, y, z, F as float32), `controller_state` (every field), internal PID states (integrator, differentiator, last state) |
| Node vs core | the same streams through the ROS 2 node: published messages equal the core outputs, bit for bit |
| Independent model vs reference | step-wise model written from the equations (not the code): \|model − ref\| ≤ 1e−14 + 1e−12·\|ref\| on every field; mode and ignore decisions exact |
| Negative control | a mutated reference (one sign flipped in the D term) must fail the model check and the port comparison |
| Required cases (fixtures) | altitude hold; climb/descent saturation and integrator growth (K2); velocity mode both axes; position mode (deadzone, sigmoid, `face_target`, `fly_fixed_wing`); heading across ±π (K9); attitude mode (K8); altitude-only (K7); flags cleared (feed-forward pass-through); arming and `is_flying` transitions (integrator clearing, last callback wins); first step huge dt (K4); equal and backward stamps; NaN estimate (K6); gain change at runtime without integrator reset (K11); no desired state yet |
| Independent sign checks (unit tests, expected values not from the code) | above target → less throttle; forward velocity deficit → negative pitch; rightward deficit → positive roll; limits ±max_roll/pitch/yaw rate and F ∈ [0, 1] hold for finite inputs |
| Characterizations asserted | K1–K13 as specified in CONTROL_CHAIN.md §5 |
| Parameters | missing `max_roll`/`max_pitch`/`max_yaw_rate` or a gain outside its `Gains.cfg` range → node exits 1 naming it, before publishing; runtime out-of-range change rejected; in-range change applied as `gainsCallback` |
| Stale/invalid/missing inputs, restart | behaviour as the original (characterized): no estimate → no command; stale desired state persists; a restarted node equals a fresh one |
| Dry-run sink | records every command with firmware interpretation (ignore bits, 100 ms offboard timeout, armed); has no hardware output; `hardware:=true` is refused (exit 2, NOT IMPLEMENTED) |
| Closed loop | **NOT IMPLEMENTED in P06** (P07; reported as N/A, not PASS) |

### `control` (P07): closed loop — fixed 2026-10-01, before any closed-loop run

USER (2026-10-01): close the loop with the stand-in low-level loop of
CONTROL_CHAIN.md §7 (a development tool, not ROSflight), driving Gazebo's
motor model directly, with the stock controller removed; record initial
stability and tracking. Everything here is **simulation with idealized
inputs** (truth attitude to REEF and to the stand-in, idealized range,
simulated velocity observations, IMU vibration assumption); it is not
ROSflight, hardware, or flight evidence.

Scenario `closed_loop` (sim time, 250 Hz IMU): disarmed settle 5 s; arm;
velocity mode with zero velocity and altitude setpoint z = −1.0 m (REEF
NED: range sensor 1.0 m above ground) for 20 s (takeoff + hover); forward
0.3 m/s 6 s; hover 6 s; left 0.3 m/s 6 s; hover 6 s; **back** (0.3 m/s
backward and 0.3 m/s right together, undoing both legs) 6 s; hover 6 s; yaw
rate 0.3 rad/s 6 s; hover 6 s; climb to z = −1.5 m 20 s; descend to
z = −0.6 m 20 s; **approach** z = −0.3 m 6 s; **land** z = +0.1 m (below the
ground, so the controller keeps descending) 8 s; **disarmed** 4 s (the
stand-in stops the motors). The return legs come before the yaw phase
because velocity commands are relative to the heading. (Amended 2026-10-01,
see the note below the table.) Gains: the
shipped quad gains (`config/reef_control_quad.yaml`) unless replaced in a
separate, explained configuration commit (controller math never changes);
stand-in gains from CONTROL_CHAIN.md §7, chosen before the first run.

| Criterion | Limit |
|---|---|
| Architecture | the world has no `MulticopterVelocityControl` and nothing publishes `/x3/cmd_vel`; the motor command topic has exactly one publisher (the stand-in); one physics engine (Gazebo); every topic, manifest and plot labels the stand-in a development tool |
| Data path | truth is used only by the documented idealized inputs (REEF attitude, range, velocity observations; stand-in attitude and rates) and for scoring; the controller reads only REEF estimates and the setpoint (checked from the ROS graph: its subscriptions and their publishers) |
| Takeoff | REEF reports takeoff and the vehicle leaves the ground within 15 s of arming |
| Stability (from takeoff to the end of `descend`; tilt and finiteness also through `approach` and `land`) | finite outputs; \|roll\|, \|pitch\| ≤ 0.35 rad (truth); range-sensor height ≥ 0.25 m; no flip; the run completes |
| Altitude step response (takeoff to 1.0 m; 1.0 → 1.5 m; 1.5 → 0.6 m) | overshoot ≤ 0.4 m; within ±0.15 m of the setpoint by 15 s after the step and staying there to the end of the phase |
| Altitude hold (last 4 s of each hover phase) | RMSE of truth sensor height vs setpoint ≤ 0.10 m |
| Horizontal velocity (last 3 s of each velocity phase, including `back`; last 4 s of hovers) | RMSE of truth body-level velocity vs command ≤ 0.15 m/s per axis |
| Yaw rate (last 3 s of the yaw phase) | \|mean truth yaw rate − 0.3\| ≤ 0.1 rad/s |
| Saturation (from takeoff to the end of `descend`) | throttle command at 0 or 1 for ≤ 5 % of samples; motor-speed clamping in ≤ 5 % of stand-in steps |
| Staleness and latency | age of the estimate at the motor command (ROS clock, which is sim time, at the stand-in step minus the command's estimate stamp), p99 ≤ 20 ms; no offboard timeout after arming |
| Causality | a second run with a +0.30 m bias on the range measurement (test hook, labelled) holds the truth height lower by 0.30 ± 0.10 m in the first hover than the nominal run: the REEF estimate, not truth, closes the altitude loop |
| End state (last 1 s of `disarmed`) | the stand-in is disarmed and every motor-speed command is 0; truth sensor height ≤ 0.05 m, speed ≤ 0.05 m/s, tilt ≤ 0.1 rad: the vehicle rests on the ground before anything shuts down |
| Return and landing | reported, not judged: horizontal distance from the takeoff point at the end of `hover_back` and at the end; touchdown vertical speed; time from the start of `land` to touchdown |
| Reproducibility | reported, not judged: a second nominal run (same seeds) and the spread of every metric (Gazebo and ROS timing are not deterministic) |
| Comparison | the stock (truth-fed) run and the REEF-controlled run are both recorded with manifests and plots; differences are reported, not judged |

Note (2026-10-01, USER-approved amendment after the GUI demo): when the
scenario ended, every ROS node stopped while the Gazebo server ran on for
5 s or more; the motor model keeps the last commanded speeds, so the
uncontrolled vehicle flew away (no recorded metric was affected; scoring
ends with the last phase). The scenario now returns, lands, and disarms
before it ends, and the end state is judged. The stand-in also commands
zero motor speeds when it shuts down. Existing limits are unchanged; the
stability and saturation windows end with `descend` because landing
legitimately goes below 0.25 m and drives the throttle to 0.

Latency margin (clarified after R2, no limit changed): the age is measured
in sim time and is quantized to the 2 ms physics step, so p99 values are
even milliseconds. Over the 19 headless closed-loop runs to 2026-10-01 the
p99 was 12–24 ms (median 12 ms; 15 runs 12–16 ms; one exactly 20 ms, which
passes; one 24 ms, which failed); the two GUI runs gave 20 and 24 ms. The
limit is therefore met with margin in most headless runs but not
guaranteed: a headless run on a busy host can fail it.

USER decision (2026-10-01): **official acceptance comes from headless runs**
(`reef_check.sh control`) on an otherwise idle machine. GUI runs
(`reef_demo.sh closed-loop --gui`) are informational only: the software-rendered
viewer and other host load lengthen the ROS pipeline's scheduling delays,
which the latency criterion measures (a GUI run on a loaded host gave p99
24 ms with every other check passing). No latency rework is planned.

Not covered in P07 (later, with their own criteria): RC override, failsafe,
estimate interruption and restart, landing quality (only reported),
position mode (needs a mocap input; the controller supports it), wind,
sensor faults.

### `control` / `faults` (P07b): closed-loop faults and position mode — fixed 2026-10-01, before any P07b code

USER (2026-10-01): estimate dropout and restart, closed-loop fault cases,
and position mode, before R2. **Strict parity with the legacy baseline: no
new failsafes**; a crash is the correct documented result where the legacy
system has no protection (long estimate loss, stand-in death). Same
labels and limits of validity as P07 (stand-in development tool, idealized
inputs, simulation only). Official results come from headless runs.

**Fault scenarios** (`run_x3_scenario.sh --closed-loop` with
`REEF_X3_CL_SCENARIO=<name>`; X3 gains): settle 5 s (disarmed); takeoff
and hover at z = −1.0 m 20 s; `fault` 15 s (the fault is injected at its
start); `recover` 10 s hover; `approach`, `land`, `disarmed` as in P07.
Faults are injected by the scenario runner through **labelled test hooks**
(off by default; enabled only by the scenario's parameters): IMU drop in
the IMU adapter (stops REEF's estimates), range drop in the range sensor,
velocity-observation drop in the REEF adapter, a graceful exit of the
controller (respawned by the launch) or of the stand-in, the estimator's
`~/reset` service, and a Gazebo pause. The P06 controller core is
unchanged; hooks live in ROS layers only. **Judged** = PASS/FAIL limit;
**characterization** = the legacy behaviour is asserted and reported, not
judged as desirable.

| Scenario | Fault | Judged | Characterization (asserted, reported) |
|---|---|---|---|
| `dropout_short` | IMU drop 50 ms (below the 100 ms offboard timeout) | estimate gap 40–100 ms observed; no offboard timeout; tilt ≤ 0.35 rad; \|h − 1.0\| ≤ 0.15 m through `fault` and `recover`; P07 end state | — |
| `dropout_long` | IMU drop until the end of the run | estimate gap ≥ 10 s; the stand-in's offboard timeout fires within 150 ms (sim) of the last command; selected throttle 0 afterwards | the vehicle falls: vertical speed ≤ −1 m/s and height < 0.1 m within 2.5 s of the timeout (crash; impact speed reported). End state not judged |
| `estimator_reset` | estimator `~/reset` in flight | the reset is accepted; outputs finite; the run completes | **(corrected)** REEF announces landed (`is_flying_reef` false), publishes no estimate during its accelerometer initialization (gap ≤ 0.1 s), re-detects takeoff (`is_flying_reef` true again), and restarts its altitude filter from z_x0: the first estimate after the reset lies between z_x0 and the measured height. Min height, max tilt, recovery reported (a crash is reported as such) |
| `controller_restart` | controller exits (graceful) and is respawned | a command gap and a new controller process are observed; outputs finite; the run completes | integrators restart from 0: altitude loss and recovery time reported (a crash is reported as such) |
| `setpoint_stale` | the runner stops publishing during a 0.3 m/s forward leg (6 s), then resumes with hover | tilt ≤ 0.35 rad; P07 end state | the last setpoint persists (no freshness check): mean truth forward velocity in the last 3 s of the stale window 0.3 ± 0.1 m/s |
| `range_loss` | range drop 10 s at hover | outputs finite; tilt ≤ 0.35 rad; the run completes | altitude from the IMU only: truth height deviation and REEF z error during the loss reported |
| `velocity_loss` | velocity-observation drop 10 s at hover | **(corrected)** no observation fused during the loss; REEF's horizontal velocity σ never below its value at the start of the loss and ≥ 10× that value at its end (C1; F11 in closed loop); tilt ≤ 0.35 rad; the run completes | horizontal truth drift during the loss reported |
| `pause_resume` | Gazebo paused 2 s (wall) at hover | the pause took effect (runner confirms); no offboard timeout; tilt ≤ 0.35 rad; \|h − 1.0\| ≤ 0.15 m through `fault` and `recover`; P07 end state | — |
| `standin_exit` | the stand-in exits (graceful) in flight | its last motor command is all zeros | the vehicle falls: vertical speed ≤ −1 m/s and height < 0.1 m within 2.5 s (crash). End state not judged |

**Position mode** (`REEF_X3_CL_SCENARIO=position_square` and
`position_face_target`). Input: an **idealized mocap pose** published by
the REEF adapter on `/x3/reef/pose_stamped` (truth position in NED and
truth attitude of FRD in NED, labelled IDEALIZED; enabled only by these
scenarios). Square of 1.5 m legs in mocap NED: (1.5, 0), (1.5, 1.5),
(0, 1.5), (0, 0), 12 s each, heading setpoint held at the initial heading.

| Criterion | Limit |
|---|---|
| Data path | the controller's inputs as in P07 plus `pose_stamped` from the REEF adapter only |
| Position (last 2 s of each leg) | horizontal distance from the waypoint ≤ 0.15 m (the lookup table's dead zone is 0.10 m) |
| Overshoot (each leg) | maximum distance beyond the waypoint along the leg ≤ 0.30 m |
| Stability and altitude | tilt ≤ 0.35 rad; altitude-hold RMSE ≤ 0.10 m in the last 4 s of each leg; P07 end state |
| K9 (characterization) | heading setpoint 2.9 rad, then −2.9 rad: the second turn goes the long way (yaw-rate command and truth yaw rate negative at its start, although the short way is positive) |
| K10 (characterization, `position_face_target`: `face_target` true, one leg to (1.5, 1.5)) | at the first position-mode step the heading setpoint equals the current heading (θ = 0 before the first lookup); later steps use θ of the previous step, so the heading turns towards the bearing of the target: **(corrected)** the heading error to the bearing decreases from the leg start and is ≤ 0.2 rad when the vehicle first comes within 0.15 m of the target; inside the dead zone the bearing of a target a few cm away swings and the heading follows it (reported) |

Runs: `reef_check.sh faults` adds every scenario above (fixture-level F1–F12
unchanged).

Corrections (2026-10-01, USER-approved after the first runs; they correct
my descriptions of the legacy behaviour, not a tolerance on the port, which
stays bit-exact): `velocity_loss` — the 6-state filter's propagation couples
velocity and attitude bias, so in maneuvering flight σ dips slightly at
some steps (496 of 2449, at most 0.010 m/s) while growing 34× overall;
`estimator_reset` — the first published estimate already includes a range
update (0.536 m between z_x0 0.25 m and the true 0.983 m), so "pinned at
z_x0" never appears in the output; K10 — the vehicle reaches the target in
about 3 s while the stand-in's slow yaw loop is still turning, and the
bearing of a target inside the dead zone swings, so "within 0.2 rad by 6 s"
was the wrong moment.

### `vision` (P08): RGB-D velocity — fixed 2026-10-01, before any P08 code

USER decisions (2026-10-01): **odometry front-end = a compact OpenCV
replacement** (option B), labelled a replacement everywhere (it is not a
port of `demo_rgbd`, which needs ROS 1, PCL, and iSAM and cannot be
reference-tested here); **`rgbd_to_velocity` gets the full P06 treatment**
(history import, unmodified original in a harness, bit-exact parity,
independent model, legacy quirks kept and asserted); camera profile
**320×240 at 15 Hz** (fallback **160×120 at 10 Hz**, documented, used only if
a container cannot hold real time); VRPN mocap and joystick teleop are
**Unsupported/Deferred** in the capability matrix; no Dockerfile change.
Validity limits as P07 (stand-in low-level loop, truth attitude to REEF).
**Truth-derived velocity is not acceptance evidence** in this target: in
vision runs the truth-based velocity observations are disabled and the
estimator's only horizontal velocity input is the vision chain.

Chain: Gazebo `rgbd_camera` on the X3 (forward-looking, legacy mounting:
about 0.14 m ahead of the body origin) → ros_gz bridge (RGB, depth,
camera info) → **replacement RGB-D odometry** (OpenCV 4: corner tracking,
depth lookup, robust pose; output `cam_to_init` in DEMO's camera
convention) → **`rgbd_to_velocity`** (ported) → `reef_msgs/DeltaToVel`
→ REEF estimator (`enable_rgbd`, `enable_measurements` true; mocap
velocity off).

| Area | Judged |
|---|---|
| Camera interface | camera info consistent with the SDF (fx = fy = w / (2 tan(fov/2)), principal point at the image centre within 0.5 px, no distortion); the ROS optical frame verified by projecting a known world target into the image (error ≤ 1 px); depth in metres; invalid pixels (outside the clip range: ±inf or NaN as delivered) counted and never used; image stamps = the render sim time; the camera extrinsics in the world equal the converter's `body_to_camera` parameters |
| `rgbd_to_velocity` port | the unmodified original (pinned, checksummed) in a harness; port output **bit-identical** on locked fixtures, every output field; independent step-wise model; negative control (a mutated original must fail); legacy quirks asserted as characterizations |
| Replacement odometry, open loop (stock truth-fed controller flying a velocity profile in the textured scene; vision not in the loop) | vision velocity vs truth body-level velocity, steady segments: RMSE ≤ 0.10 m/s per axis and |bias| ≤ 0.03 m/s; output rate ≥ 10 Hz (sim); sign/frame tests (forward motion → +x, rightward → +y); measured noise vs the configured covariance reported (noise, frame, and timing assessment of the replacement) |
| REEF with vision, open loop | REEF horizontal velocity RMSE ≤ 0.10 m/s per axis against truth (the P05 limit) on vision input only; outputs finite; gate decisions and fusion counts reported; data path from the ROS graph: the estimator's horizontal velocity input comes only from `rgbd_to_velocity` |
| Weak texture / depth loss (scripted segments) | the odometry's health shows the loss within 0.5 s (sim) and the chain publishes no velocity while lost (never truth); REEF's horizontal variance grows; after the segment, velocity output resumes within 1 s and REEF's error is back within 0.10 m/s within 3 s |
| Delayed (200 ms) and missing (1 s) frames (labelled test hooks, off by default) | outputs finite; no vision velocity error above 0.3 m/s; latency and rate reported |
| Closed loop (REEF controller and the P07 stand-in; vision is the only horizontal velocity input) | velocity-leg tracking RMSE ≤ 0.15 m/s per axis in steady segments; stability as P07; response to a weak-texture segment recorded as a characterization (legacy: no failsafe) |
| Performance | wall-time processing rate and CPU of the odometry, image rate, and real-time factor per profile, reported separately from sim-time correctness |
| Capability matrix | every advertised sensing/command mode marked: supported in simulation / supported with idealized input / Unsupported-Deferred (VRPN mocap, joystick teleop: USER) |

Wrappers: `reef_check.sh vision`, `reef_demo.sh vision [--gui]`. Official
results headless on an idle machine (USER, P07).

### `release` (P09)

A fresh clone and a freshly built image, following documented steps only,
reproduce a representative scenario and its scores. The run works offline
after asset setup. The capability matrix separates simulation-only,
untested, and unsupported features.
