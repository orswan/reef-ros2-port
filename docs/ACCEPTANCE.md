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
scored; limits unchanged):** the IMU gets 1.0 m/s² per-axis white "rotor
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
- **Horizontal velocity:** thresholds are set once the velocity input is
  defined. With idealized truth-derived velocity inputs, results must be
  labelled idealized.
- **Consistency:** finite values; covariance bounds reported. Any statistical
  consistency test (for example NEES) states its assumptions.
- **Timing:** estimate stamp age relative to the input stamp is reported, in
  sim time and in wall time.

### `faults` (P05)

A missing required stream or broken initialization gives a nonzero result or
an explicit unhealthy state, never a silent pass. Out-of-order and duplicate
measurements are rejected or handled as specified, with no double fusion.
Pause/resume and reset behave as specified. **Replay determinism (PROPOSED):**
two replays of the same bag with an ordered, single-threaded input path agree
to 1e-9 relative, or the source of any nondeterminism is documented. Live and
replay clocks never share a test scope.

### `control` (P06–P07)

To be defined with the controller contract. At minimum: the stock truth-fed
Gazebo outer controller is **disabled** in REEF-controlled runs; there is a
causality test (perturbing or removing the REEF estimate changes the control
path as specified); tracking error, overshoot, settling, saturation, and
estimate-staleness limits are fixed per scenario; there is one physics
integration and one owner per control layer.

### `vision` (P08)

Rendered RGB and depth images are processed; truth-derived velocity is not
acceptance evidence. Weak-texture, dropout, and delay cases must show
documented health and recovery. Wall-time processing rate is reported
separately from sim-time correctness.

### `release` (P09)

A fresh clone and a freshly built image, following documented steps only,
reproduce a representative scenario and its scores. The run works offline
after asset setup. The capability matrix separates simulation-only,
untested, and unsupported features.
