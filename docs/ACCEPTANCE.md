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

## 5. Future targets: criteria to be fixed before implementation

These are drafts. Items marked **PROPOSED** must be confirmed (or replaced,
with reasons) at the start of their milestone and before any port output is
scored.

### `estimator` (P04–P05): estimator quality

Scenario: the X3 scenario (or its extension), fixed seeds, sim time. Warm-up:
the settle phase plus 2 s after takeoff. Metrics per phase, in the estimator's
documented frame (NED/body-level), after conversion from truth.

- **Altitude RMSE (PROPOSED):** ≤ 0.05 m, and peak error ≤ 0.15 m outside
  takeoff/landing transients. Rationale: idealized sonar σ = 0.01 m and IMU
  σ_a = 0.02 m/s² should support centimetre-level vertical estimates; the
  margin covers lag during 0.4 m/s ramps.
- **Vertical velocity RMSE (PROPOSED):** ≤ 0.10 m/s.
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
