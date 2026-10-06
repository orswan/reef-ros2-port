# Corrections register (P09.5 Modernization)

The tag `sim-baseline-v0.1.0` (`main` `7590a97`) froze the faithful baseline:
the ports reproduce the pinned originals bit for bit, legacy defects
included. This document is the register of deliberate departures from that
baseline on `main`.

**USER decisions (2026-10-06).** Corrections are **on by default**: the
shipped configuration is the safest version, because the goal is physical
flight. Legacy behaviour stays reachable through per-correction toggles, used
by the parity runs and nothing else. An independent review (Codex) reviews
phase 1 before any code is written
([reviews/Codex_Phase1_Packet.md](reviews/Codex_Phase1_Packet.md)).

## 1. Rules

1. **ID and toggle.** `C*` estimator, `KC*` controller (the number is the
   K-item it fixes), `NC*` chain-level behaviour with no legacy counterpart.
   One boolean parameter per correction, default `true`; `false` reproduces
   the legacy path exactly.
2. **Approval before code.** A register entry here plus criteria in
   ACCEPTANCE.md, both approved, precede the implementation.
3. **Parity is never traded away.** With every toggle off, `check_port.py`
   (253/253) and `check_control.py` (106/106) stay bit-identical to the
   originals. A correction that cannot be switched off is rejected.
4. **Legacy tests and goldens are immutable.** They keep asserting the legacy
   path with the toggles off. A correction adds tests; it never edits theirs.
5. **No hidden dependencies.** A safety correction may not rely on a test
   hook, on simulation truth, or on anything unavailable on hardware.
6. **One correction per commit**, with its before/after record. A correction
   that changes tuning says so in the same commit.

## 2. Status

| ID | Fixes | Phase | State |
|---|---|---|---|
| C1 | D1 (XY flag never cleared) | pre-P09.5 | **implemented**, default on since R1 |
| NC1 | no estimate freshness check | 1 | proposed |
| NC2 | no setpoint freshness check | 1 | proposed |
| KC5 | K5 (no output inhibition) | 1 | proposed |
| KC6 | K6 (non-finite state latched) | 1 | proposed |
| C7 | D8 (attitude never NaN-checked) | 1 | proposed |
| C8 | D9 (double dt after a skipped sample) | 1 | proposed |
| C9 | gate accepts a non-finite D² or covariance | 1 | proposed |
| C10 | covariance parameters not PSD-checked | 1 | proposed |
| KC1 | K1 (D term anti-damping) | 2 | not yet specified |
| KC2 | K2 (ineffective anti-windup) | 2 | not yet specified |
| KC3, KC4 | K3, K4 (derivative kick, first dt) | 2 | not yet specified |
| KC9 | K9 (heading without wrapping) | 2 | not yet specified |
| KC11 | K11 (no integrator or differentiator resets) | 2 | not yet specified |
| C2–C6 | D2, D3, D4, D6, takeoff detection | 2 or later | deferred since R1 |
| R2 follow-ups | STATUS item 28 | 2 | deferred from P09 |
| vision world XML | STATUS item 29 | 2 | deferred from P09 |

## 3. Phase 1: safety

Phase 1 takes the four failure classes where the legacy system has no
protection at all and the consequence is loss of the vehicle. It deliberately
excludes anything that changes control tuning; that is phase 2.

### NC1 — estimate freshness (`correction_nc1_estimate_timeout`)

**Defect.** The controller acts on whatever estimate it holds, with no notion
of age ([control_node.cpp:328-332](../src/reef_control/src/control_node.cpp#L328-L332)
stores `last_estimate_stamp_`, but nothing reads it for a freshness decision;
it is only copied into the outgoing command's header).
There is no legacy counterpart: ROS 1 REEF had none either.

**Failure scenario** [V] (P07b, STATUS item 27). When REEF stops publishing,
the controller keeps integrating against a frozen state. In
`dropout_long` the vehicle crashes; in `velocity_loss` it follows REEF's
drifting estimate 2.7 m in 10 s.

**Proposed fix.** The node computes the estimate age from its own clock and
passes it to the core; the core decides. Age > `estimate_timeout_s`
(proposed 0.2 s, ≈ 50 estimate periods at 250 Hz) enters a **degraded
state**: level attitude, yaw rate 0, and the altitude channel holding the
last valid thrust rather than integrating. Recovery requires a fresh
estimate; entering and leaving the state is published on `status` and logged
once per transition.

**Open question for review.** What to command while degraded. Inhibiting
publication is *not* safe here: any command gap > 100 ms drops the vehicle
through the stand-in (STATUS item 27), so the correction must keep publishing
a defined safe command. Candidates: hold last thrust (proposed), a
parameterized descent, or level-and-hold-altitude using range only. The
choice needs the reviewer's judgement, since it decides what happens on
hardware when the estimator dies in flight.

**Expected result.** `dropout_long` no longer crashes: attitude stays within
0.1 rad of level and the vehicle descends no faster than 1 m/s.
`velocity_loss` drift drops from 2.7 m to under 0.5 m in 10 s.

**Tests.** A core unit test driving ages across the threshold; the P07b
`dropout_long` and `velocity_loss` scenarios re-scored with the correction on
and, unchanged, with it off.

**Parity impact.** None with the toggle off (the core never consults age).

### NC2 — setpoint freshness (`correction_nc2_setpoint_timeout`)

**Defect.** `desired_state` has no age check, so a dead setpoint source
leaves the last command in force indefinitely.

**Failure scenario** [V] (P07b `setpoint_stale`, STATUS item 27): a 6 s stale
setpoint keeps the vehicle moving at the last commanded velocity.

**Proposed fix.** Age > `setpoint_timeout_s` (proposed 0.5 s) replaces the
setpoint with zero horizontal velocity, zero yaw rate and the altitude of the
last valid setpoint — stop and hold, not land, since a stale setpoint says
nothing about the estimator. Published on `status`.

**Expected result.** `setpoint_stale`: horizontal speed below 0.1 m/s within
1 s of the timeout, altitude held within 0.1 m, no crash.

**Tests.** Core unit test; the `setpoint_stale` scenario both ways.

**Parity impact.** None with the toggle off.

### KC5 — output inhibition (`correction_kc5_inhibit_output`)

**Defect** (K5). Commands are published from the first estimate on, armed or
not; the only inhibition is the firmware's.

**Failure scenario.** On hardware, a controller that publishes before arming
relies entirely on the firmware's arming logic; a mux or mode bug then has a
live command available. In simulation the stand-in's arming hides it.

**Proposed fix.** Publish nothing until all of: armed, one estimate inside
`estimate_timeout_s`, one setpoint inside `setpoint_timeout_s`. Integrators
stay cleared while inhibited, which also removes the first-sample windup that
K3/K4 cause on the first armed step.

**Open question for review.** Whether inhibition should publish *nothing* or
an explicit zero/safe command. With the stand-in, publishing nothing before
arming is safe (no offboard timeout while disarmed), but the hardware
contract is unverified until P10 (STATUS item 22).

**Expected result.** No command before arming in any scenario; nominal
closed-loop results unchanged within run-to-run noise (26/26 still PASS).

**Tests.** A node-level test asserting zero published commands before arming;
nominal and causality runs re-scored.

**Parity impact.** None with the toggle off.

### KC6 — non-finite containment (`correction_kc6_finite_guard`)

**Defect** (K6). A non-finite error returns 0 but **stores** the non-finite
state ([simple_pid.cpp:66-71](../src/reef_control/src/simple_pid.cpp#L66-L71)
sets `last_error_` and `last_state_` before returning). With `kd > 0` the
differentiator stays NaN permanently and the output becomes NaN; `std::min`
and `std::max` pass NaN through, so the saturated output is NaN too.

**Failure scenario.** One NaN in the estimate — a divergent filter, a bad
sensor frame, a missing field — permanently destroys every channel that uses
a derivative. The vehicle is lost with no recovery path.

**Proposed fix.** On a non-finite input: do not store it, do not integrate,
return 0, and count the event. Clamp with explicit comparisons so a NaN can
never reach the output. Recovery is automatic on the next finite sample. A
non-finite *output*, if one is ever computed, is replaced by 0 and reported.

**Expected result.** A single NaN estimate costs one control step. After it,
outputs are finite and tracking resumes; a NaN never reaches the motor
command. New fault case: inject one NaN, then 5 s of valid data, and require
finite commands throughout and tracking recovered within 1 s.

**Tests.** Core unit tests (NaN in state, in setpoint, in derivative, with
`kd > 0` and `kd = 0`); a new fault case in `check_faults.py`.

**Parity impact.** None with the toggle off. With it on, `check_control.py`'s
K6 characterization must still pass in legacy mode and gains a corrected-mode
counterpart.

### C7 — attitude validity (`correction_c7_check_attitude`)

**Defect** (D8). Only the acceleration is NaN-checked
([xyz_estimator.cpp:131](../src/reef_estimator/src/xyz_estimator.cpp#L131));
a NaN or non-unit quaternion propagates into `C_NED_to_body_frame` and
poisons the state.

**Failure scenario.** A NaN attitude makes every subsequent estimate NaN,
which then reaches the controller (see KC6) and the motors.

**Proposed fix.** Extend the existing IMU guard: reject the sample if the
quaternion is non-finite or its norm is outside `1 ± 1e-3`, with the same
throttled log and the same "no state change" semantics as the NaN
acceleration path. Normalize valid quaternions (currently used unnormalized).

**Expected result.** A NaN or degenerate attitude skips the sample instead of
destroying the filter; the estimate stays finite across the event.

**Tests.** Fixture cases: NaN quaternion, zero quaternion, norm 1.1. Assert
finite state and that the skip is reported.

**Parity impact.** None with the toggle off. **Note:** normalization changes
the numbers for *valid but unnormalized* input, so it is a separate toggle
(`correction_c7_normalize_attitude`) scored against the independent model.

### C8 — dt continuity (`correction_c8_dt_after_skip`)

**Defect** (D9). A skipped sample does not advance `last_time_stamp`, so the
next accepted sample integrates over a double dt.

**Failure scenario.** Every skip — now more frequent with C7 — injects a
propagation step with twice the true interval, inflating the covariance and
the bias estimate. Consecutive skips scale it linearly.

**Proposed fix.** Advance `last_time_stamp` whenever a sample is rejected, so
dt always measures the interval actually integrated. Reject dt outside
`(0, max_dt]` (proposed `max_dt` = 10 × `estimator_dt`) and report it instead
of propagating a wild interval.

**Expected result.** After a skip the next dt equals the true sample
interval. The s11 legacy regression keeps asserting the double dt with the
toggle off.

**Tests.** Fixture case with one skipped sample, asserting the dt sequence;
a case with 50 consecutive skips asserting no covariance blow-up.

**Parity impact.** None with the toggle off; `s11` is unchanged.

### C9 — observation and gate validity (`correction_c9_gate_finite`)

**Defect** (R1 finding 3). The gate is `if (D² > threshold) reject`
([xyz_estimator.cpp:346](../src/reef_estimator/src/xyz_estimator.cpp#L346),
`:373`, `:402`, and the mocap XY gate). A NaN `D²` compares false, so the
observation is **accepted**. Neither the observation nor its covariance is
checked for finiteness, zero or negative variance.

**Failure scenario.** One NaN or zero-covariance velocity message is fused
with infinite confidence, which pins or destroys the state. No fault case
covers it today (R1 finding 3), so the baseline has never been tested
against it.

**Proposed fix.** Before computing `D²`: reject the observation unless the
measurement is finite and its covariance is finite, symmetric and positive
definite within tolerance. Then require `D²` to be finite *and* within the
threshold to accept — a non-finite `D²` is a rejection, not an acceptance.
Each rejection is counted per source on `diagnostics`.

**Expected result.** A NaN observation, a zero-variance observation and a
negative-variance observation are each rejected with the state unchanged, and
the estimator keeps running on the remaining sources.

**Tests.** Fixture cases per source (range, RGB-D XY, mocap Z, mocap XY) ×
(NaN measurement, NaN covariance, zero variance, negative variance); new
`check_faults.py` cases for the two most likely on hardware.

**Parity impact.** None with the toggle off, which keeps master's
accept-on-NaN behaviour and is what the 253/253 parity run uses.

### C10 — covariance parameters PSD (`correction_c10_psd_parameters`)

**Defect** (R1 finding 4). Matrix parameters are checked for symmetry and
non-negative diagonal only
(`covarianceError`,
[matrix_operation.h:136-154](../src/reef_msgs/include/reef_msgs/matrix_operation.h#L136-L154):
square, finite, exactly symmetric, non-negative diagonal). An indefinite
matrix passes all four.

**Failure scenario.** A plausible-looking but indefinite `P0` or `Q` makes
the filter diverge or produce negative variances, with no startup error.
This is a configuration error that reaches flight.

**Proposed fix.** Add an eigenvalue check: reject any covariance parameter
whose minimum eigenvalue is below `-tol·max|P|` (proposed `tol` = 1e-12, the
tolerance already used by the invariant checks), naming the parameter and the
eigenvalue. Consistent with the existing project rule that invalid
parameters are fatal at startup, so this correction is **not** toggleable:
it only ever turns a silent acceptance into a startup error, and D10 already
established that invalid-parameter handling is a deliberate deviation.

**Open question for review.** Whether to require strict positive
definiteness (reject a singular covariance) or allow semidefiniteness. A
singular `P0` is legitimate for a perfectly known initial state; a singular
`Q` is not obviously wrong either. Proposed: require PSD, and warn on
singular.

**Expected result.** An indefinite covariance parameter stops startup with a
message naming it; every shipped configuration still starts.

**Tests.** `test_ros_parameters` cases: indefinite, singular, valid; a
negative control confirming every shipped YAML passes.

**Parity impact.** None: parameter validation runs before any estimate, and
all legacy fixtures use valid covariances.

## 4. What phase 1 deliberately excludes

- Anything that changes control tuning (KC1, KC2, KC3, KC4, KC9, KC11). The
  simulation gains in `reef_control_x3_sim.yaml` were chosen *with* the K1
  sign error and the inactive anti-windup present; correcting them changes
  the closed-loop response and requires re-tuning and new acceptance
  numbers. Phase 2 handles that as one block.
- The estimator corrections C2–C6, which change the filter's numbers rather
  than its behaviour on invalid input.
- Hardware behaviour (P10): the firmware command contract, `MIN_THROTTLE`,
  RC override and the real failsafe are unverified and out of scope here.
