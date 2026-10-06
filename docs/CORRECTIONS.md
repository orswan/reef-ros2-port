# Corrections register (P09.5 Modernization)

The tag `sim-baseline-v0.1.0` (`main` `7590a97`) froze the faithful baseline:
the ports reproduce the pinned originals bit for bit, legacy defects
included. This document is the register of deliberate departures from that
baseline on `main`.

**Revision 2 (2026-10-06)**, after the independent Codex design review of
revision 1 ([reviews/Codex_Phase1_Review.md](reviews/Codex_Phase1_Review.md),
resolutions in
[reviews/Codex_Phase1_Resolutions.md](reviews/Codex_Phase1_Resolutions.md)).
The review's verdict was "revise the design before implementation"; the
architecture below is the revision. Nothing here is implemented yet.

**USER decisions (2026-10-06).**
- Corrections are **on by default**: the shipped configuration is the safest
  version, because the goal is physical flight. Legacy behaviour stays
  reachable through per-correction toggles, used by the parity runs.
- **Every correction has a toggle, without exception** (including C10), so
  legacy parity is reproducible without caveats.
- The degraded-flight **mechanism** is built now; its **terminal action** is a
  parameter, `bounded_descent` in simulation, with the hardware default
  gated on P10 firmware verification.
- Timestamp and derivative **reseeding** is pulled into phase 1 (KC5b, with
  its own toggle); K1 and K2 stay in phase 2.

## 1. Rules

1. **ID and toggle.** `C*` estimator, `KC*` controller (numbered by the
   K-item it fixes), `NC*` chain-level behaviour with no legacy counterpart.
   One boolean parameter per correction, default `true`; `false` reproduces
   the legacy path exactly. No exceptions.
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
7. **Layering** (review §layering, accepted). Node and adapter layers own
   clocks, transport validation and periodic scheduling. A ROS-free
   supervisor owns capability decisions and transitions, and receives ages,
   validity flags and reset epochs — never a ROS clock. Callbacks that other
   threads may run only set flags.
8. **Nothing returns "zero" as a safety action.** Zero thrust removes lift;
   zero is a numerical default, not a safe actuator policy. Safe behaviour is
   decided by the supervisor (NC1c) and enforced at the sink (NC3).

## 2. Status

| ID | Toggle | Fixes | Phase | State |
|---|---|---|---|---|
| C1 | `correction_c1_clear_xy_flag` | D1 (XY flag never cleared) | pre-P09.5 | **implemented**, default on since R1 |
| NC3 | `correction_nc3_command_validation` | no validation of the outgoing command | 1 | proposed (rev 2) |
| KC6 | `correction_kc6_finite_guard` | K6 (non-finite state latched) | 1 | proposed (revised) |
| KC5a | `correction_kc5a_readiness_gate` | K5 (no output inhibition) | 1 | proposed (revised) |
| KC5b | `correction_kc5b_reseed_timing` | K3, K4 at enable/recovery only | 1 | proposed (rev 2, USER) |
| C8 | `correction_c8_gap_handling` | unbounded and unaccounted propagation gaps | 1 | proposed (**premise corrected**) |
| C7a | `correction_c7a_imu_validity` | D8 and the infinity gap in the acceleration guard | 1 | proposed (revised) |
| C7b | `correction_c7b_normalize_attitude` | unnormalized quaternions | 1 | proposed |
| C9 | `correction_c9_observation_validity` | gate accepts non-finite D², no measurement or covariance validation | 1 | proposed (revised) |
| C10 | `correction_c10_covariance_contract` | covariance parameters and runtime `S` not definiteness-checked | 1 | proposed (toggle added, USER) |
| NC1a | `correction_nc1a_command_supervisor` | commands exist only while estimates arrive | 1 | proposed (rev 2) |
| NC1b | `correction_nc1b_input_health` | no health or capability contract on any input | 1 | proposed (rev 2) |
| NC1c | `correction_nc1c_degraded_policy` | no degraded state, no escalation | 1 | proposed (rev 2) |
| NC2 | `correction_nc2_setpoint_health` | no setpoint freshness or validity check | 1 | proposed (revised) |
| KC1, KC2 | — | K1 (D-term anti-damping), K2 (anti-windup) | 2 | not yet specified; **flight-safety issues** (review) |
| KC3, KC4 | — | K3, K4 beyond KC5b's enable-time reseeding | 2 | not yet specified |
| KC9, KC11 | — | K9 (heading wrap), K11 (resets) | 2 | not yet specified |
| C2–C6 | — | D2, D3, D4, D6, takeoff detection | 2 or later | deferred since R1 |
| D5 | — | XY gates use the previous `R` | 2 | **newly registered** (review); needs its own decision |
| R2 follow-ups | — | STATUS item 28 | 2 | deferred from P09 |
| vision world XML | — | STATUS item 29 | 2 | deferred from P09 |

Phase 1 is therefore **14 toggles** (12 new plus C1 and, for counting,
C7b). Test coverage is per-toggle plus profiles plus a named interaction set
(§5), not the cross product.

## 3. Phase 1 architecture

The review's central finding: **command continuity, input health and
degraded policy are three different concerns**, and revision 1 conflated them
into one age check inside a callback that does not run when the input stops.

```
            ┌──────────────── node / adapter layer (ROS, clocks, timers) ────────────────┐
estimate ──►│ receipt-time stamping, ordering and epoch checks ──► ages + validity flags │
setpoint ──►│                                                                            │
pose     ──►│ NC1a periodic scheduler at command_rate_hz (default 50 Hz)                 │
status   ──►│                                                                            │
            └────────────────────────────────┬───────────────────────────────────────────┘
                                             ▼
            ┌──────── ROS-free supervisor (NC1b health, NC1c policy) ────────┐
            │ capability set: {horizontal, altitude, attitude, setpoint}     │
            │ state: NORMAL → DEGRADED(capability) → TERMINAL(action)        │
            └────────────────────────────────┬───────────────────────────────┘
                                             ▼
                        controller core (KC5a gate, KC5b seeding, KC6 guards)
                                             ▼
                        NC3 outgoing-command validation ──► command sink
```

A separate `controller_health` topic carries the supervisor's state. **Not**
`status`: [V] `status` is the controller's arming *input*, published by the
stand-in and read in
[controller.cpp:58-62](../src/reef_control/src/controller.cpp#L58-L62);
publishing degradation there would collide with that authority.

### NC1a — command continuity (`correction_nc1a_command_supervisor`)

**Defect.** [V] `computeCommand()` is called only from
[controller.cpp:40-49](../src/reef_control/src/controller.cpp#L40-L49), the
estimate callback. When estimates stop, commands stop. The stand-in's
`offboard_timeout_ms` is 100 ms
([x3_standin.yaml:24](../src/reef_fc_standin/config/x3_standin.yaml#L24)),
so the vehicle is dropped 100 ms later, long before revision 1's proposed
200 ms age threshold would have reacted. Detection must not be slower than
the watchdog it is protecting against.

**Proposed fix.** The node runs a periodic scheduler at `command_rate_hz`
(default 50 Hz, a 5× margin on the 100 ms watchdog). Each tick, if the
estimate-driven path has not published within one period, the scheduler
publishes the supervisor's current command. In NORMAL state that is the last
computed command, republished unchanged; in DEGRADED or TERMINAL it is the
policy's command. Publication is never skipped while armed.

**Expected result.** Command age at the stand-in never exceeds
2 / `command_rate_hz` while armed, regardless of estimate arrival. A total
estimator loss produces continuous commands and no offboard timeout.

**Tests.** Node test: stop the estimate stream, assert continuous publication
at the scheduled rate and `offboard_timeouts == 0`; `dropout_long` and
`controller_restart` scenarios.

**Parity impact.** None with the toggle off (no timer is created).

### NC1b — input health and capability (`correction_nc1b_input_health`)

**Defect.** No input has a health contract. [V] Revision 1's publication-age
check could not detect `velocity_loss`, where the estimator publishes at
250 Hz while its horizontal observations are gone — the review's "fresh
publication does not establish usable state".

**Proposed fix.** Each required input gets: monotonic **receipt** age (for
liveness), source-stamp age (for measurement age, after validating the clock
relationship), an ordering and duplicate policy, and a reset/startup epoch.
Required inputs are **mode-dependent**: velocity mode needs estimate and
setpoint; position mode also needs pose; every mode needs arming status.
Validity means finite, in-range, and ordered — not merely recent.

From these, the supervisor derives a **capability set**:

| Capability | Lost when |
|---|---|
| `setpoint` | setpoint stale, invalid, or from a stale epoch |
| `horizontal` | no usable horizontal velocity observation fused for `horizontal_timeout_s` (C9 provides the last-fused time per source) |
| `altitude` | no usable range or altitude observation fused for `altitude_timeout_s`, or the z state is invalid |
| `attitude` | attitude input non-finite or stale ([A] on hardware this comes from the flight controller; in simulation it is idealized) |
| `estimate` | no estimate received for `estimate_timeout_s`, or the estimate is invalid |

Thresholds are parameters with **no default claimed as flight-safe**:
`estimate_timeout_s` 0.2, `setpoint_timeout_s` 0.5, `horizontal_timeout_s`
1.0, `altitude_timeout_s` 1.0 are initial test candidates, to be set from
detection delay, actuation delay, speed and clearance (review §2), not from
nominal latency. The register records them as candidates until measured.

**Expected result.** `velocity_loss` is detected as a loss of `horizontal`
within `horizontal_timeout_s`, while the estimate stream is still fresh.

**Tests.** Supervisor unit tests over synthetic age/validity/epoch
sequences, including future stamps, duplicates, out-of-order arrival, and a
ROS-time pause and backward jump ([clock design](https://design.ros2.org/articles/clock_and_time.html));
`velocity_loss`, `range_loss`, `estimator_reset` scenarios.

**Parity impact.** None with the toggle off (the supervisor is not consulted).

### NC1c — degraded policy and escalation (`correction_nc1c_degraded_policy`)

**Defect.** There is no degraded state at all: the controller either acts on
whatever it has or stops existing.

**Proposed fix.** A state machine, capability-driven, with an explicitly
bounded bridge and a parameterized terminal action (USER decision: build the
mechanism, gate the hardware default on P10).

| State | Entry | Command |
|---|---|---|
| NORMAL | all capabilities for the active mode | the controller's output |
| DEGRADED(`setpoint`) | setpoint lost (NC2) | hold position intent: zero horizontal velocity, zero yaw rate, last valid altitude setpoint |
| DEGRADED(`horizontal`) | horizontal capability lost | horizontal authority **removed**: level attitude command, zero yaw rate; altitude control retained while `altitude` holds |
| BRIDGE | `estimate` or `altitude` lost | the last **validated** command, held for at most `bridge_max_s` (candidate 0.3 s). Bounded by construction: expiry forces TERMINAL |
| TERMINAL | bridge expired, or attitude lost | `degraded_terminal_action`: `bounded_descent` (simulation default), `hold`, or `handoff` (hand authority to the flight controller; **requires P10**) |

`bounded_descent` commands level attitude and a descent rate from the
characterized thrust map, bounded by `descent_rate_max`; it is a
*simulation* default and is labelled as such, because the hardware thrust
contract is unverified (STATUS item 22). Last-thrust hold is **not** approved
as an altitude guarantee (review §1): it is only the BRIDGE content, with an
expiry.

**Recovery** (review §6, accepted) requires sustained health: an exit age
threshold below the entry threshold, at least `recover_samples` distinct
advancing samples (candidate 5–10) over at least `recover_duration_s`
(candidate 0.05–0.1 s), reset-epoch agreement, KC5b reseeding, and a bounded
transition back to normal commands. TERMINAL does **not** auto-resume the
interrupted mission when observations return.

**Open [A].** `handoff` assumes the flight controller keeps healthy attitude
stabilization when REEF fails. That is unverified (STATUS item 22) and is why
it is not a default.

**Expected result.** `dropout_long`: no crash — attitude within 0.1 rad of
level, descent within `descent_rate_max`, vehicle intact. `setpoint_stale`:
horizontal speed ≤ 0.1 m/s within 1 s, altitude held within 0.1 m.
`velocity_loss`: commanded horizontal velocity ≤ 0.05 m/s after the
transition (drift itself REPORTED until measured; revision 1's 0.5 m claim
had no mechanism behind it).

**Tests.** Supervisor state-machine unit tests for every transition,
including alternating good/bad samples, burst recovery, delayed queues,
restart while armed, and simultaneous estimate and setpoint loss; the P07b
scenarios re-scored both ways.

**Parity impact.** None with the toggle off; the legacy crash outcomes stay
reproducible and keep their characterizations.

### NC2 — setpoint health (`correction_nc2_setpoint_health`)

**Defect.** [V] `desired_state` is stored unconditionally
([controller.cpp:35-38](../src/reef_control/src/controller.cpp#L35-L38)); a
dead source leaves the last command in force. Observed: a 6 s stale setpoint
keeps the vehicle moving (P07b `setpoint_stale`).

**Proposed fix.** Setpoint health feeds NC1b: receipt age, stamp age,
ordering, finiteness and range, plus mode consistency (a position setpoint in
velocity mode is invalid). Loss enters DEGRADED(`setpoint`).

**Expected result and tests.** As in NC1c's table; unit tests for stale,
non-finite, out-of-order, mode-mismatched and duplicate setpoints.

**Parity impact.** None with the toggle off.

### NC3 — outgoing command validation (`correction_nc3_command_validation`)

**Defect.** [V] Nothing validates the command that leaves the controller. The
attitude branch copies `desired_state_.attitude` straight through with no
clamp ([controller.cpp:117-122](../src/reef_control/src/controller.cpp#L117-L122)),
unlike the velocity branch above it, and `std::min`/`std::max` pass NaN
through, so `command.F` can be NaN
([controller.cpp:105](../src/reef_control/src/controller.cpp#L105)).

**Proposed fix.** One validation point at the sink, over the **complete**
command: mode is a known value; `ignore` bits are consistent with the mode;
every field that the mode uses is finite and within its limit; fields the
mode ignores are set to a defined value rather than stale data (K7 stays
otherwise). A command that fails is not published; the supervisor is told,
which escalates per NC1c rather than silently dropping commands (dropping is
what the 100 ms watchdog punishes).

**Expected result.** No NaN, no out-of-limit and no unknown-mode command can
reach the sink, on any path, including attitude mode.

**Tests.** Table-driven sink tests per mode and per field; a fuzz test over
the command struct; assertion that every scenario's recorded commands are
valid.

**Parity impact.** None with the toggle off. With it on, K8's
"attitude-mode angles are not clamped" characterization keeps asserting the
legacy path with the toggle off and gains a corrected counterpart.

### KC6 — non-finite containment (`correction_kc6_finite_guard`)

**Defect** (K6), with the review's correction to its scope. [V] The 4-argument
overload stores the state before returning
([simple_pid.cpp:66-71](../src/reef_control/src/simple_pid.cpp#L66-L71)), and
the 3-argument overload updates `differentiator_` from `x - last_state_`
([simple_pid.cpp:45](../src/reef_control/src/simple_pid.cpp#L45)) — the
**state**, not the error. So a non-finite **state** poisons the
differentiator permanently, while a non-finite **setpoint** with a finite
state does not. Revision 1's "every non-finite error latches" was too broad.

**Proposed fix.** Validate inputs **before either overload mutates any
member**; compute candidate updates and commit them only if finite; count
rejected samples. Add a finite check before the clamps, since comparison
clamps pass NaN. Guard against overflow from finite-but-extreme inputs.
Returning 0 stays the *numerical* result of a rejected step; it is **not** an
actuator policy — persistent rejection raises a capability loss to NC1b, and
NC3 is the final gate.

**Expected result.** A single non-finite state costs one control step, with
automatic recovery on the next finite sample; a non-finite setpoint never
touches the differentiator; no NaN reaches a motor command. Permanent
latching stays reproducible with the toggle off.

**Tests.** Unit tests across the matrix (NaN/inf in state, setpoint, supplied
derivative) × (`kd > 0`, `kd = 0`) × (both overloads); a new `check_faults.py`
case injecting one non-finite estimate followed by valid data.

**Parity impact.** None with the toggle off.

### KC5a — readiness gate (`correction_kc5a_readiness_gate`)

**Defect** (K5). Commands are published from the first estimate on, armed or
not.

**Proposed fix.** Three distinct situations (review §3, accepted):

| Situation | Behaviour |
|---|---|
| positively disarmed | publish nothing; prevent entry into autonomous control where the interface allows it |
| armed, not ready (no valid required inputs for the mode, or epoch mismatch) | **not** silence: the supervisor's DEGRADED/TERMINAL command, because silence while armed is what the watchdog punishes |
| flying, readiness lost | NC1c degradation, never a publication stop |

"Ready" means every mode-required input is fresh **and** valid **and**
ordered **and** from the current epoch — not merely received once.
Arming status itself carries a freshness contract (NC1b).

**Expected result.** Zero commands before arming in every scenario; nominal
closed-loop results unchanged within run-to-run noise.

**Tests.** Node tests for arm, disarm, rearm, restart-while-armed; nominal
and causality runs re-scored.

**Parity impact.** None with the toggle off.

### KC5b — enable-time reseeding (`correction_kc5b_reseed_timing`)

**Defect** (K3, K4 at the moments that matter for phase 1). [V] `last_state_`
starts at 0 and the first step uses `dt = stamp − 0`. Clearing integrators
does not fix the derivative kick or the first-dt behaviour, so a correction
that re-enables control after degradation would reintroduce them at every
recovery (review §3).

**Proposed fix.** When control is enabled or resumed — arming, readiness
gained, recovery from DEGRADED/TERMINAL — seed the control timestamp from the
current sample and the derivative history from the current state, rather than
from 0. Scope is **enable and resume transitions only**; the general K3/K4
behaviour stays for phase 2, so this is not a tuning change in steady state.

**Expected result.** No derivative kick and no huge first dt at any enable or
recovery; steady-state output unchanged.

**Tests.** Unit tests on the first step after each transition; a scenario
that degrades and recovers twice, asserting no command spike (bounded by the
existing attitude limits) at either recovery.

**Parity impact.** None with the toggle off. With it on, the K3 and K4
characterizations keep asserting the legacy path in legacy mode.

### C7a — IMU validity (`correction_c7a_imu_validity`)

**Defect** (D8, plus the review's infinity finding). [V] The guard is
`std::isnan(getVectorMagnitude(ax, ay, az))`
([xyz_estimator.cpp:131](../src/reef_estimator/src/xyz_estimator.cpp#L131))
over `sqrt(x*x + y*y + z*z)`
([xyz_estimator.cpp:535-538](../src/reef_estimator/src/xyz_estimator.cpp#L535-L538)):
an **infinite** component yields `inf`, and `isnan(inf)` is false, so the
sample is accepted with an infinite acceleration. The attitude quaternion is
not checked at all, and the initialization path
([xyz_estimator.cpp:116](../src/reef_estimator/src/xyz_estimator.cpp#L116))
averages samples with no validity check either.

**Proposed fix.** Reject a sample unless every acceleration component is
finite and within a physical range, and the quaternion is finite with norm in
`1 ± quaternion_norm_tol` (candidate 1e-3). Apply the same validity rule on
the initialization path. Keep the existing throttled report, and report the
reason. Rejection feeds C8's gap handling and NC1b's capability set.

**Expected result.** NaN, ±inf, zero-norm and norm-1.1 inputs are each
rejected with the state unchanged and the reason reported; initialization
cannot be poisoned.

**Tests.** Fixture cases per failure mode, on both the running and the
initialization paths.

**Parity impact.** None with the toggle off.

### C7b — attitude normalization (`correction_c7b_normalize_attitude`)

**Defect.** The quaternion is used unnormalized, so a slightly non-unit
attitude scales the rotation matrix.

**Proposed fix.** Normalize after the C7a validity check. Separate toggle
because, unlike C7a, this **changes the numbers for valid input** and is
therefore scored against the independent model rather than master.

**Tests.** Fixture with a 1.01-norm quaternion: output matches the
independent model with the toggle on, and master with it off.

### C8 — propagation gaps (`correction_c8_gap_handling`)

**Premise corrected.** Revision 1 called the post-skip double `dt` the defect
and proposed advancing `last_time_stamp` on rejection. The review is right
that this is wrong, and the code confirms it: [V] the NaN branch returns
before `last_time_stamp` is updated, so the next accepted sample's `dt` is
the **true elapsed time** since the last propagation. Advancing the timestamp
would have deleted real time from the filter. The hazards are different:

1. [V] `dt` is unbounded: a long gap propagates in one step.
2. [V] XY process noise does not grow with the gap as a double integrator's
   should: `xyEst.Q` is scaled once at initialization by the nominal `dt²`
   ([xyz_estimator.cpp:71](../src/reef_estimator/src/xyz_estimator.cpp#L71))
   while the propagation adds `G*Q*G'*dt` linearly
   ([xy_estimator.cpp:123](../src/reef_estimator/src/xy_estimator.cpp#L123)).
   For z, `updateLinearModel()` rebuilds `F` and `Q = Q0*dt` from the live
   `dt` each step
   ([xyz_estimator.cpp:167](../src/reef_estimator/src/xyz_estimator.cpp#L167)),
   so z is consistent and XY is not.
3. One acceleration sample is applied across the whole gap (zero-order hold),
   which is defensible but must be bounded.

**Proposed fix.** Keep the elapsed time. Bound `dt` to `max_propagation_dt`
(candidate 10 × `estimator_dt`). Within the bound, scale the XY process-noise
contribution by the actual interval so uncertainty grows with the gap. Beyond
the bound, do not propagate: mark the affected state invalid, report it, and
require reinitialization — which NC1b sees as a lost capability. Never move
the reference stamp backward and never accept a future stamp as the new
baseline.

**Expected result.** After a skipped sample the next step integrates the true
elapsed interval **and** the covariance grows accordingly; a gap beyond the
bound invalidates rather than silently propagating. The `s11` legacy
regression keeps asserting the legacy behaviour with the toggle off.

**Tests.** Fixtures: one skipped sample (dt sequence and covariance growth);
50 consecutive skips (covariance must **grow** — revision 1's "no blow-up"
criterion was backwards); a gap beyond `max_propagation_dt` (invalidation);
backward and future stamps.

**Parity impact.** None with the toggle off.

### C9 — observation and gate validity (`correction_c9_observation_validity`)

**Defect** (R1 finding 3, with the review's additions). [V] The gates are
`if (D² > threshold) reject`
([xyz_estimator.cpp:346](../src/reef_estimator/src/xyz_estimator.cpp#L346),
`:373`, `:402`, `:426`), so a non-finite `D²` takes the accept branch when
execution reaches the comparison. Neither the measurement nor its covariance
is validated, and `S.inverse()` is used explicitly with no factorization
check.

**Proposed fix.**
- Validate the measurement (finite, in range) and its covariance (finite,
  symmetric, positive definite within a scale-aware tolerance) **per source**,
  before computing `D²`.
- Validate `S = HPH' + R`: positive definite and acceptably conditioned, via
  a **checked solve** (LDLᵀ/LLᵀ with success inspected) instead of
  `S.inverse()`.
- Require `D²` finite **and** within the threshold to accept.
- Account per source: last valid **received**, last actually **fused**,
  invalid-data rejections separated from innovation outliers. NC1b consumes
  these for capability loss; there is no universal rejection count (review
  §4: five rejects mean different outages at 20 Hz and 250 Hz, and silence
  produces no rejects at all).

Rationale correction: zero measurement variance is rejected as a **sensor
contract violation**, not because `S` is necessarily singular — the review is
right that `S = HPH' + R` can stay invertible with `R = 0`.

**Out of scope, newly registered:** D5, the XY gates using the previous
`R` (the incoming covariance is installed after acceptance,
[xyz_estimator.cpp:228](../src/reef_estimator/src/xyz_estimator.cpp#L228)).
Validating the incoming covariance does not change that coupling; correcting
it needs its own decision and is listed in §2 for phase 2.

**Expected result.** Per source and per fault (NaN or inf measurement, NaN
covariance, zero variance, negative variance, indefinite covariance,
ill-conditioned `S`): the observation makes **no measurement-update
contribution**, identical to an otherwise identical trajectory with the
observation absent, and the rejection is counted and attributed.

**Tests.** Fixtures per source × fault; a solve-failure case; comparison
against an observation-absent trajectory rather than a bare "state
unchanged" assertion.

**Parity impact.** None with the toggle off, which keeps master's
accept-on-non-finite behaviour — the behaviour the 253/253 parity run uses.

### C10 — covariance contract (`correction_c10_covariance_contract`)

**Defect** (R1 finding 4). [V] `covarianceError` checks square, finite,
exactly symmetric and non-negative diagonal
([matrix_operation.h:136-154](../src/reef_msgs/include/reef_msgs/matrix_operation.h#L136-L154));
an indefinite matrix passes all four.

**Proposed fix** (review §5, accepted; **toggled**, per the USER's decision
that the workflow has no exceptions):
- `P0`, `Q`: require positive **semi**definiteness, with scale-aware
  tolerance and an inspected eigensolver status. Intentional singularity is
  allowed; warn where it matters operationally.
- Measurement `R`: require strict positive definiteness, as a sensor
  contract.
- Runtime `S`: handled in C9 (definiteness and conditioning before a solve).

With the toggle off, the legacy acceptance is restored exactly, so legacy
fixtures and the parity runs need no caveat.

**Expected result.** An indefinite `P0`/`Q` or a singular `R` stops startup
with a message naming the parameter and the offending eigenvalue; every
shipped YAML still starts; a singular `P0` is accepted with a warning.

**Tests.** `test_ros_parameters` cases: indefinite, singular, valid, per
parameter class; a negative control over every shipped YAML; toggle-off cases
asserting the legacy acceptance.

**Parity impact.** None: validation precedes any estimate, and all legacy
fixtures use valid covariances. The toggle makes that provable rather than
argued.

## 4. Landing order

Dependencies, from the review and from the architecture above:

1. **NC3** — the backstop, so nothing later can emit an invalid command.
2. **KC6** — before KC5, so the guards exist when the gate starts enabling
   control.
3. **KC5a**, then **KC5b** — the gate, then the seeding its transitions need.
4. **C8** — before C7a, because C7a increases the rejection rate and C8 is
   what makes a rejection safe.
5. **C7a**, then **C7b**.
6. **C9**, then **C10**.
7. **NC1a** → **NC1b** → **NC1c** → **NC2** — the supervisor last, since it
   consumes C9's fused-time accounting and KC5/KC6's transitions.

One correction per commit, each with its before/after record.

## 5. Interaction tests (not the cross product)

Per-toggle both states, plus the all-off (legacy parity) and all-on (shipped
default) profiles, plus these named combinations, which the review identified
as the ones that actually interact:

- **C7a × C8** — rejection rate against gap handling.
- **C9 × C1** — rejection accounting against the XY flag correction.
- **KC5a × KC6** — enabling control while inputs are non-finite.
- **KC5b × NC1c** — reseeding on recovery from each degraded state.
- **NC1b × NC2** — simultaneous sensing and setpoint loss.
- **NC1a × NC3** — a validation failure during scheduled publication (the
  case where "don't publish" meets "never go silent while armed").
- **C10 × C9** — a parameter accepted at startup that later yields an
  ill-conditioned `S`.

Transition coverage: startup, arm, disarm, rearm, restart while armed,
degradation, recovery, double degradation, ROS-time pause and backward jump.

## 6. What phase 1 still excludes

- **K1 and K2** (D-term anti-damping, ineffective anti-windup). The review
  calls them flight-safety issues and we agree; they stay in phase 2 (USER)
  because correcting them changes the closed-loop response and the simulation
  gains in `reef_control_x3_sim.yaml` were tuned with both defects present.
  Phase 2 therefore owns re-tuning and new acceptance numbers.
- **K3/K4 in general**, beyond KC5b's enable-time reseeding.
- **C2–C6** and **D5**, which change the filter's numbers rather than its
  behaviour on invalid input.
- **Hardware behaviour (P10):** the firmware command contract, `MIN_THROTTLE`,
  RC override, the real failsafe, and therefore the `handoff` terminal
  action. This is also why no threshold or terminal default here is claimed
  as flight-validated.
