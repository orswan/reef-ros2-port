# Corrections register (P09.5 Modernization)

The tag `sim-baseline-v0.1.0` (`main` `7590a97`) froze the faithful baseline:
the ports reproduce the pinned originals bit for bit, legacy defects
included. This document is the register of deliberate departures from that
baseline on `main`.

**Revision 4 (2026-10-06)**, after three independent Codex review passes
([pass 1](reviews/Codex_Phase1_Review.md),
[pass 2](reviews/Codex_Phase1_Review_V2.md),
[pass 3, the targeted confirmation](reviews/Codex_Phase1_Review_V3.md);
resolutions for all three in
[Codex_Phase1_Resolutions.md](reviews/Codex_Phase1_Resolutions.md)). Pass 3
**withheld final confirmation**: contract B1 was accepted for simulation,
while D1 required amendment ("losing arming-status messages during
established flight must not immediately cut thrust") and contracts A, C and E
needed reconciliation. Revision 4 carries those amendments; they are
contract changes inside the agreed architecture, and pass 3 states no wider
architectural review is needed for them. The amendments are listed for
sign-off in
[Codex_Phase1_Amendments.md](reviews/Codex_Phase1_Amendments.md).
**Nothing is implemented yet.**

**USER decisions (2026-10-06).**
1. Corrections are **on by default in the shipped configuration**; the legacy
   path is reachable only through each correction's named toggle, which the
   parity runs use.
2. **Every correction has a toggle**, C10 included — no governance exception.
3. **Staged activation** (new in rev 3): each correction lands with its toggle
   **default false**; one capstone commit flips the phase-1 defaults to true
   once the whole set is complete and scored end to end. Partially assembled
   defaults are never exposed.
4. **Terminal policy** is a model-based **descent command** with bounded
   command values and the resulting speed **REPORTED, not bounded**: a
   descent rate cannot be guaranteed without healthy vertical state. `hold`
   is defined as frozen thrust and is **explicitly not an acceptable terminal
   policy**. Contact and disarm authority are defined without truth data.
5. **C8 drops the process-noise rescaling.** It bounds `dt` and defines the
   outage boundary only; the XY process-noise discretization becomes its own
   registered research item (C11). The legacy mathematics is not guessed at.
6. Timestamp and derivative **reseeding** is in phase 1 (KC5b); K1 and K2 stay
   in phase 2.
7. The third review pass is a **targeted confirmation** of the fallback
   contracts, not another architectural review.

**Review-driven amendments carried in rev 4** (pass 3): the arming contract
is split by history, so a status dropout in established flight no longer cuts
thrust; UNCONTAINED suppresses publication and is recorded as an uncontained
failure; termination publishes a single validated transition command rather
than an indefinite minimum-thrust stream; the landing detector is a
range-based **criterion**, not proof of contact; `descent_max_s` is an
emergency-action budget, not an inferred time to landing; the timing budget
scores the observed receiver gap and assumes a two-timer-period bound; clock
discontinuities have five explicit rules; delivered rotor force is
distinguished from requested collective force; and activating the simulation
descent default requires a comparison against command suppression.

## 1. Rules

1. **ID and toggle.** `C*` estimator, `KC*` controller (numbered by the
   K-item it fixes), `NC*` chain-level or interface behaviour with no legacy
   counterpart. One boolean parameter per correction; `false` reproduces the
   legacy path exactly. No exceptions.
2. **Default state is phase-dependent** (USER 3): `false` while the phase is
   being built, flipped to `true` for the shipped configuration by the
   phase's capstone commit, which is also where the end-to-end scoring lands.
3. **Approval before code.** A register entry here plus criteria in
   ACCEPTANCE.md, both approved, precede the implementation.
4. **Parity is never traded away.** With every toggle off, `check_port.py`
   (253/253) and `check_control.py` (106/106) stay bit-identical to the
   originals. A correction that cannot be switched off is rejected. Note that
   replacing an explicit inverse with a factorized solve changes results at
   the bit level, so such a change always needs its toggle-off path to keep
   the legacy expression (C9a, C9b).
5. **Legacy tests and goldens are immutable.** They keep asserting the legacy
   path with the toggles off. A correction adds tests; it never edits theirs.
6. **No hidden dependencies.** A safety correction may not rely on a test
   hook, on simulation truth, or on anything unavailable on hardware. Truth
   may **score** a fallback; it may never **drive** one.
7. **One correction per commit**, with its before/after record.
8. **Layering.** Node and adapter layers own clocks, transport validation and
   periodic scheduling. A ROS-free supervisor owns capability decisions and
   transitions and receives ages, validity flags and epochs — never a clock.
   Callbacks other threads may run only set flags.
9. **Zero is never a safety action.** Zero thrust removes lift. A rejected
   numerical step returns zero *and* propagates its rejection status; the
   command is decided by the supervisor and enforced at the sink.
10. **No claim without an established basis.** Pass 2 rejected two of our
    claims (the XY noise defect, a frequency ratio presented as a timing
    margin). A quantitative claim needs its units, assumptions and
    derivation, or it is recorded as REPORTED and unproven.

## 2. Status

Phase 1 is **15 corrections, 15 toggles** (plus C1, already shipped).

| ID | Toggle | Fixes | Phase | State |
|---|---|---|---|---|
| C1 | `correction_c1_clear_xy_flag` | D1 (XY flag never cleared) | pre-P09.5 | **implemented**, default on since R1 |
| NC4 | `correction_nc4_estimator_health` | no usable health interface (diagnostics are ~1 Hz) | 1 | proposed (rev 3) |
| NC3 | `correction_nc3_command_validation` | outgoing command never validated | 1 | proposed |
| KC6 | `correction_kc6_finite_guard` | K6, plus rejection status never propagated | 1 | proposed (rev 3) |
| KC5a | `correction_kc5a_readiness_gate` | K5, plus unknown arming status | 1 | proposed (rev 3) |
| KC5b | `correction_kc5b_transition_seeding` | K3, K4 at authority transitions | 1 | proposed (rev 3) |
| C8 | `correction_c8_dt_bound` | unbounded propagation `dt`; no outage boundary | 1 | proposed (**scope cut**, rev 3) |
| C7a | `correction_c7a_imu_validity` | D8 and the infinity gap | 1 | proposed |
| C7b | `correction_c7b_normalize_attitude` | unnormalized quaternions | 1 | proposed (fixtures fixed, rev 3) |
| C9a | `correction_c9a_gate_validity` | gate accepts non-finite `D²`; no measurement or covariance validation | 1 | proposed |
| C9b | `correction_c9b_fusion_validity` | `update()`/`partialUpdate()` use unchecked inverses | 1 | proposed (rev 3) |
| C10 | `correction_c10_covariance_contract` | covariance parameters not definiteness-checked | 1 | proposed |
| NC1a | `correction_nc1a_command_supervisor` | commands exist only while estimates arrive | 1 | proposed |
| NC1b | `correction_nc1b_input_health` | no health or capability contract on any input | 1 | proposed |
| NC1c | `correction_nc1c_degraded_policy` | no degraded state, no arbitration, no escalation | 1 | proposed (rev 3) |
| NC2 | `correction_nc2_setpoint_health` | no setpoint freshness or validity check | 1 | proposed |
| C11 | — | XY process-noise discretization: units, stochastic model and discretization undocumented | research | **registered** (rev 3, USER 5) |
| NC1a-I | — | the stand-in reports command **source age**, not receiver inter-arrival gap | 1 | **instrumentation task** (rev 4); needs `reef_check.sh control` |
| — | — | descent versus command-suppression comparison within a stated envelope | 1 | **evidence task** (rev 4), gates the simulation default |
| KC1, KC2 | — | K1, K2 | 2 | flight-safety issues (both passes); phase 2 with re-tuning |
| KC3, KC4 | — | K3, K4 in steady state | 2 | not specified |
| KC9, KC11 | — | K9, K11 | 2 | not specified |
| C2–C6 | — | D2, D3, D4, D6, takeoff detection | 2+ | deferred since R1 |
| D5 | — | XY gates use the previous `R` | 2 | registered; pass 2 permits it as a documented **gate approximation** provided C9b validates the fusion |
| controller death | — | NC1a cannot cover its own process restart | P10 | registered (rev 3) |
| range-only altitude fallback | — | independent vertical fallback | later | deferred, both passes |
| IMU-based contact detection | — | contact without range or truth | later | registered (rev 3) |
| R2 follow-ups, vision XML | — | STATUS items 28, 29 | 2 | deferred from P09 |

## 3. Phase 1 architecture

Three separate concerns — command continuity, input health, degraded policy —
plus a single arbitration point and a single validation gate.

```
      ┌──────────── node / adapter layer (ROS types, clocks, timers) ────────────┐
      │ receipt-time stamping, ordering, duplicates, epochs, PAUSE detection     │
      │ NC1a scheduler at command_rate_hz ── one arbitration call per publication │
      └───────────────────────────────┬──────────────────────────────────────────┘
                                      ▼
      ┌──────── ROS-free supervisor (ages, flags, epochs in; decision out) ──────┐
      │ NC1b capability set {estimate, horizontal, altitude, attitude, setpoint} │
      │ NC1c state: NORMAL → DEGRADED(cap) → BRIDGE → TERMINAL → UNCONTAINED     │
      │             plus ARMING_UNKNOWN, entered from any state                  │
      └───────────────────────────────┬──────────────────────────────────────────┘
                                      ▼
            controller core (KC5a gate, KC5b seeding, KC6 guards + rejection)
                                      ▼
            NC3 validation (one gate, all paths) ──► sink ──► stand-in (100 ms)
```

**Single arbitration** (pass 2): every publication path, timer or callback,
takes exactly one supervisor decision per published command. A fresh estimate
callback cannot overwrite a latched TERMINAL or UNCONTAINED command.

Health is published on `controller_health` (supervisor) and
`estimator_health` (NC4). Never on `status`, which [V] is the controller's
arming **input** ([controller.cpp:58-62](../src/reef_control/src/controller.cpp#L58-L62)).

### NC4 — estimator health interface (`correction_nc4_estimator_health`)

**Defect.** [V] The only health-ish output is `diagnostics`, published once
per 250 IMU callbacks — about 1 Hz
([sensor_manager.cpp:221-224](../src/reef_estimator/src/sensor_manager.cpp#L221-L224)).
Pass 2: safety must not depend on that cadence, and nothing today reports
fused-observation age, epoch or state validity.

**Proposed fix.** A dedicated `estimator_health` message from the estimator
**node** (the core stays ROS-free), published at `health_rate_hz` (candidate
50) and on every validity or epoch change, carrying: per-source age of the
last **fused** observation (not received), per-source rejection counts split
into invalid-data and innovation-outlier, the estimator epoch (incremented on
reset or reinitialization), per-state validity flags, and the message's own
stamp plus a validity horizon so a consumer can expire it.

**Repeated fusion of one old observation must not refresh health** (pass 2):
the age reported is the age of the observation's own stamp, not of the fusion
event. This is what makes D1's re-fusion and the C1 correction visible to the
supervisor.

**Expected result.** NC1b can distinguish "publishing but blind"
(`velocity_loss`) from "silent" (`dropout_long`) from "restarted"
(`estimator_reset`) using only this interface.

**Tests.** Node tests over synthetic fusion sequences, including re-fusion of
one observation (age must keep growing), reset (epoch must change), and
expiry when the estimator stops. Scenario assertions in `velocity_loss`,
`range_loss`, `estimator_reset`.

**Parity impact.** None with the toggle off: no publisher is created and the
core is untouched.

### NC1a — command continuity (`correction_nc1a_command_supervisor`)

**Defect.** [V] `computeCommand()` is called only from the estimate callback
([controller.cpp:40-49](../src/reef_control/src/controller.cpp#L40-L49)), so
commands stop when estimates stop, and the stand-in drops the vehicle
`offboard_timeout_ms` = 100 ms later
([x3_standin.yaml:24](../src/reef_fc_standin/config/x3_standin.yaml#L24)).

**Proposed fix.** A node timer at `command_rate_hz` publishes the
supervisor's arbitrated command whenever the estimate-driven path has not
published within one period.

**Timing budget, amended (pass 3).** Rev 3's budget double-counted and
under-bounded. Two accounting corrections:

1. A receiver inter-command gap measures the **whole publication and
   transport path**, so it must not be added to the timer and processing
   terms. The **scored** quantity is the observed receiver gap against
   `T_watchdog − M`; the separately measured components (timer period,
   scheduling jitter, processing, transport) only *explain* the budget.
2. A periodic check that republishes once the command age exceeds one period
   permits nearly **two timer periods** between commands. That two-period
   bound is the design figure unless the implementation schedules against an
   exact publication deadline, which is the preferred option.

```
observed receiver inter-command gap  <  T_watchdog − M        (scored)
T_watchdog = 100 ms;  design bound on the gap = 2 / command_rate_hz
M = a stated FIXED reserve, justified by worst observed gaps and stress tests
```

`M` is a fixed reserve for the stated simulation envelope, not a percentile:
a percentile leaves the tail unprotected. [A] 20 ms is an initial experiment,
not evidence of adequacy on this host. A passing run establishes empirical
margin, never a scheduling guarantee on a non-real-time host.

**Instrumentation task (new, rev 4).** [V] The stand-in's `cmd_age` is
`now − command.header.stamp`
([standin_node.cpp:193](../src/reef_fc_standin/src/standin_node.cpp#L193),
stamp captured at [:107](../src/reef_fc_standin/src/standin_node.cpp#L107)) —
**source age, not receiver inter-arrival gap**. Receipt-time instrumentation
must be added to the stand-in, preserving the existing source-age field.
Changing `reef_fc_standin` requires `reef_check.sh control` (AGENTS).

**Scope limit** (pass 2, unchanged). NC1a covers **estimator loss while the
controller is alive**. It cannot cover its own process death, so
`controller_restart` (3.7 s) is out of scope and registered for P10.

**Tests.** Estimate stream stopped while armed; receiver-gap distribution
recorded and scored; stress runs under adverse scheduling; `dropout_long`.

**Parity impact.** None with the toggle off (no timer).

### NC1b — input health and capability (`correction_nc1b_input_health`)

**Defect.** No input has a health contract; publication freshness cannot see
`velocity_loss`, where the estimator publishes at 250 Hz with no usable
horizontal observations.

**Proposed fix.** Per input: monotonic **receipt** age for liveness,
validated source-stamp age for measurement age, ordering and duplicate
policy, and a reset/startup epoch. Required inputs are **mode-dependent**.
Capabilities derive from NC4's health message plus these ages:

| Capability | Lost when |
|---|---|
| `estimate` | no estimate received within `estimate_timeout_s`, or state validity false, or epoch unqualified |
| `horizontal` | no horizontal observation **fused** within `horizontal_timeout_s` (NC4 reports it) |
| `altitude` | no altitude observation fused within `altitude_timeout_s`, or z invalid |
| `attitude` | **two distinct cases**: loss of the *companion attitude input* while firmware stabilization survives — level commands remain meaningful; versus *flight-controller attitude failure* — level commands are meaningless, and this is UNCONTAINED. Absent attitude-control authority may not select a level-attitude action (pass 3) |
| `setpoint` | NC2 |

**Clock discontinuity rules, amended (pass 3).** A stopped clock alone cannot
distinguish a pause from a failure, so the adapter supplies the distinction
explicitly:

| Situation | Rule |
|---|---|
| **Confirmed pause** | freeze simulation-action time and simulation-input ages **only** with explicit, fresh pause confirmation |
| **Clock stopped, pause not confirmed** | classify clock liveness as unknown or failed using monotonic time; freshness is **not** suspended indefinitely |
| **Large forward discontinuity** | expire overdue inputs and absolute action deadlines **immediately**; do not integrate the jump and do not emit catch-up commands; requalify affected state and epochs |
| **Backward jump** | invalidate the epoch and require readiness again |
| **Normal positive advancement** | process normally within the timing and outage bounds |
| **Resume** | reject pre-pause queued data that fails ordering or freshness |

[V] `pause_resume` pauses Gazebo for 2 s of **wall** time
([pause_resume.yaml](../src/reef_sim/config/closed_loop/pause_resume.yaml)),
which a naive monotonic timeout reports as input failure; hence the
confirmation requirement rather than a blanket freeze.

**Thresholds.** Candidates only, REPORTED with measured margins, derived per
failure class from permitted speed, tilt, altitude and obstacle proximity;
sensor, health-transport, scheduling and actuator delays; estimation and
missing-input uncertainty; available braking or vertical authority; and the
whole trajectory through detection, BRIDGE and TERMINAL. Vertical clearance
is assessed separately from horizontal stopping distance, and levelling
removes commanded acceleration without braking existing velocity.

**Tests.** Synthetic age/validity/epoch sequences (future stamps, duplicates,
out-of-order, confirmed pause, unconfirmed clock stop, forward jump, backward
jump, resume with stale queue); fault injection at several speeds and flight
phases; adverse scheduling and transport.

**Parity impact.** None with the toggle off.

### NC1c — degraded policy, arbitration and escalation (`correction_nc1c_degraded_policy`)

A capability-driven state machine with explicit precedence, one arbitration
point, a deterministic fallback table, and a bounded non-recursive
invalid-command path.

**Fallback table (new, rev 4; pass 3 B2).** Selection is deterministic:
capability state in, exactly one action out. A prerequisite that is not met
disqualifies the action, and the next eligible row is taken.

| Capabilities lost | Action | Prerequisites | Command fields and thrust source | Expiry |
|---|---|---|---|---|
| none | NORMAL | all mode-required inputs fresh, valid, ordered, current epoch | controller output | — |
| `setpoint` | DEGRADED(setpoint) | `estimate`, `altitude`, `attitude` held | zero horizontal velocity, zero yaw rate, altitude from the last valid setpoint | none (steady) |
| `horizontal` | DEGRADED(horizontal) | `altitude`, `attitude` held | level attitude, zero yaw rate, thrust from the altitude controller | none (steady) |
| `altitude` | BRIDGE(altitude) | `attitude` held; a last thrust **validated as non-saturated and inside a stated band** | level attitude, zero yaw rate, that banded thrust | `bridge_max_s`, absolute |
| `estimate` | BRIDGE(estimate) | as above | as above | `bridge_max_s`, absolute |
| bridge expired, or `estimate`+`altitude` together | TERMINAL | `attitude` held | `degraded_terminal_action` | `descent_max_s`, absolute |
| `attitude` (flight-controller failure), or no eligible row, or the fallback fails validation | UNCONTAINED | — | **publication suppressed** | latched |

Precedence for simultaneous losses: attitude > estimate > altitude >
horizontal > setpoint, escalating **directly** without traversing
intermediate states. An expiry is **absolute**: it cannot be refreshed by
repeated bad packets, a changing failure reason, republication, or capability
flicker.

**Single arbitration.** Every publication path — timer or callback — takes
exactly one supervisor decision per published command. A fresh estimate
callback cannot overwrite a latched TERMINAL or UNCONTAINED command.

**Terminal action** (USER): `descent_command`, a model-based descent command
with bounded command values and the resulting speed **REPORTED, never
bounded**. `handoff` requires P10 and a failed handoff falls to UNCONTAINED.
`hold` means frozen thrust, is **rejected** as a terminal policy, and a
configuration naming it is refused at startup.

**Requested force versus delivered force, amended (pass 3 A2).** Bounding the
thrust *command* does not bound delivered force. [V] The stand-in allocates
collective thrust and attitude torques and then clamps **each rotor force**
to `[0, f_max]`
([standin.cpp:64-71](../src/reef_fc_standin/src/standin.cpp#L64-L71)), so
clipping changes the delivered collective force and torques; and because the
torques follow attitude error
([:61-63](../src/reef_fc_standin/src/standin.cpp#L61-L63)), a low collective
command can still yield nonzero rotor thrust. The contract therefore
distinguishes **requested collective force** from **delivered rotor forces**,
and every descent run reports rotor saturation and the actual attitude
response throughout.

**Evidence required before activation, amended (pass 3 A1).** Characterizing
the descent does not establish "better than nothing". Within a stated finite
envelope and duration, each descent case is recorded **alongside the
command-suppression case from equivalent initial conditions**: initial
height, vertical and horizontal velocity, tilt, failure cause; commands,
actual attitude, actuator saturation, trajectory, termination time, disarm
acknowledgment; peak descent speed, touchdown speed, horizontal displacement,
contact outcome; and the conditions under which descent **worsens** the
outcome or never reaches the landing criterion. "No commanded climb" is an
intent statement, not a guarantee of monotonic descent — a descending thrust
command may initially accompany upward motion. The mechanism may be
implemented before this evidence exists; the simulation **default** is
activated only after it, and the word "safer" is used only with the
comparison in hand.

**Range-based landing criterion, amended (pass 3 C3).** Not a proof of
contact, and renamed accordingly. It is a safety-critical landing detector
with prerequisites: a valid, fresh range from the designated source, with
known sensor offset and usable surface and tilt geometry; **distinct
advancing observations over a minimum dwell**, never repeated fusion of one
reading; a valid rate estimate; detector reset after invalid or gapped
observations; and entry **only** during landing or terminal descent. The
constrained simulation envelope states the maximum residual height and drop
accepted when zero thrust is requested. Tests: low stationary hover near the
ground, a frozen range reading, and a false close surface.

**Termination, amended (pass 3 B1/C1/C2).**

- With the landing criterion met: command zero collective thrust, publish the
  **disarm request**, and remain terminal until a **fresh DISARMED status**
  confirms the result; otherwise report failure to acknowledge. Zero
  collective does **not** prove the motors stopped (attitude torque
  allocation can still produce rotor thrust), so motor shutdown is confirmed
  after the stand-in accepts the disarm.
- With no vertical state: `descent_max_s` is a **maximum emergency-action
  budget**, chosen from the permitted flight envelope — **not** an inferred
  time to landing. A recent qualified last-known altitude may refine it only
  with a stated age horizon and uncertainty growth, and must never be
  *required* to enter emergency descent, since it may be exactly what was
  lost. Expiry means "the supported emergency-action budget ended", not
  "landing completed".
- A separate timeout covers **failure to obtain the landing criterion while
  `altitude` is nominally healthy**.
- At timed termination the minimum-thrust command is a **single validated
  transition command**, followed by suppression — never indefinite
  minimum-thrust publication — and it is omitted entirely if it fails
  validation. The outcome is recorded as UNCONTAINED.

**Disarm request interface.** Its recipient is defined; acknowledgment is a
fresh status; retries are bounded; and it takes **priority over a scenario
runner's arm request**, so a competing publisher cannot undo terminal
disarming. Publication of the request is not completion.

**UNCONTAINED, accepted for simulation (pass 3 B1).** When neither the normal
candidate nor an eligible fallback passes validation: suppress actuator-command
publication, latch UNCONTAINED, and **continue health reporting**. Do not
revive a cached command because it once passed validation. Cancel scheduled
republication, and test that queued or in-flight commands cannot restart an
indefinite stream. This lets the stand-in's watchdog expire; that is an
**uncontained failure**, not a safe landing and not a guaranteed motor
shutdown. It is a simulation policy only: P10 must establish what silence
actually does on hardware — RC fallback, mode transitions, firmware
failsafe — and the policy does not transfer automatically.

**Invalid-command path.** Validate the normal candidate at the single gate;
on failure latch the reason and select a capability-compatible fallback from
the table; validate it at the **same** gate; on failure invoke the terminal
response and record UNCONTAINED. No recursion and no bouncing between
validator and supervisor. Fallback parameters are validated **at startup**,
and failure of the fallback itself is tested.

**Recovery.** Exit age below entry age; `recover_samples` distinct advancing
samples over `recover_duration_s`; epoch agreement; KC5b reseeding; bounded
transition. TERMINAL and UNCONTAINED never auto-resume the mission, and a
late queued command never regains authority after disarm.

**Tests.** Every transition and precedence combination; the fallback table
row by row including disqualified prerequisites; alternating good/bad
samples; burst recovery; delayed queues; simultaneous losses; fallback
parameter failure; both candidates invalid; disarm during each state; descent
versus suppression comparisons; the P07b scenarios both ways.

**Parity impact.** None with the toggle off; the legacy crash outcomes stay
reproducible with their characterizations.

### NC2 — setpoint health (`correction_nc2_setpoint_health`)

**Defect.** [V] `desired_state` is stored unconditionally
([controller.cpp:35-38](../src/reef_control/src/controller.cpp#L35-L38)); a
6 s stale setpoint keeps the vehicle moving (P07b `setpoint_stale`).

**Proposed fix.** Receipt age, stamp age, ordering, duplicates, finiteness,
range and **mode consistency** feed NC1b; loss enters DEGRADED(`setpoint`).

**Expected result and tests.** Per NC1c's table; unit tests for stale,
non-finite, out-of-order, duplicate and mode-mismatched setpoints.

**Parity impact.** None with the toggle off.

### NC3 — outgoing command validation (`correction_nc3_command_validation`)

**Defect.** [V] Nothing validates the outgoing command. The attitude branch
copies `desired_state_.attitude` with no clamp
([controller.cpp:117-122](../src/reef_control/src/controller.cpp#L117-L122)),
unlike the velocity branch, and `std::min`/`std::max` pass NaN, so
`command.F` can be NaN
([controller.cpp:105](../src/reef_control/src/controller.cpp#L105)).

**Proposed fix.** One gate, on every path: known mode; `ignore` bits
consistent with the mode; every used field finite and within limit; ignored
fields set to a defined value; no serialization overflow in the float fields.
Failure enters the bounded invalid-command path above — it never silently
drops a command and never publishes an invalid one.

**Expected result.** No NaN, out-of-limit, unknown-mode or overflowing
command reaches the sink on any path, attitude mode included.

**Tests.** Per-mode table tests; a fuzz test over the command struct; an
assertion over every scenario's recorded commands; the both-candidates-invalid
case.

**Parity impact.** None with the toggle off; K8's "attitude angles are not
clamped" characterization keeps asserting the legacy path.

### KC6 — non-finite containment and rejection status (`correction_kc6_finite_guard`)

**Defect** (K6, scope corrected in rev 2 and extended in rev 3). [V] The
4-argument overload stores state before returning
([simple_pid.cpp:66-71](../src/reef_control/src/simple_pid.cpp#L66-L71)); the
3-argument overload updates `differentiator_` from `x - last_state_`
([simple_pid.cpp:45](../src/reef_control/src/simple_pid.cpp#L45)), the state
rather than the error, so a non-finite **state** latches permanently while a
non-finite **setpoint** with a finite state does not.

**Proposed fix.** Validate before **either** overload mutates a member;
compute candidate updates and commit only if finite; finite check before the
clamps; overflow guard for finite-but-extreme inputs. **Propagate rejection
status** into command assembly (pass 2): a numerical zero accompanied only by
a counter can still become an unintended zero-thrust command, so the
assembling code receives the rejection for that step and the supervisor
decides.

**Expected result.** One non-finite state costs one control step with
automatic recovery; a non-finite setpoint never touches the differentiator;
no NaN and no unintended zero-thrust command reaches the sink.

**Tests.** (NaN, ±inf) × (state, setpoint, supplied derivative) × (`kd > 0`,
`kd = 0`) × both overloads; a rejection-status propagation test asserting the
command assembled during a rejected step; a fault case injecting one
non-finite estimate followed by valid data.

**Parity impact.** None with the toggle off.

### KC5a — readiness gate and arming contract (`correction_kc5a_readiness_gate`)

**Defect** (K5). Commands are published from the first estimate on, armed or
not.

**Proposed fix.** Readiness gating, plus an arming contract **split by
history** (pass 3 D1, which refused rev 3's version: withholding thrust the
moment status goes stale in established flight converts a status-channel
dropout into a deliberate loss of lift).

| Situation | Behaviour |
|---|---|
| positively disarmed, fresh status | publish nothing; block entry into autonomous control where the interface allows |
| status unknown, **never confirmed armed** or last confirmed **disarmed** | publish no flight commands; do not initiate autonomous thrust |
| status unknown, **previously confirmed armed with active control** | **block mission continuation** and enter a bounded emergency policy, continuing while commands remain valid and the required capabilities survive. This preserves previously established emergency authority; it grants no new normal-flight authority from a stale status |
| emergency budget expired, or no valid fallback | UNCONTAINED, with the publication-suppression policy |
| armed, fresh status, not ready | the supervisor's arbitrated command — not silence, which the watchdog punishes |
| flying, readiness lost | NC1c degradation, never a publication stop |

**Leaving uncertainty** requires **fresh authoritative status**, not a fresh
*positive* one (pass 3): a fresh DISARMED report resolves the uncertainty and
**immediately terminates publication**; a fresh ARMED report permits
requalification but must not automatically resume a latched terminal mission.

**Status freshness horizon (D2).** The ~10 Hz rate gives an expected period,
not a deadline:

```
T_status = k * T_period + J_status + D_transport
```

with an explicit allowed missed-message count `k` and a measured delay
budget. [A] Three periods plus a measured allowance is an initial simulation
candidate, not an approved safety limit. Source age and receipt liveness are
measured **separately**; identity, order and epoch are validated, so repeated
old "armed" messages cannot refresh trust. The status-loss deadline is kept
**separate** from the bounded emergency-continuation deadline.

"Ready" means every mode-required input is fresh **and** valid **and**
ordered **and** from the current epoch. A late queued command must not regain
authority after disarm.

**Tests.** Startup with unknown status; stale status after confirmed armed
flight (must not cut thrust); stale status never armed; fresh DISARMED during
each state; fresh ARMED after terminal; repeated stale "armed" messages;
arm, disarm, rearm; restart while armed; a queued command arriving after
disarm.

**Parity impact.** None with the toggle off.

### KC5b — authority-transition seeding (`correction_kc5b_transition_seeding`)

**Defect** (K3, K4 at transitions). [V] `last_state_` starts at 0 and the
first step uses `dt = stamp − 0`, so any re-enable reintroduces a derivative
kick and a huge first dt.

**Proposed fix.** At **every control-authority transition** — arming,
readiness gained, recovery from any degraded state, **per-axis** authority
recovery, and mode changes (pass 2) — seed the control timestamp from the
current sample and the derivative history from the current state. Define
integrator behaviour explicitly as well: frozen while authority is absent,
and reset or retained per transition type, stated per axis. Steady-state
K3/K4 behaviour is untouched (phase 2). Reseeding changes transition
trajectories even though the steady-state control law is unchanged, so it is
scored on transitions.

**Tests.** First step after each transition type, per axis; degrade and
recover twice; mode change while flying; no command spike beyond the attitude
limits.

**Parity impact.** None with the toggle off; K3 and K4 characterizations keep
asserting the legacy path.

### C7a — IMU validity (`correction_c7a_imu_validity`)

**Defect** (D8, plus the infinity finding). [V] The guard is
`std::isnan(getVectorMagnitude(ax, ay, az))`
([xyz_estimator.cpp:131](../src/reef_estimator/src/xyz_estimator.cpp#L131))
over `sqrt(x*x+y*y+z*z)`
([:535-538](../src/reef_estimator/src/xyz_estimator.cpp#L535-L538)), so an
infinite component yields `inf` and `isnan(inf)` is false. The quaternion is
unchecked, and the initialization path
([:116](../src/reef_estimator/src/xyz_estimator.cpp#L116)) averages samples
with no validity check at all.

**Proposed fix.** Reject unless every acceleration component is finite and
within a physical range **and** the quaternion is finite with norm within
`quaternion_norm_tol` (candidate 1e-3) of 1. Same rule on the initialization
path. Rejection reports its reason and feeds C8 and NC1b.

**Expected result.** NaN, ±inf, zero-norm, norm-1.01 and norm-1.1 inputs are
rejected on both paths, state unchanged, reason reported.

**Parity impact.** None with the toggle off.

### C7b — attitude normalization (`correction_c7b_normalize_attitude`)

**Proposed fix.** Normalize **after** C7a's validity check. Separate toggle
because it changes the numbers for valid input, so it is scored against the
independent model.

**Behaviour table** (pass 2; rev 2's question 6 was wrong — C7a rejects
norm 1.1 regardless of C7b, and 1.01 also fails C7a's tolerance):

| C7a | C7b | Required behaviour |
|---|---|---|
| on | on | reject invalid norm; normalize accepted input |
| on | off | reject invalid norm; use the accepted near-unit input unchanged |
| off | off | legacy path |
| off | on | explicitly defined normalization-only behaviour, **including** zero-norm and non-finite input, which must not divide by zero |

**Fixtures.** Normalization uses **norm 1.0005** (inside C7a's tolerance);
1.01 and 1.1 are **rejection** fixtures. C7a-on/C7b-off is compatibility
evidence and retains small rotation errors; it is not equivalent to the
normalized profile, and all-on is the shipped profile.

### C8 — propagation `dt` bound and outage boundary (`correction_c8_dt_bound`)

**Scope cut in rev 3** (USER 5). Rev 2 claimed an XY process-noise defect.
Pass 2 is right that the cited equations do not establish it: [V] the XY
contribution is `ΔP = G Q_xy Gᵀ Δt` with `Q_xy = Q_param h²` at the nominal
`h`, which already grows linearly with the gap, and whether additional
scaling is correct depends on what `Q_param` represents — continuous
white-noise intensity, uncertainty in an acceleration held across the gap, or
something correlated. `xy_Q` carries no documented units or stochastic
assumptions ([estimator_master.yaml:32](../src/reef_estimator/config/estimator_master.yaml#L32)).
**The rescaling is removed from C8** and registered as C11.

Rev 1's premise was also wrong, as rev 2 recorded: [V] the NaN branch returns
before `last_time_stamp` is updated, so the post-skip `dt` is true elapsed
time, and advancing the stamp would delete real time.

**Proposed fix, now narrow.**
- Bound the single integration step: `dt > max_propagation_dt` is not
  integrated as one step. Within an admissible gap, **substeps** address
  numerical integration error; they do not reconstruct missing acceleration
  or attitude, and this is stated rather than implied.
- Define the **outage boundary** separately from the step limit (pass 2: a
  small integration-step limit need not equal the maximum permissible
  outage). Beyond the outage boundary, three distinct actions:
  1. mark the state **unusable for control** (NC1b sees a lost capability);
  2. maintain or propagate the internal estimate only where justified;
  3. **reacquire and requalify** before authority is restored.
- **No blind reinitialization** (pass 2): resetting airborne altitude,
  velocity, bias, covariance or takeoff state to startup defaults can create
  a plausible but wrong estimate and an immediate control transient. Any
  reinitialization is **mode-aware** and must not re-enter ground calibration
  while airborne — which also avoids depending on C4 (D4, "cannot start in
  the air"), still deferred.
- Never move the reference stamp backward; never accept a future stamp as the
  new baseline.

**Expected result.** A long gap is never integrated as one unbounded step;
the elapsed time is preserved; beyond the outage boundary the state is marked
unusable and requalified rather than silently trusted or blindly reset.
`s11` keeps asserting the legacy behaviour with the toggle off.

**Tests.** One skipped sample (dt sequence unchanged in value, substep count
asserted); a gap beyond `max_propagation_dt` within the outage boundary
(substeps, state still usable); a gap beyond the outage boundary (marked
unusable, requalification required, no ground calibration while airborne);
backward and future stamps.

**Parity impact.** None with the toggle off.

### C9a — gate validity (`correction_c9a_gate_validity`)

**Defect** (R1 finding 3). [V] All four gates are `if (D² > threshold)
reject` ([xyz_estimator.cpp:346](../src/reef_estimator/src/xyz_estimator.cpp#L346),
`:373`, `:402`, `:426`), so a non-finite `D²` takes the accept branch when
execution reaches the comparison; the measurement and covariance are
unvalidated, and `S.inverse()` is explicit.

**Proposed fix.** Validate the measurement (finite, in range) and its
covariance (finite, symmetric, positive definite, scale-aware tolerance) per
source; compute `D²` through a **checked solve** whose factorization status is
inspected; require `D²` finite **and** within the threshold to accept; count
and attribute rejections, separating invalid data from innovation outliers,
and report them through NC4. No universal rejection count (pass 1: five
rejects mean different outages at 20 Hz and 250 Hz, and silence produces no
rejects at all).

Zero measurement variance is rejected as a **sensor-contract violation**, not
because `S` is necessarily singular.

**Parity note.** A solve is not bit-identical to an inverse multiply, so the
toggle-off path keeps the legacy expression exactly (rule 4).

**Tests.** Per source × (NaN, ±inf measurement; NaN covariance; zero
variance; negative variance; indefinite covariance; ill-conditioned `S`),
judged against an otherwise identical trajectory with the observation
**absent**.

### C9b — fusion validity (`correction_c9b_fusion_validity`)

**Defect** (pass 2, newly found and verified here). [V]
`Estimator::update()` and `partialUpdate()` compute
`K = P Hᵀ S⁻¹` with unchecked `S.inverse()`
([estimator.cpp:33](../src/reef_estimator/src/estimator.cpp#L33),
[:47](../src/reef_estimator/src/estimator.cpp#L47)). A checked **gate** solve
does not protect this: the fusion runs later, with a different `S`.

**Proposed fix.** Validate the fusion solve: factorize `S` with an inspected
status, compute the candidate state and covariance, and **commit only if**
the candidate is finite and the covariance stays symmetric and PSD within
tolerance; otherwise reject the fusion, report it, and leave the state
untouched. Pass 2 permits D5 (the gate's use of the previous `R`) to remain a
documented **gate approximation** provided the fusion itself is validated.

**Parity note.** As C9a: the toggle-off path keeps `S.inverse()`.

**Tests.** Fixtures driving a singular and an ill-conditioned fusion `S`; a
case where the candidate covariance loses PSD; assertion that a rejected
fusion leaves state and covariance bit-identical to the no-observation
trajectory.

### C10 — covariance contract (`correction_c10_covariance_contract`)

**Defect** (R1 finding 4). [V] `covarianceError` checks square, finite,
exactly symmetric and non-negative diagonal
([matrix_operation.h:136-154](../src/reef_msgs/include/reef_msgs/matrix_operation.h#L136-L154));
an indefinite matrix passes.

**Proposed fix.** `P0`, `Q`: positive **semi**definite, scale-aware
tolerance, inspected eigensolver status, intentional singularity allowed and
warned where operationally relevant. Measurement `R`: strictly positive
definite as a sensor contract. Runtime `S`: C9a and C9b. Toggled, per USER 2.

**Tests.** Indefinite, singular and valid cases per parameter class; a
negative control over every shipped YAML; toggle-off cases asserting legacy
acceptance.

## 4. Landing order and staged activation

Pass 2: NC3 and KC5a cannot exercise their specified fallback before NC1c
exists, and partially assembled defaults must not ship. With USER 3's staged
activation, every correction below lands with its toggle **default false**:

1. **NC4** — the health interface everything else reads.
2. **NC1a** → **NC1b** → **NC1c** — scheduler, health, supervisor. The
   supervisor exists before anything delegates to it.
3. **NC3** — the single validation gate, now able to use the invalid-command
   path.
4. **KC6** → **KC5a** → **KC5b** — guards, then the gate, then transition
   seeding.
5. **C8** → **C7a** → **C7b** — gap handling before the corrections that
   raise the rejection rate.
6. **C9a** → **C9b** → **C10**.
7. **NC2**.
8. **Capstone commit**: flip the phase-1 defaults to true, run the full
   corrected-profile scoring end to end, and record the before/after.

## 5. Interaction tests (not the cross product)

Per toggle in both states; the all-off (legacy parity) and all-on (shipped)
profiles; and: C7a×C8, C9a×C1, C9a×C9b, C9b×C10, KC5a×KC6, KC5b×NC1c,
NC1b×NC2, NC1a×NC3, NC4×NC1b (stale or expired health), NC1c×KC5a (disarm
during each state).

Transitions: startup, arm, disarm, rearm, restart while armed, degradation,
recovery, double degradation, simultaneous losses with precedence, ROS-time
pause, backward time jump, queued command after disarm.

## 6. Still excluded from phase 1

- **K1, K2** and the general K3/K4 behaviour: phase 2, with re-tuning, since
  `reef_control_x3_sim.yaml` was tuned with both defects present. Both review
  passes call K1/K2 flight-safety issues; the deferral is a scope decision
  (USER), not a judgement that they are benign.
- **C11** (XY process-noise discretization) — research: establish `Q_param`'s
  units, stochastic assumptions and discretization before changing any
  equation. Pass 2 also notes z's live-`dt` scaling does not prove z's
  missing-input model is adequate.
- **C2–C6**, **D5** (permitted as a documented gate approximation),
  **KC9/KC11**.
- **Controller-process death** (`controller_restart`): needs a surviving
  component or firmware fallback; registered for P10.
- **Range-only altitude fallback** and **IMU-based contact detection**.
- **Hardware behaviour (P10)**: the firmware command contract,
  `MIN_THROTTLE`, RC override, the real failsafe, `handoff`, and any hardware
  output path, which the sink refuses until an approved contract exists.
