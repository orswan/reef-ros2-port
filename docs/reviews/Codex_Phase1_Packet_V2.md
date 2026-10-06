# Codex review packet V2: P09.5 phase 1 (safety)

**This is a second pass.** You reviewed revision 1 of this design and
returned "revise the design before implementation". Your review is in
[Codex_Phase1_Review.md](Codex_Phase1_Review.md); what we did with each
finding is in [Codex_Phase1_Resolutions.md](Codex_Phase1_Resolutions.md).
Every finding was accepted; three were narrowed with stated reasons. The
design below is the revision. **No code has been written yet.**

**What this pass asks for.** Whether the revised architecture is now sound
enough to implement, which of the six open questions in §6 you would answer
differently, and whether the narrowing in §5 is defensible. Please challenge
the revision rather than re-deriving the original findings — but say so if you
think a finding was mis-applied.

**We independently verified every code claim in your review before accepting
it.** All were confirmed. Two were extended: the IMU initialization path
([xyz_estimator.cpp:116](../../src/reef_estimator/src/xyz_estimator.cpp#L116))
averages samples with no validity check either, and the process-noise defect
behind your C8 objection is **asymmetric** between the filters — `xyEst.Q` is
scaled once at initialization by the nominal `dt²`
([xyz_estimator.cpp:71](../../src/reef_estimator/src/xyz_estimator.cpp#L71))
while the propagation adds `G*Q*G'*dt` linearly
([xy_estimator.cpp:123](../../src/reef_estimator/src/xy_estimator.cpp#L123)),
whereas z rebuilds `F` and `Q = Q0*dt` from the live `dt` every step
([xyz_estimator.cpp:167](../../src/reef_estimator/src/xyz_estimator.cpp#L167)).
So XY is under-noised across a gap and z is not.

## 1. Decisions taken since your review (fixed, not open)

| Decision | By |
|---|---|
| Build the degraded-flight **mechanism** now; the terminal action is a parameter, `bounded_descent` in simulation, hardware default gated on P10 firmware verification | USER |
| **Every** correction gets a toggle, C10 included — no governance exception | USER |
| Timestamp and derivative reseeding enters phase 1 as KC5b with its own toggle; K1 and K2 stay in phase 2 | USER |
| Corrections are on by default; legacy reachable only via toggles, which the parity runs use | USER (pre-review) |
| `handoff` to the flight controller is a parameter value, not a default, because the firmware's attitude-stabilization contract is unverified | implementer, from your [A] |
| Range-only altitude fallback is not in phase 1: by your own prerequisite list it is a separate controller needing its own validation, and may share the failure that killed the estimator | implementer |

## 2. The revised architecture

Your central finding — that command continuity, input health and degraded
policy are three concerns, and revision 1 collapsed them into one age check
inside a callback that stops running when the input stops — is the shape of
the revision.

```
            ┌──────────── node / adapter layer (ROS types, clocks, timers) ───────────┐
estimate ──►│ receipt-time stamping, ordering, duplicate and epoch checks             │
setpoint ──►│          ──► ages + validity flags + reset epochs                       │
pose     ──►│                                                                         │
status   ──►│ NC1a periodic scheduler at command_rate_hz (default 50 Hz)              │
            └───────────────────────────────┬─────────────────────────────────────────┘
                                            ▼
            ┌────── ROS-free supervisor (no clock passed in) ──────┐
            │ NC1b capability set {horizontal, altitude,           │
            │        attitude, setpoint, estimate}                 │
            │ NC1c state: NORMAL → DEGRADED(cap) → BRIDGE → TERMINAL│
            └───────────────────────────────┬─────────────────────┘
                                            ▼
                 controller core (KC5a gate, KC5b seeding, KC6 guards)
                                            ▼
                 NC3 outgoing-command validation ──► sink ──► stand-in (100 ms watchdog)
```

Degradation is published on a new `controller_health` topic, never on
`status`, which [V] is the controller's arming **input**
([controller.cpp:58-62](../../src/reef_control/src/controller.cpp#L58-L62)).

### The 13 phase-1 corrections

| ID | Toggle | What it does |
|---|---|---|
| NC1a | `correction_nc1a_command_supervisor` | node timer at `command_rate_hz` (50 Hz) republishes the supervisor's current command whenever the estimate-driven path has not published within a period; never silent while armed. Acts 5× faster than the 100 ms `offboard_timeout_ms` it protects against |
| NC1b | `correction_nc1b_input_health` | per-input monotonic **receipt** age (liveness) plus validated source-stamp age (measurement age), ordering, duplicate and epoch policy; mode-dependent required inputs; derives the capability set. Detects `velocity_loss` as lost `horizontal` while publication is still fresh |
| NC1c | `correction_nc1c_degraded_policy` | capability-driven state machine; bounded BRIDGE (`bridge_max_s`, expiry forces TERMINAL); terminal action parameter (`bounded_descent` \| `hold` \| `handoff`); recovery requires exit age below entry age, `recover_samples` distinct advancing samples over `recover_duration_s`, epoch agreement, KC5b reseeding, bounded transition; TERMINAL never auto-resumes the mission |
| NC2 | `correction_nc2_setpoint_health` | setpoint freshness, finiteness, range, ordering and mode consistency → DEGRADED(`setpoint`): zero horizontal velocity, zero yaw rate, last valid altitude |
| NC3 | `correction_nc3_command_validation` | one validation point over the **complete** outgoing command: known mode, `ignore` bits consistent with the mode, every used field finite and in limit, ignored fields set to a defined value. Failure escalates through the supervisor instead of producing silence |
| KC6 | `correction_kc6_finite_guard` | validate before **either** PID overload mutates a member; compute candidate updates, commit only if finite; finite check before the clamps; overflow guard. Zero is the numerical result of a rejected step, never an actuator policy |
| KC5a | `correction_kc5a_readiness_gate` | disarmed → publish nothing; armed-but-not-ready → the supervisor's command, not silence; flying and readiness lost → degradation, not a publication stop. "Ready" = fresh **and** valid **and** ordered **and** current epoch, per mode; arming status itself has a freshness contract |
| KC5b | `correction_kc5b_reseed_timing` | seed the control timestamp from the current sample and the derivative history from the current state at every enable and recovery; steady-state K3/K4 behaviour untouched (phase 2) |
| C8 | `correction_c8_gap_handling` | **premise replaced**: keep the elapsed time, bound `dt` to `max_propagation_dt`, scale the XY process-noise contribution by the actual interval so uncertainty grows with the gap, invalidate beyond the bound; never move the stamp backward, never accept a future stamp as the baseline |
| C7a | `correction_c7a_imu_validity` | every acceleration component finite and in range (closes the `isnan(sqrt(inf))` gap), quaternion finite with norm in 1 ± tol, on the running **and** initialization paths; rejection feeds C8 and NC1b |
| C7b | `correction_c7b_normalize_attitude` | normalize after validation; separate toggle because it changes numbers for valid input, so it is scored against the independent model |
| C9 | `correction_c9_observation_validity` | validate measurement and covariance per source; validate `S` by a **checked solve** (not `S.inverse()`) for definiteness and conditioning; require `D²` finite **and** within threshold; per-source accounting of last-received vs last-**fused**, invalid data vs innovation outlier, feeding NC1b. No universal rejection count. Zero variance is rejected as a sensor-contract violation, not because `S` is necessarily singular |
| C10 | `correction_c10_covariance_contract` | `P0`/`Q` positive semidefinite with scale-aware tolerance and inspected eigensolver status (intentional singularity allowed, warned); measurement `R` strictly positive definite; now toggled |

**Landing order** (from your dependency findings): NC3 → KC6 → KC5a → KC5b →
C8 → C7a → C7b → C9 → C10 → NC1a → NC1b → NC1c → NC2. One correction per
commit.

**Interaction tests** (not the cross product): C7a×C8, C9×C1, KC5a×KC6,
KC5b×NC1c, NC1b×NC2, NC1a×NC3, C10×C9, plus transition coverage for startup,
arm, disarm, rearm, restart while armed, degradation, recovery, double
degradation, and a ROS-time pause and backward jump.

## 3. Thresholds, all candidates

None is claimed as flight-safe. Each is REPORTED with its measured margin
before any promotion to a judged limit, and each is to be derived from
detection delay, actuation delay, speed and clearance rather than from
nominal latency.

| Parameter | Candidate | Basis |
|---|---|---|
| `command_rate_hz` | 50 | 5× margin on the 100 ms watchdog |
| `estimate_timeout_s` | 0.2 | to be re-derived; was your §2 objection |
| `setpoint_timeout_s` | 0.5 | ~0.15 m travel at the scenario's 0.3 m/s before braking |
| `horizontal_timeout_s`, `altitude_timeout_s` | 1.0 | observation-rate dependent (20 Hz range, 100 Hz velocity) |
| `bridge_max_s` | 0.3 | 3 watchdog periods |
| `recover_samples` / `recover_duration_s` | 5–10 / 0.05–0.1 | your §6 candidate |
| `max_propagation_dt` | 10 × `estimator_dt` | arbitrary; wants justification |

## 4. Acceptance claims we withdrew

Per your "before approving criteria, revise these claims": the
`velocity_loss` 0.5 m drift limit (no mechanism behind it — now a REPORTED
measurement with a commanded-velocity criterion instead); the frozen-thrust
≤ 1 m/s descent guarantee (withdrawn entirely); "50 skips with no covariance
growth" (inverted — growth is now required); C9's "state unchanged" (now: no
measurement-update contribution, judged against an otherwise identical
trajectory with the observation absent); KC6's finiteness criterion (now
command safety: no NaN, no unintended zero-thrust command, outputs in
limits); and the 256-combination figure (14 toggles, enumerated).

## 5. The three narrowings to challenge

1. **KC5b scope.** Reseeding at enable and recovery transitions only; general
   K3/K4 correction stays in phase 2. Our reason: enable-time seeding is
   required for a correct recovery and does not change steady-state numbers,
   while general K3/K4 correction does and belongs with re-tuning.
2. **`handoff` not the default.** Adopted as a parameter value only, because
   it rests on an unverified firmware assumption. Our reason: shipping an
   unvalidated failsafe as the default is worse than shipping a bounded
   simulation behaviour and gating the hardware default on P10.
3. **Range-only altitude fallback excluded from phase 1.** Our reason: your
   own prerequisite list makes it a separate controller with its own
   validation, and it may share the failure that killed the estimator.

## 6. Open questions for this pass

1. **Threshold basis.** Is stopping distance plus detection and actuation
   delay the right derivation, and what else belongs in it? What evidence
   would make `estimate_timeout_s` defensible rather than merely tested?
2. **`bounded_descent` without a hardware thrust map.** It bounds descent
   using the *stand-in's* characterized linear thrust map. Is a descent-rate
   bound meaningful under those conditions, or should the simulation default
   also be `hold` until P10 characterizes the real map?
3. **BRIDGE margin structure.** `bridge_max_s` 0.3 s against a 50 Hz
   scheduler and a 100 ms watchdog. Should BRIDGE be expressed in watchdog
   periods rather than seconds, and is 3 periods the right bound?
4. **NC1a × NC3 deadlock.** A command failing validation must not be
   published, but silence while armed is punished. We escalate and publish
   the supervisor's command instead. Is there a state where both the computed
   command and the policy command are invalid, and what should happen then?
5. **C8 invalidation.** Beyond `max_propagation_dt` we invalidate the state
   and require reinitialization. Could invalidation plus reinitialization be
   worse than a bounded propagation with inflated covariance, and how should
   that boundary be chosen?
6. **C7a/C7b split.** With C7a on and C7b off, a norm-1.1 quaternion is
   accepted and scales the rotation matrix — deliberately, because that is
   master's behaviour for valid input. Is that an acceptable shipped
   combination, or should validity imply normalization?

## 7. Context unchanged from V1

Simulation only; no hardware output; the low-level loop is a labelled
development-tool stand-in, not ROSflight. Attitude and the velocity
observations are idealized (simulation truth); there is no attitude
estimator. The estimate→command path has a 20 ms p99 limit needing an idle
host, and a correction is budgeted at ≤ 2 ms of p99. Known hardware gaps
(P10): the firmware command contract, `MIN_THROTTLE`, RC override, the real
failsafe; `is_flying` is unconnected as in the legacy launch files. Evidence
labels: **[V]** observed in this project, **[A]** assumption. The ground
rules in V1 §"Ground rules" still bind every proposal: reachable bit-exact
parity, immutable legacy goldens, no dependence on truth or test hooks,
ROS-free numerical cores, fatal invalid parameters.

## 8. How to respond

Most useful, in order: whether the revision is implementable as it stands;
answers to §6; a verdict on the §5 narrowings; then anything in §2 you would
restructure. If you still see a missed hazard class, that outranks
everything else. Please flag anything you could not verify from this packet
alone, and state your assumptions.
