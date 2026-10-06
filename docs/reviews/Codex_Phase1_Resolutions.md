# Resolutions: Codex phase-1 design review

Review: [Codex_Phase1_Review.md](Codex_Phase1_Review.md) (independent, Codex,
2026-10-06, against repository HEAD `3322e6d`). Verdict: **revise the design
before implementation**. Reviewed artefact: revision 1 of
[../CORRECTIONS.md](../CORRECTIONS.md), the draft criteria, and
[Codex_Phase1_Packet.md](Codex_Phase1_Packet.md).

The reviewer states it ran no simulations or fault tests and changed no
files; its recommendations are engineering judgements, not demonstrated
hardware behaviour. Its own labels are preserved below.

**Every code claim in the review was re-checked here against the same tree
before being accepted.** Results are marked [V] where this project verified
them independently, [A] for assumptions.

## 1. Outcome summary

| Finding | Resolution |
|---|---|
| Controller stops computing when estimates stop; an age check in that path cannot detect loss | **accepted** → NC1a (periodic supervisor) |
| 200 ms threshold is slower than the 100 ms watchdog it protects against | **accepted** → NC1a at `command_rate_hz` 50 Hz |
| Fresh publication does not establish usable state; NC1 cannot see `velocity_loss` | **accepted** → NC1b (health and capability), and the drift criterion withdrawn |
| Other inputs need freshness and validity (pose, attitude, status, ordering, mode-dependent requirements) | **accepted** → NC1b health contract |
| Some commands bypass PID protection (attitude branch) | **accepted** → NC3 (outgoing-command validation) |
| The acceleration guard misses infinity | **accepted** → C7a, scope widened; initialization path too |
| Continuing publication can suppress the downstream failsafe | **accepted** → bounded BRIDGE with expiry and escalation (NC1c) |
| Q1 last-thrust hold must not be an altitude or descent guarantee | **accepted** → BRIDGE content only, bounded; `bounded_descent` is the terminal action |
| Q1 capability hierarchy and handoff to a verified fallback | **accepted in part** → capability hierarchy adopted; `handoff` is a parameter value gated on P10 |
| Q2 thresholds unjustified by nominal latency | **accepted** → all thresholds are candidates, REPORTED with margins, derived from stopping distance |
| Q2 separate command heartbeat from estimate usability | **accepted** → NC1a vs NC1b are now different corrections |
| Q2 monotonic receipt time plus validated source stamps; pause/jump tests | **accepted** → NC1b |
| Q3 distinguish disarmed / armed-not-ready / in-flight | **accepted** → KC5a's three-situation table |
| Q3 clearing integrators does not fix timing; seed timestamp and derivative | **accepted** → KC5b (USER pulled it into phase 1 with its own toggle) |
| Q4 no universal rejection count; track received vs fused, invalid vs outlier | **accepted** → C9 accounting feeds NC1b |
| Q4 validate `S`, factorization, finite `D²`; prefer a checked solve | **accepted** → C9 |
| Q4 the "zero variance ⇒ infinite gain" rationale is too broad | **accepted** → rationale changed to a sensor-contract violation |
| Q5 PSD for `P0`/`Q`, strict PD for `R`, conditioning for runtime `S` | **accepted** → C10 and C9 |
| Q5 governance conflict: C10 declared non-toggleable | **accepted** → USER: add the toggle; no exceptions |
| Q6 recovery hysteresis, distinct samples, bounded transition, no auto-resume | **accepted** → NC1c recovery rules |
| "Every non-finite error latches" is too broad | **accepted** → KC6 scope corrected |
| KC6 must validate before either overload mutates state; clamps pass NaN; zero is not a safe actuator policy | **accepted** → KC6 restated |
| Land C8 before C7; C8's justification is wrong | **accepted** → C8 premise rewritten, order fixed |
| Land KC6 before KC5 | **accepted** → landing order §4 |
| Layering is sound; pass ages and flags, not a clock | **accepted** → rule 7 |
| Use a separate controller-health topic; `status` is the arming input | **accepted** → `controller_health` |
| Per-toggle tests are insufficient; add named interactions | **accepted** → §5 interaction set |
| The 256-combination figure is inaccurate | **accepted** → 14 toggles, enumerated |
| Five acceptance claims to revise | **accepted** → all five withdrawn or rewritten |
| D5 (XY gates use the previous `R`) needs an explicit decision | **accepted** → registered for phase 2 |
| K1/K2 remain flight-safety issues though they may stay in phase 2 | **accepted with a scope decision** (USER): phase 2, with the timing dependencies pulled forward as KC5b |

Nothing was rejected. Two findings were narrowed, and one was extended by our
own verification; those three are below.

## 2. Independent verification of the review's code claims

| Claim | Our check |
|---|---|
| Commands come only from the estimate callback | [V] confirmed: `currentStateCallback` is the sole caller of `computeCommand()`, [controller.cpp:40-49](../../src/reef_control/src/controller.cpp#L40-L49) |
| Attitude branch bypasses the clamps | [V] confirmed: [controller.cpp:117-122](../../src/reef_control/src/controller.cpp#L117-L122) copies `attitude.x/y/yaw` raw; the velocity branch above clamps. This is K8, already characterized, but the review's framing as a validation gap is the useful one |
| Acceleration guard misses infinity | [V] confirmed and **extended**: `getVectorMagnitude` is `sqrt(x*x+y*y+z*z)` ([xyz_estimator.cpp:535-538](../../src/reef_estimator/src/xyz_estimator.cpp#L535-L538)), so an infinite component yields `inf` and `isnan(inf)` is false. We also found the **initialization** path averages samples with no validity check at all ([xyz_estimator.cpp:116](../../src/reef_estimator/src/xyz_estimator.cpp#L116)); C7a now covers it |
| `status` is the controller's arming input | [V] confirmed: [controller.cpp:58-62](../../src/reef_control/src/controller.cpp#L58-L62). Revision 1's "published on `status`" was wrong |
| "Every non-finite error latches" is too broad | [V] confirmed: the 3-argument overload updates `differentiator_` from `x - last_state_` ([simple_pid.cpp:45](../../src/reef_control/src/simple_pid.cpp#L45)) — the state, not the error. A non-finite setpoint with a finite state leaves the differentiator clean |
| NaN gate analysis correct "when execution reaches that comparison" | [V] confirmed at all four gates ([xyz_estimator.cpp:346](../../src/reef_estimator/src/xyz_estimator.cpp#L346), `:373`, `:402`, `:426`); the per-source caveat is right and C9's tests are per source |
| XY gates use the previous `R` | [V] confirmed: the incoming covariance is installed after acceptance, [xyz_estimator.cpp:228](../../src/reef_estimator/src/xyz_estimator.cpp#L228). This is D5; registered, not fixed in phase 1 |
| `S = HPH' + R` can be invertible with `R = 0` | [V] correct as mathematics; our "infinite confidence" wording was wrong and is withdrawn |
| C8's justification is wrong: the longer dt is genuinely elapsed time | [V] confirmed and **this is the most consequential finding**. The NaN branch returns before `last_time_stamp` is updated ([xyz_estimator.cpp:131-141](../../src/reef_estimator/src/xyz_estimator.cpp#L131-L141)), so the next accepted sample's `dt` is the true elapsed time. Revision 1 would have deleted real time from the filter |
| Gaps must increase uncertainty | [V] confirmed, with the mechanism located: `xyEst.Q` is scaled once at initialization by the nominal `dt²` ([xyz_estimator.cpp:71](../../src/reef_estimator/src/xyz_estimator.cpp#L71)) while the propagation adds `G*Q*G'*dt` linearly ([xy_estimator.cpp:123](../../src/reef_estimator/src/xy_estimator.cpp#L123)), so XY is under-noised across a gap. For z, `updateLinearModel()` rebuilds `F` and `Q = Q0*dt` from the live `dt` every step ([xyz_estimator.cpp:167](../../src/reef_estimator/src/xyz_estimator.cpp#L167)), so z is consistent. The defect is asymmetric between the two filters |
| 100 ms watchdog | [V] confirmed: `offboard_timeout_ms: 100` ([x3_standin.yaml:24](../../src/reef_fc_standin/config/x3_standin.yaml#L24)) |

## 3. Findings narrowed, with reasons

1. **"Phase 1 must include the timing and state-transition handling needed by
   its own fallbacks."** Accepted, but scoped: KC5b reseeds the control
   timestamp and derivative history **at enable and recovery transitions
   only**. The general K3/K4 behaviour stays in phase 2. Reason: enable-time
   seeding is required for a correct recovery and does not change
   steady-state numbers, whereas general K3/K4 correction does and belongs
   with re-tuning. USER decision, 2026-10-06.
2. **Capability-dependent fallback with handoff to the flight controller.**
   Adopted as the architecture, but `handoff` ships as a parameter value that
   is not the default, because [A] it assumes the flight controller retains
   healthy attitude stabilization when REEF fails — unverified until P10
   (STATUS item 22). Simulation default is `bounded_descent`. USER decision,
   2026-10-06.
3. **Range-only altitude fallback.** Not adopted in phase 1. The review's
   own list of prerequisites (independent sensor access, freshness, geometry,
   tilt compensation, surface assumptions, bounds, transition tests) is a
   separate controller with its own validation, and it may share the failure
   that killed the estimator. Registered as a candidate for a later phase.

## 4. What changed in the plan

Revision 1 had 8 corrections and 10 toggles. Revision 2 has 13 phase-1
corrections and 14 toggles (counting C1, already shipped):

- **New:** NC1a (command continuity), NC1b (input health and capability),
  NC1c (degraded policy, escalation, recovery), NC3 (outgoing-command
  validation), KC5b (enable-time reseeding), C7b split from C7a, C10 gains a
  toggle.
- **Rewritten:** NC1 split three ways; NC2 widened from freshness to health;
  KC5 split and restated as three situations; KC6 restated as
  validate-before-mutate with command safety as the criterion; C8's premise
  replaced; C9 extended to `S`, factorization and per-source accounting.
- **Withdrawn claims:** the `velocity_loss` 0.5 m drift limit; the
  frozen-thrust ≤ 1 m/s descent guarantee; "50 skips with no covariance
  growth"; "state unchanged" as C9's criterion; "finiteness" as KC6's
  criterion; the 256-combination figure; "fused with infinite confidence".
- **Newly registered for later:** D5.

## 5. Open items for the second review pass

1. The capability thresholds are candidates with no measured justification
   yet; they are REPORTED until measured. Are the proposed derivations
   (stopping distance, detection plus actuation delay) the right basis?
2. `bounded_descent` depends on the characterized thrust map of a
   *simulation* stand-in. Is a descent-rate bound meaningful without the
   hardware map, or should the simulation default also be `hold` until P10?
3. BRIDGE at 0.3 s with a 50 Hz scheduler and a 100 ms watchdog: is the
   margin structure right, or should BRIDGE be expressed in watchdog periods?
4. NC1a × NC3: a command that fails validation must not be published, but
   silence while armed is punished. The current answer is to escalate and
   publish the supervisor's command instead. Is there a failure mode where
   both the computed and the policy command are invalid?
5. C8's invalidation path marks state invalid beyond `max_propagation_dt`.
   Does invalidation plus reinitialization risk a worse outcome than a
   bounded propagation with inflated covariance?
6. Does splitting C7a and C7b leave a state where validity is checked but
   normalization is off, in which a norm-1.1 quaternion is accepted and
   scales the rotation matrix? (Our answer: yes, deliberately, because that
   is master's behaviour for valid input — but it should be challenged.)
