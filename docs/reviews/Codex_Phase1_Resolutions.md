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

---

# Pass 2 resolutions (revision 2 → revision 3)

Review: [Codex_Phase1_Review_V2.md](Codex_Phase1_Review_V2.md) (independent,
Codex, 2026-10-06, against HEAD `067f603`). Verdict: **"a sound architectural
direction, but I would not approve implementation exactly as written"**, with
the remaining work in the fallback contract, C8's noise model, and
contradictory acceptance statements. The reviewer again states it ran nothing
and changed no files.

As in pass 1, every code claim was re-checked here before acceptance. **Three
of this pass's findings are against claims we made**, and all three stand.

## 1. Claims of ours that pass 2 refuted

| Our claim (rev 2) | Status |
|---|---|
| XY process noise does not grow with the gap, because `xyEst.Q` is scaled at the nominal `dt²` | **Refuted.** [V] The contribution is `ΔP = G Q_xy Gᵀ Δt`, which already grows linearly with elapsed `dt`. Whether further scaling is correct depends on what `Q_param` means, and `xy_Q` has no documented units or stochastic assumptions ([estimator_master.yaml:32](../../src/reef_estimator/config/estimator_master.yaml#L32)). We inferred a defect from a double-integrator intuition the XY state may not satisfy. The rescaling is removed from C8 and registered as research item C11 (USER decision) |
| 50 Hz gives a 5× safety margin on the 100 ms watchdog | **Refuted.** A frequency ratio is not a margin. Replaced by the budget `T_timer + J_scheduling + T_processing + T_transport < T_watchdog − M`, every term measured, transport from receiver-side inter-command gaps |
| With C7a on and C7b off, a norm-1.1 quaternion is accepted | **Refuted.** C7a rejects any norm outside 1 ± 1e-3 regardless of C7b, and our own C7b fixture used norm 1.01, which C7a also rejects. The fixture is now norm 1.0005; 1.01 and 1.1 are rejection fixtures; the four-row behaviour table is specified, including C7a-off/C7b-on |
| `bounded_descent` bounds the descent rate | **Refuted.** A thrust map gives force, not speed: constant thrust below weight accelerates downward with no speed-limiting mechanism, and enforcing a bound would need the vertical state whose loss triggered the fallback. The terminal action is now a model-based **descent command** with bounded command values and the speed **REPORTED** (USER decision) |
| `controller_restart` gains corrected-mode criteria under NC1a | **Refuted.** A timer inside a dead process publishes nothing. NC1a covers estimator loss while the controller lives; process death is registered for P10 and that scenario keeps only its legacy characterization |
| "No spurious capability loss" across a ROS-time pause and backward jump | **Refuted as written.** It would have preserved stale readiness. Now: a pause accrues no liveness age and fabricates no failure, while a **backward jump invalidates the epoch and requires readiness again** |
| 50 skips propagate with covariance growth | **Refuted as self-contradictory** against a ten-period step limit. Withdrawn; growth within an admissible gap and invalidation beyond the outage boundary are now separate criteria, and the noise question is C11 |

## 2. Pass 2 claims verified here

| Claim | Our check |
|---|---|
| Estimator diagnostics fire about once per 250 IMU callbacks | [V] confirmed: [sensor_manager.cpp:221-224](../../src/reef_estimator/src/sensor_manager.cpp#L221-L224), ~1 Hz at 250 Hz IMU → NC4 |
| `update()` and `partialUpdate()` still use unchecked inverses | [V] confirmed: `K = P Hᵀ S⁻¹` at [estimator.cpp:33](../../src/reef_estimator/src/estimator.cpp#L33) and [:47](../../src/reef_estimator/src/estimator.cpp#L47). Our C9 covered only the gate → split into C9a and C9b |
| A timer cannot cover its own process restart | [V] confirmed by construction against the documented 3.7 s restart |
| Monotonic liveness conflicts with a deliberate pause | [V] confirmed: [pause_resume.yaml](../../src/reef_sim/config/closed_loop/pause_resume.yaml) pauses 2 s of wall time, which a 0.2 s receipt-age timeout would report as failure |
| Bridge content must be specified per cause | Accepted: validation establishes admissibility, not suitability; a last validated command may hold sustained tilt, a yaw turn or saturated thrust |
| Both candidate commands can be invalid | Accepted: bounded non-recursive path, startup validation of fallback parameters, UNCONTAINED recorded rather than called safe |
| Unknown arming status is a distinct state | Accepted: ARMING_UNKNOWN, with startup, stale-armed and confirmed-disarm behaviour and no late queued command regaining authority |
| One arbitration decision per published command | Accepted: a fresh estimate callback cannot overwrite a latched terminal command |
| Blind reinitialization can be worse than bounded propagation | Accepted: mark unusable / propagate where justified / requalify, mode-aware, never re-entering ground calibration while airborne |
| Rejection status must reach command assembly | Accepted: KC6 propagates it; a counter alone is insufficient |
| NC3 and KC5a cannot exercise a supervisor that does not exist | Accepted: landing order reordered (NC4 → NC1a/b/c → NC3 → KC6/KC5a/KC5b → C8/C7 → C9a/C9b/C10 → NC2) and staged activation adopted (USER): toggles land default false, a capstone commit flips them |
| The P10 gate must be enforced in the implementation | Accepted: the sink refuses hardware output until an approved contract parameter exists; scored by a negative test |
| Companion attitude-input loss differs from flight-controller attitude failure | Accepted: NC1b distinguishes them; the latter is UNCONTAINED |
| KC5b must cover every authority transition, per-axis and mode changes, and define integrator behaviour | Accepted |
| Terminal behaviour needs a terminal objective | Accepted: contact detection from range while `altitude` is healthy, otherwise a duration-bounded descent, then minimum thrust, a published **disarm request** and UNCONTAINED; never truth-driven |
| Threshold derivation needs envelope, uncertainty, BRIDGE motion and fallback capability | Accepted, with vertical clearance assessed separately from horizontal stopping distance |
| `hold` is not intrinsically safer | Accepted, and stronger (USER): `hold` means frozen thrust, retains the original hazard, and is **rejected** as a terminal policy — a configuration naming it is refused at startup |
| D5 may remain a documented gate approximation if the fusion is validated | Accepted: D5 stays registered for phase 2, C9b supplies the condition |
| The three narrowings are defensible | Noted; KC5b's widening to every authority transition is folded in |

Nothing in pass 2 was rejected.

## 3. What changed in the plan

Revision 2 had 13 phase-1 corrections and 14 toggles; revision 3 has **15
and 15**:

- **New:** NC4 (estimator health interface), C9b (fusion-solve validity).
- **Rewritten:** NC1a gains a measured timing budget and an explicit scope
  limit; NC1b gains pause/backward-jump semantics, the attitude-loss
  distinction and NC4 as its source; NC1c gains per-cause bridge content, an
  unrefreshable expiry, precedence, single arbitration, ARMING_UNKNOWN,
  UNCONTAINED, the invalid-command path and a terminal objective with disarm
  authority; KC5a gains the arming contract; KC5b widens to every authority
  transition; KC6 gains rejection propagation; C7b gains the behaviour table
  and corrected fixtures; C8 is cut to `dt` bounding plus a separate outage
  boundary; C9 splits.
- **Process:** staged activation (toggles default false, capstone flip), and
  a new rule that a quantitative claim without units, assumptions and a
  derivation is REPORTED and unproven — written because pass 2 caught two
  such claims.
- **Registered:** C11 (XY process-noise discretization), controller-process
  death, IMU-based contact detection.

## 4. Open for the targeted confirmation pass

See [Codex_Phase1_Confirmation_Packet.md](Codex_Phase1_Confirmation_Packet.md):
the fallback contracts only — terminal semantics and termination, the
invalid-command path and UNCONTAINED, arming-status states, and the timing
budget.

---

# Pass 3 resolutions (revision 3 → revision 4)

Review: [Codex_Phase1_Review_V3.md](Codex_Phase1_Review_V3.md) (independent,
Codex, 2026-10-06, against HEAD `617b2a6`), the **targeted confirmation** of
the five fallback contracts. Outcome: **B1 accepted for simulation; final
confirmation withheld** pending a D1 amendment and reconciliation of C's
termination publication and landing criterion with E's timing measurements.
Pass 3 states no wider architectural review is needed for these amendments.
The reviewer again ran nothing and changed no files; its [A] assumptions are
that the stand-in's attitude loop stays operational and hardware output stays
blocked until P10.

Both code claims were re-checked here. Both confirmed.

## 1. Contract outcomes

| Contract | Outcome | Amendment in rev 4 |
|---|---|---|
| **A** terminal action | implementable **with** an envelope and comparative evidence | activation of the simulation default now requires descent cases recorded alongside command-suppression cases from equivalent initial conditions, inside a stated finite envelope and duration; "no commanded climb" is downgraded to an intent statement; requested collective force and delivered rotor forces are reported separately |
| **B** invalid-command path | **B1 accepted** for the simulation stand-in; B2 needs a table | UNCONTAINED suppresses actuator publication, latches, continues health reporting, cancels scheduled republication, never revives a cached command, and is recorded as an *uncontained failure*; an explicit deterministic **fallback table** replaces "capability-compatible fallback" |
| **C** contact and disarm | implementable after clarification | renamed a **range-based landing criterion** with prerequisites and a dwell requirement; `descent_max_s` is an emergency-action budget, not time-to-landing; absolute timers; a separate timeout for failing to obtain the criterion; termination publishes a **single validated transition command** then suppresses; disarm has a recipient, acknowledgment by fresh status, bounded retries and priority over a scenario runner's arm request; zero collective is not evidence of stopped motors |
| **D** arming states | **not implementable as written** | split by history: never-armed or last-disarmed → no flight commands; **previously confirmed armed with active control → bounded emergency continuation**, not an immediate thrust cut; expiry → UNCONTAINED. "Fresh positive status" becomes **fresh authoritative status**; `T_status = k·T_period + J_status + D_transport` |
| **E** timing budget | amend measurements and clock rules | the observed receiver gap is scored directly against `T_watchdog − M` and is **not** summed with the component terms; the design bound is **two timer periods**; `M` is a stated fixed reserve, not a percentile; five explicit clock-discontinuity rules; receipt-time instrumentation added to the stand-in |

## 2. Pass 3 claims verified here

| Claim | Our check |
|---|---|
| Rotor-force clipping changes delivered collective force and torque; attitude correction can produce nonzero rotor thrust at low collective | [V] confirmed: `f = inv_ * (T, τx, τy, τz)` with each component clamped to `[0, f_max]` ([standin.cpp:64-71](../../src/reef_fc_standin/src/standin.cpp#L64-L71)); the torques follow attitude error ([:61-63](../../src/reef_fc_standin/src/standin.cpp#L61-L63)), so a low `T` still yields positive rotor forces |
| `cmd_age` does not measure receiver inter-arrival gaps | [V] confirmed: `age = now − cmd_stamp_` with `cmd_stamp_` taken from the command header ([standin_node.cpp:193](../../src/reef_fc_standin/src/standin_node.cpp#L193), [:107](../../src/reef_fc_standin/src/standin_node.cpp#L107)) — source age. Registered as instrumentation task NC1a-I, which requires `reef_check.sh control` |
| A periodic check republishing after one period of age permits nearly two periods between commands | Accepted as a design bound; the preferred implementation schedules against an exact publication deadline |
| Receiver gaps measure the whole path and must not be added to the component terms | Accepted; this was a double-count in rev 3 |
| Immediate thrust withholding on stale status in established flight is inappropriate | Accepted, and it is the finding that blocked confirmation. Rev 3's single ARMING_UNKNOWN state is withdrawn |
| A fresh negative status also resolves uncertainty | Accepted; "fresh authoritative status" replaces "fresh positive status" |
| Low range plus low rate can mean hover near the ground, a nearby object, or a stuck reading | Accepted; prerequisites, dwell, rate validity, detector reset and entry conditions added, with tests for all three confounds |
| Request publication is not completion | Accepted; terminal persists until fresh DISARMED, with failure-to-acknowledge reported |
| Indefinite minimum-thrust publication contradicts UNCONTAINED suppression | Accepted; a single validated transition command, then suppression |
| A stopped clock cannot distinguish pause from failure | Accepted; freezing requires explicit fresh pause confirmation, and five discontinuity rules are specified |
| Forward jumps need their own rule | Accepted; expire overdue inputs and absolute deadlines immediately, no catch-up commands, requalify |

Nothing in pass 3 was rejected.

## 3. What changed in the plan

Revision 4 is **amendments, not restructuring**: the correction set, IDs,
landing order, interaction tests and phase boundary are unchanged from
revision 3. Amended: NC1a (timing budget, two-period bound, instrumentation
prerequisite), NC1b (five clock rules), NC1c (fallback table, delivered-force
reporting, activation evidence, landing criterion, termination and disarm,
UNCONTAINED suppression) and KC5a (arming contract split by history, status
freshness formula). Two new tasks are registered: NC1a-I (receipt-time
instrumentation) and the descent-versus-suppression evidence task that gates
the simulation default.

## 4. Status

Final confirmation is **withheld** until the amendments are signed off. They
are listed for that purpose in
[Codex_Phase1_Amendments.md](Codex_Phase1_Amendments.md). No further
architectural review is expected.
