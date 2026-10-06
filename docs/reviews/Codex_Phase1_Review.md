**Verdict: revise the design before implementation.** The failure classes are credible, but the proposed fixes do not yet establish a safe degraded-flight policy. The largest gaps are command scheduling during estimator loss, confusing estimate freshness with estimator health, and assuming that finite or zero output is safe output.

[V] I inspected the packet, correction register, draft acceptance criteria, and relevant source at repository HEAD `3322e6d`. I did not run simulations or fault tests. Existing flight results remain project-reported evidence; recommendations below are engineering judgments, not demonstrated hardware behavior. No files were changed.

**Missed hazards that should shape phase 1**

- **The controller stops computing when estimates stop.** [V] Commands are generated from the estimate callback in [controller.cpp](/root/ros2_ws/reef_ros2/src/reef_control/src/controller.cpp:40). An age check inside that path cannot detect total publication loss. NC1 needs a periodic supervisor that runs without incoming estimates. Moreover, waiting 200 ms before taking over publication allows the stand-in’s 100 ms watchdog to expire. Command continuity must begin before that deadline.
- **Fresh publication does not establish usable state.** An estimator can publish at 250 Hz while its velocity observations are absent, its covariance grows, or its state is invalid. NC1 as specified cannot detect `velocity_loss`, so its proposed ≤0.5 m drift criterion lacks a corresponding mechanism.
- **Other control inputs need freshness and validity checks.** Position mode consumes a separate pose; arming status can remain latched; attitude availability is assumed. Include pose, attitude, status, timestamp ordering, and mode-dependent required inputs in the health contract.
- **Some commands bypass PID protection.** [V] The direct-attitude branch copies requested attitude values without the normal attitude clamps in [controller.cpp](/root/ros2_ws/reef_ros2/src/reef_control/src/controller.cpp:104). KC6 needs a final check of the complete outgoing command, including mode, ignore bits, limits, and the serialized float fields.
- **The acceleration guard misses infinity.** [V] The estimator checks `isnan(norm)`, rather than finiteness of each acceleration component. C7 should cover non-finite acceleration as well as quaternion validity, including initialization.
- **Continuing publication can suppress the downstream failsafe.** A stream of degraded commands keeps the offboard link apparently alive. That must be an intentional, bounded policy with a defined escalation—not an indefinite substitute for the flight controller’s failsafe.

**Answers to the six open questions**

**1. NC1 degraded command**

**Do not approve indefinite last-thrust hold as an altitude-hold or bounded-descent guarantee.** It is a possible short bridge while transferring control.

Last thrust may reflect a climb, descent, saturated PID, or tilt compensation. After leveling, its vertical effect changes. Battery, payload, disturbances, and the unverified hardware throttle map further undermine the assumption. Level attitude also does not stop existing horizontal motion.

My preferred architecture is:

1. Detect which control capabilities remain available.
2. Transfer to a verified flight-controller fallback using its own healthy sensors.
3. Use a brief, explicitly bounded bridge command only while that transfer completes.

A capability hierarchy could retain altitude control after horizontal sensing loss, use stabilized descent after altitude-estimation loss, and require firmware-owned emergency behavior after attitude loss. This resembles the capability-dependent fallbacks documented by [PX4](https://docs.px4.io/main/en/config/safety), but provides no evidence about ROSflight.

Of the packet’s three choices:

| Choice | Recommendation |
|---|---|
| Last valid thrust | Short bridge only; define expiry and validate the thrust selected |
| Parameterized descent | Preferred terminal behavior **when supported by verified attitude control and a characterized thrust/vertical-control contract** |
| Range-only altitude hold | A separate fallback controller requiring its own validation |

Range-only control needs independent sensor access, freshness, usable geometry, tilt compensation, surface assumptions, bounds, and transition tests. It may share the failure that killed the estimator. A fixed lower throttle cannot guarantee a descent rate.

[A] This recommendation assumes the flight controller retains healthy attitude stabilization when REEF fails. If that assumption is false, commanding level attitude is insufficient.

**2. NC1/NC2 thresholds**

**Keep 0.2 s and 0.5 s as experimental candidates; neither is justified by nominal p99 latency alone.**

A p99 of 10–20 ms does not bound scheduling stalls, consecutive losses, or worst-case stopping distance. Select deadlines using detection delay, supervisor jitter, actuator response, current speed, braking ability, and available clearance.

For horizontal motion, a useful first check is:

\[
d_{\text{stop}}\approx v(T_{\text{detect}}+T_{\text{actuation}})+\frac{v^2}{2a_{\text{brake}}}.
\]

At the scenario’s 0.3 m/s, a 0.5 s setpoint timeout already permits approximately 0.15 m travel before braking begins.

For NC1, separate **command heartbeat timing** from **estimate usability timing**. Continuous command publication must not wait for a 200 ms timeout. Nor should the timer repeatedly integrate against the same frozen estimate.

Use monotonic receipt time for hardware liveness, plus source timestamps for measurement age after validating the clock relationship. Reject future, duplicate, and out-of-order data according to an explicit policy. Simulation pause/reset semantics need separate tests: ROS time can pause and jump backward, as described in the [ROS clock design](https://design.ros2.org/articles/clock_and_time.html).

A non-real-time host cannot promise a hard watchdog deadline; firmware must handle host stalls.

**3. KC5 inhibition**

**Distinguish disarmed inhibition, armed-but-not-ready behavior, and in-flight degradation.**

For the current stand-in, silence while positively disarmed is reasonable. For hardware, neither silence nor an all-zero command is established as safe until mode, ignore-bit, throttle, and timeout semantics are verified.

- Before readiness, prevent entry into autonomous control or arming where the interface permits it.
- If already armed without usable inputs, select a verified fallback.
- Once flying, loss of readiness must enter degradation, not simply stop publication.

KC5’s “one fresh estimate and one fresh setpoint” is insufficient unless “fresh” also means valid, ordered, from the current startup/reset epoch, and appropriate for the requested mode. Arming status itself needs a freshness contract.

Clearing integrators does **not** fix first-step timing or derivative initialization. Seed the control timestamp and derivative history when enabling the corrected path; test arm/disarm/rearm and restart while armed.

**4. C9 on rejection**

**Reject an invalid observation immediately; escalate according to lost sensing capability and elapsed time since usable fusion.**

Do not use a universal rejection count. Five rejects mean different outage durations at 20 Hz and 250 Hz, and silence produces no rejects at all.

Track separately:

- last valid received observation;
- last observation actually fused;
- invalid-data rejections versus innovation outliers;
- source availability and relevant state uncertainty.

An isolated invalid sample should not force whole-vehicle degradation if sufficient independent sensing remains healthy. Persistent loss of all usable horizontal observations should remove horizontal velocity/position control authority even while estimator publications remain fresh.

C9 also needs numerical checks beyond incoming covariance: validate the innovation covariance \(S\), factorization success, and finite, non-negative \(D^2\). Prefer a checked solve over explicit inversion.

[V] The XY gates currently use the **previous** `xyEst.R`; the incoming covariance is installed after acceptance in [xyz_estimator.cpp](/root/ros2_ws/reef_ros2/src/reef_estimator/src/xyz_estimator.cpp:228). Incoming-covariance validation alone leaves that coupling unchanged. Correcting it needs an explicit registered decision.

The claim that zero measurement variance necessarily produces infinite gain is too broad: \(S=HPH^T+R\) can remain invertible with \(R=0\). Rejecting zero variance is defensible as a sensor contract, but use the correct rationale.

**5. C10 strictness**

**Use PSD for state/process covariances and require usable innovation covariance at runtime.**

- `P0` and `Q`: permit positive semidefinite matrices, including intentional singularity.
- Measurement `R`: strict positive definiteness is a sensible operational sensor contract here, unless exact constraints are explicitly supported.
- Runtime `S`: require positive definiteness and acceptable numerical conditioning before a solve.

Use documented, scale-aware tolerances and check eigensolver success. Warn on singular `P0`/`Q` when it matters operationally; do not characterize every singular covariance as erroneous.

There is also a governance conflict: C10 is declared non-toggleable despite the supplied rule requiring a toggle for every correction. D10’s historical exception is a rationale, not explicit approval for another exception. Resolve this before coding. Either approve C10 as startup contract enforcement or provide a toggle used by legacy tests.

**6. Recovery hysteresis**

**Yes: require sustained health, distinct advancing samples, and a controlled transition.**

One fresh sample may be a delayed packet, a transient recovery, or a newly restarted estimator with an incorrect initial state.

Use:

- immediate entry on invalid required state; timeout entry for missing data;
- an exit age threshold below the entry threshold;
- both a minimum number of distinct samples and a minimum healthy duration;
- capability-specific health checks and reset detection;
- timestamp/derivative reseeding and a bounded transition back to normal commands.

[A] Five to ten samples over at least 50–100 ms is a reasonable initial test candidate, not an approved flight value. Test alternating good/bad samples, burst recovery, delayed queues, restart, and recovery during simultaneous setpoint loss.

A terminal landing fallback should not automatically resume the interrupted mission merely because observations return.

**Failure-analysis corrections, ordering, and layering**

[V] The NaN-gate analysis is correct: `NaN > threshold` is false, so the accept branch is taken **when execution reaches that comparison**. Some inputs encounter earlier checks; test each source separately.

[V] A NaN state can permanently poison the internal PID differentiator. However, “every non-finite error permanently latches” is too broad: a NaN setpoint with finite measured state does not necessarily poison that differentiator, and supplied derivatives have different behavior.

KC6 must validate **before either PID overload mutates state**, calculate candidate state updates, and commit them only if finite. Explicit comparison clamps still pass NaN through unless there is a preceding finite check. Finite inputs can also overflow. Returning zero from a helper is not a safe actuator policy—zero vertical output may remove thrust.

**Land C8 before C7, with separate commits.** But revise C8’s justification: after a skipped sample, the longer dt is genuinely elapsed time. Reducing it to one interval silently omits motion and process uncertainty. Track the accepted sample timeline separately from the propagated-state time; define how gaps increase uncertainty, invalidate state, or trigger reinitialization. Do not blindly move the timestamp backward or accept a future stamp as the new baseline.

**Land KC6 before KC5**, and include corrected startup/resumption timing in KC5. Freezing integrators during degradation also requires explicit recovery handling.

The proposed layering is sound: node/adapters own clocks, transport validation, and periodic scheduling; a ROS-free supervisor/core owns capability decisions and transitions. Pass ages, validity flags, and reset epochs—not a ROS clock. Keep callbacks that run on other threads limited to flags.

Use a separate controller-health topic. [V] `status` is already the controller’s firmware arming input; publishing degradation there risks an authority collision.

**Toggle coverage and acceptance**

Keep per-correction toggles. A `legacy_mode` profile may configure them conveniently, but should not add another ambiguous precedence layer.

Individual on/off tests plus all-off/all-on profiles are **insufficient for the identified interactions**. Add targeted combinations: C7×C8, C9×C1, KC5×KC6, NC1×NC2, and simultaneous sensing/setpoint loss. Test startup, degradation, recovery, disarm, and restart transitions. The advertised 256 combinations are also inaccurate: normalization adds a toggle, C10 lacks one, and C1 already exists.

Before approving criteria, revise these claims:

- NC1 cannot meet `velocity_loss` drift limits using publication age alone.
- Frozen thrust cannot guarantee ≤1 m/s descent.
- Fifty skipped samples without covariance growth can indicate unjustified confidence.
- C9’s “state unchanged” should mean no measurement-update contribution, compared with an otherwise identical rejection trajectory.
- KC6 must score safe command behavior, not merely finiteness.

K1/K2 may remain in phase 2 for simulation development, but they remain flight-safety issues. Phase 1 must include the timing and state-transition handling needed by its own fallbacks; deferring those dependencies would leave the new safety behavior incomplete.