**Revision 2 is a sound architectural direction, but I would not approve implementation exactly as written.** Most first-review findings are addressed. The remaining changes are concentrated in the fallback contract, C8’s noise model, and several contradictory acceptance statements.

[V] I read the V2 packet, resolutions, correction register, revised acceptance criteria, and relevant source at HEAD `067f603`. I ran no simulations or fault tests and changed no files. The conclusions below are design judgments; hardware behavior remains unverified.

**The principal remaining issues**

1. **A thrust map does not establish a descent-rate bound.** It maps command to force; speed additionally depends on initial velocity, attitude, mass, disturbances, drag, and elapsed time.
2. **C8’s claimed XY noise defect is not yet established by the cited equations.** XY already adds a process-noise contribution proportional to elapsed dt. The nominal-dt² scaling deserves investigation, but does not by itself prove that replacing it with actual-dt² is correct.
3. **C7’s specification contradicts Question 6 and its normalization fixture.** C7a explicitly rejects norm 1.1; turning C7b off must not change that. Norm 1.01 also fails the proposed C7a tolerance.
4. **NC1a cannot cover its own process restart.** A timer inside a stopped controller cannot maintain commands through the documented 3.7 s restart.
5. **Command continuity must have an explicit exception when no valid fallback exists.** “Never silent while armed” cannot override command validation or firmware authority.

**1. Threshold basis**

Stopping distance plus detection and actuation delay is the right starting point. Add uncertainty, accumulated motion during BRIDGE, and the capability of the fallback itself.

For each failure class, establish:

- permitted initial speed, tilt, altitude, and proximity to obstacles;
- sensor age, health-report transport delay, supervisor scheduling delay, and actuator response;
- estimation error and missing-input uncertainty;
- available braking or vertical recovery authority;
- the complete trajectory through detection, BRIDGE, and TERMINAL.

Horizontal stopping distance alone cannot justify an estimate timeout during altitude-control loss. Assess vertical clearance separately. Likewise, leveling after horizontal sensing loss removes commanded acceleration but does not brake existing velocity.

A defensible `estimate_timeout_s` needs an operating envelope and evidence that the total delay remains inside that envelope: fault injection at different speeds and flight phases, adverse scheduling/transport conditions, uncertainty analysis, and measured margins. Nominal p99 latency remains insufficient.

**The 50 Hz scheduler has a frequency ratio, not a demonstrated safety margin.** Require a budget such as:

\[
T_{\text{timer}}+J_{\text{scheduling}}+T_{\text{processing}}+
T_{\text{transport}} < T_{\text{watchdog}}-M.
\]

Here \(M\) is an explicit reserve. Measure inter-command gaps at the receiver, not merely publication times.

Also reconcile monotonic liveness with simulation pause behavior. A wall-clock timeout will expire during a deliberate simulation pause unless the adapter explicitly distinguishes paused simulation from failed hardware inputs. A backward ROS-time jump should invalidate the old epoch and require readiness again; “no spurious capability loss” must not mean preserving stale readiness. The [ROS clock design](https://design.ros2.org/articles/clock_and_time.html) explicitly permits pauses and backward jumps.

**2. Simulation `bounded_descent` versus hardware policy**

**Keep a simulation descent policy if desired; do not replace it with `hold` simply because hardware is uncharacterized. But revise what “bounded” means.**

The stand-in’s characterized map makes thrust selection meaningful **within that simulation model**. It does not alone establish a descent-speed guarantee. Approximately,

\[
\dot v_z=g-\frac{T\cos\phi\cos\theta}{m}+\text{disturbances}.
\]

Constant thrust below weight produces continuing downward acceleration without a speed-limiting mechanism. Thrust equal to weight preserves an existing downward velocity in the idealized case. A speed bound requires feedback, a justified dynamic model over a finite horizon, or a demonstrated terminal-speed mechanism.

Acceptable simulation alternatives are:

- **Model-based descent command:** bounded command values, with resulting descent speed reported.
- **Finite-horizon descent envelope:** a speed bound justified over specified initial conditions, disturbances, and duration.
- **Closed-loop descent:** only when a usable vertical-state source remains available.

When `estimate` or `altitude` is lost, do not silently use that same invalid state to enforce `descent_rate_max`. Simulation truth may score the behavior, but must not drive the fallback.

`hold` is not intrinsically safer. If it means frozen thrust, it retains the original hazard indefinitely. If it means closed-loop altitude hold, it requires healthy altitude capability. Define the parameter’s semantics precisely and prohibit unsupported combinations.

For hardware, retain the P10 gate **in the implementation**, not just documentation: the simulation terminal policy must not become a hardware default through an omitted configuration override. Hardware output should remain unavailable until an approved command and failsafe contract exists.

The present “vehicle intact” criterion also needs a terminal objective. Descending onto the ground while indefinitely publishing nonzero thrust is not a completed landing. Define contact/landing behavior and disarm authority without relying on truth.

**3. BRIDGE duration and watchdog structure**

**Express BRIDGE in physical seconds. Report its ratio to watchdog periods, but do not derive it from that ratio.**

These deadlines serve different purposes:

- The watchdog detects loss of commands.
- BRIDGE limits exposure to a temporary fallback.

With continuous publication, the watchdog does not expire during BRIDGE. Therefore, “three watchdog periods” supplies no physical justification for 0.3 s.

Choose BRIDGE duration from the maximum acceptable trajectory under the bridge command, including velocity, tilt, altitude, thrust uncertainty, and transfer latency.

The revised bridge holds the **entire last validated command**. That command could contain sustained tilt, a yaw turn, or saturated thrust. Validation establishes admissibility, not suitability for a failure. Specify bridge content per capability loss; do not assume the last normal command is the safest bridge.

BRIDGE must have an absolute expiry that cannot be refreshed by repeated bad packets, changing failure reasons, or command republication. Simultaneous capability losses should escalate immediately according to explicit precedence rather than traversing every state sequentially.

**4. NC1a × NC3: both commands invalid**

**Yes, both can be invalid.** Examples include a corrupt or absent last command, invalid fallback parameters, unavailable attitude authority, a serialization overflow, or a policy branch that requires a capability already lost.

Use a bounded, nonrecursive path:

1. Validate the normal candidate.
2. On failure, latch the reason and select a capability-compatible fallback.
3. Validate the fallback through the same final gate.
4. If it fails, invoke the defined terminal failure response and report the condition.

Do not repeatedly bounce between validator and supervisor. Validate fallback parameters at startup, and test failure of the fallback itself.

**Never publish an invalid command to satisfy continuity.** When no valid command exists, explicit transfer/release of authority—or stopping publication under a verified downstream failsafe contract—may be necessary. Until P10 establishes that contract, this outcome must be recorded as uncontained rather than described as safe.

The stale-arming-status case also needs its own state. Unknown arming status is neither positively disarmed nor permission to generate flight thrust. Define behavior for startup with unknown status, stale previously armed status, and confirmed disarm. A late queued command must not regain authority after disarm.

Two related implementation requirements remain:

- All publication paths must use one supervisor arbitration decision. A fresh estimate callback must not overwrite a latched terminal command.
- NC1a covers estimator loss while the controller remains alive. Covering controller death requires a surviving component or firmware fallback. Revise the `controller_restart` acceptance claim accordingly.

**5. C8 invalidation and reinitialization**

**Yes: blind reinitialization can be worse than bounded propagation.** Resetting airborne altitude, velocity, bias, covariance, or takeoff state to startup defaults can create a plausible-looking but incorrect estimate and an immediate control transient.

Separate three actions:

- mark the state unusable for control;
- maintain or propagate an internal estimate where justified;
- reacquire and requalify the state before restoring authority.

A small integration-step limit need not equal the maximum permissible outage. Within an admissible gap, substeps can address numerical integration error, while additional uncertainty addresses missing input. Substeps do not reconstruct the missing acceleration or attitude.

Choose the outage boundary from maneuver bounds, input uncertainty, observability, numerical stability, and available recovery information. Reinitialization must be mode-aware and must not silently re-enter ground calibration while airborne.

**The proposed XY noise scaling still needs a mathematical decision.** [V] The current implementation uses

\[
Q_{\text{XY}}=Q_{\text{param}}h^2,\qquad
\Delta P_{\text{noise}}=GQ_{\text{XY}}G^\top\Delta t,
\]

where \(h\) is nominal dt. Its noise contribution already grows linearly with the gap.

Whether additional scaling is correct depends on what `Q_param` represents:

- continuous white-noise intensity commonly gives interval-proportional growth;
- uncertainty in an acceleration held across a gap can give velocity variance proportional to interval squared;
- other correlated uncertainties require their own model.

Specify units, stochastic assumptions, and the discretization before changing the equation. The XY state is not simply a position/velocity double integrator. Similarly, z’s live-dt scaling does not prove that its missing-input model is adequate.

The criteria also conflict: with a limit of ten nominal periods, fifty consecutive skips should cause invalidation. Require covariance growth **within** the admissible gap and invalidation **beyond** it. Do not require fifty-skip propagation and hard invalidation simultaneously.

**6. C7a/C7b split**

**The split is acceptable, but the combination described in Question 6 is incorrect.**

With C7a on and tolerance \(10^{-3}\):

| C7a | C7b | Required behavior |
|---|---|---|
| On | On | Reject invalid norm; normalize accepted input |
| On | Off | Reject invalid norm; use accepted near-unit input unchanged |
| Off | Off | Legacy path |
| Off | On | Explicitly defined normalization-only behavior, including zero/non-finite input |

A norm-1.1 quaternion must be rejected in the first two rows. It is not valid under the new contract merely because legacy code accepted it.

Use a norm such as 1.0005 for the normalization fixture with both corrections enabled. Keep norm 1.01 or 1.1 as rejection fixtures. Preserve the ability to test each correction independently.

All-on remains the appropriate shipped profile. C7a-on/C7b-off is useful compatibility evidence, but retains small rotation errors and should not be presented as equivalent to the normalized profile.

**Assessment of the three narrowings**

| Narrowing | Assessment |
|---|---|
| KC5b limited to enable/recovery | **Defensible**, provided it covers every control-authority transition, including per-axis recovery and mode changes. Define integrator freeze/reset behavior as well as derivative and timestamp seeding. Reseeding changes transition trajectories even if the steady-state control law remains unchanged. |
| `handoff` not the default | **Defensible.** An unverified handoff should not be the default. This also requires an enforced hardware configuration gate and a defined response to failed handoff. |
| Range-only altitude fallback deferred | **Defensible.** Retaining healthy existing altitude control is different from introducing an independent range-only fallback. The latter warrants separate design and validation. |

**Additional restructuring before approval**

- **Provide an operational health interface.** [V] Current estimator diagnostics are emitted roughly once per 250 IMU callbacks. Do not make safety depend on that existing diagnostic cadence. Specify timestamped, expiring health information carrying actual fused-observation age, estimator epoch, and state validity. Repeated fusion of one old observation must not refresh health.
- **Validate the actual measurement update.** [V] `Estimator::update()` and `partialUpdate()` still use unchecked inverses. A checked gate solve does not protect a later fusion using changed covariance/state. Validate the actual fusion solve and candidate state before committing it; D5 may remain a documented gate approximation.
- **Distinguish companion attitude-input loss from flight-controller attitude failure.** Level commands can remain meaningful in the former if firmware stabilization survives; they are unsupported in the latter.
- **Propagate rejection explicitly from KC6.** A numerical zero accompanied only by a counter can still become an unintended zero-thrust command. The command assembly must receive rejection status during that step.
- **Reorder dependent activation.** NC3 and KC5a cannot exercise their specified supervisor fallback before NC1c exists. Separate building the mechanism from activating the complete correction; do not expose partially assembled defaults.

With these contracts reconciled, I would support implementation and simulation validation. The revised layering and targeted interaction tests are appropriate; the remaining work is to make the failure responses precise enough to implement and score without assuming unavailable control authority.