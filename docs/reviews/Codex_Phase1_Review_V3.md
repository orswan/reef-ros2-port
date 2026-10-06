**I do not confirm all five contracts as written.** B1 is defensible for simulation, but D1 needs amendment: losing arming-status messages during established flight must not immediately cut thrust. Contact detection and timing accounting also need the clarifications below. These are contract amendments within the agreed architecture.

[V] I checked the packet, revision-3 register and criteria, and relevant stand-in source at `617b2a6`. No simulations or fault tests were run. Recommendations are engineering judgments. [A] They assume the stand-in’s attitude loop remains operational and hardware output remains blocked until P10.

**Contract A — implementable with an explicit simulation envelope and evidence requirement**

**A1. Evidence that `descent_command` improves the outcome**

Your proposed evidence characterizes behavior, but does not establish “better than nothing.” That requires a comparison with command suppression from equivalent initial conditions.

Before activating the simulation default, record:

- Initial height, vertical/horizontal velocity, tilt, and failure cause.
- Commands, actual attitude, actuator saturation, trajectory, termination time, and disarm acknowledgment.
- Peak descent speed, touchdown speed, horizontal displacement, and resulting damage/contact outcome, alongside the command-suppression case.
- Conditions under which descent worsens the outcome or never reaches the landing criterion.

Define a **finite test envelope and duration**, even though descent speed remains REPORTED. You need not establish a universal speed guarantee before implementing the mechanism. Activation as the simulation default should follow evidence within that envelope; describing it as safer requires comparative evidence.

“No commanded climb” is an intent statement. A descending-thrust command may initially accompany upward motion, and cannot guarantee monotonic descent.

**A2. Can bounded commands produce unacceptable force?**

Yes. Bounded command magnitude limits requested collective force only under the characterized map. It does not establish acceptable acceleration, impact speed, or attitude-control authority.

[V] The stand-in allocates collective thrust and attitude torques, then clips individual rotor forces in [standin.cpp](/root/ros2_ws/reef_ros2/src/reef_fc_standin/src/standin.cpp:48). Clipping can change the delivered collective force and torque. At low collective thrust, attitude correction can also produce nonzero rotor thrust.

Amend A to distinguish **requested collective force** from **delivered rotor forces**, and report saturation and actual attitude response throughout the descent.

**Contract B — B1 accepted for simulation; add the fallback table**

**B1. Suppress publication in UNCONTAINED?**

**Yes, when neither the normal candidate nor a capability-compatible fallback passes validation.** Suppress actuator-command publication, latch UNCONTAINED, and continue health reporting. Do not revive a cached command merely because it previously passed validation.

This lets the stand-in’s watchdog expire, but it remains an **uncontained failure**, not a safe landing or guaranteed motor shutdown. Cancel scheduled republication and test that queued/in-flight commands cannot restart an indefinite command stream.

Before P10, this remains a simulation policy. P10 must establish what silence actually does on hardware, including RC fallback, mode transitions, and firmware failsafe behavior; do not transfer this policy automatically.

Reconcile this with Contract C: if UNCONTAINED always suppresses commands, minimum thrust at timed termination must be a **single validated transition command**, followed by suppression—not indefinite minimum-thrust publication. If even that command is invalid, omit it.

**B2. Is capability compatibility sufficiently defined?**

**No: include an explicit table.** It should specify, for each capability-loss combination:

- Eligible bridge/terminal action and its prerequisites.
- Command fields and permitted thrust source.
- Expiry, simultaneous-loss precedence, and invalid-fallback response.

State selection must be deterministic. In particular, absent attitude-control authority cannot select a level-attitude descent command.

**Contract C — implementable after clarifying landing evidence, timeout, and acknowledgment**

**C1. Disarm request versus leaving disarm to firmware**

A separate **disarm request is the right interface**. Define its recipient, acknowledgment through fresh status, bounded retry behavior, and priority relative to any scenario runner’s arm request. A competing publisher must not undo terminal disarming.

Request publication is not completion. Remain terminal until fresh DISARMED status confirms the result; otherwise report failure to acknowledge.

Commanding zero collective thrust also does not prove motors are stopped. In this stand-in, attitude torque allocation can still produce rotor thrust. Confirm motor shutdown after the stand-in accepts disarm.

**C2. Choosing `descent_max_s` without altitude**

Treat it as a **maximum emergency-action duration**, not an inferred time to landing.

Use the permitted flight envelope to select the budget. A recent qualified last-known altitude can refine the estimate only with a stated age horizon and uncertainty growth. Do not require that information to enter emergency descent: it may be precisely what has been lost.

Unknown height and unbounded descent speed preclude guaranteeing touchdown before expiry. Expiry means “the supported emergency-action budget ended,” not “landing completed.” Record the ensuing disarm request and command suppression as UNCONTAINED.

Make the timer absolute; capability flicker must not reset it. Define a timeout for failure to obtain the landing criterion even while altitude remains nominally healthy.

**C3. Does range-based contact detection recreate the excluded fallback?**

It does **not** recreate a range-only altitude controller, but it introduces a safety-critical **landing detector**. Its prerequisites still matter.

Low range plus low height rate can indicate stationary flight just above the ground, a nearby object, or a stuck measurement. Healthy altitude estimation alone is insufficient.

Amend the contract to require:

- Valid, fresh range from the designated source, with known sensor offset and usable surface/tilt geometry.
- Distinct advancing observations over a minimum dwell time—not repeated fusion of one reading.
- A valid rate estimate and reset of the detector after invalid/gapped observations.
- Entry only during landing/terminal descent.
- Tests for low stationary hover, frozen range, and false close surfaces.

Call this a **range-based landing criterion**, rather than proof of physical contact. For the constrained simulation envelope, specify the maximum residual height/drop accepted when zero thrust is requested.

**Contract D — not implementable as written; split ARMING_UNKNOWN by history**

**D1. Withholding thrust after status loss during flight**

**No: immediate withholding is inappropriate when status becomes stale after confirmed active flight.** It converts a status-channel dropout into deliberate loss of lift.

Use these distinct cases:

| History | Behavior while status is unknown |
|---|---|
| Never confirmed armed, or last confirmed disarmed | Publish no flight commands; do not initiate autonomous thrust |
| Previously confirmed armed with active control | Block mission continuation; enter a bounded emergency policy while commands remain valid and required capabilities survive |
| Emergency budget expires or no valid fallback exists | UNCONTAINED; apply the explicit publication-suppression policy |

This bounded continuation preserves previously established emergency authority; it does not grant new normal-flight authority from a stale status.

A fresh DISARMED report must immediately terminate publication. A fresh ARMED report permits requalification, but must not automatically resume a latched terminal mission. Replace “requires fresh POSITIVE status to leave” with **fresh authoritative status**: a fresh negative status resolves uncertainty too.

**D2. Status freshness horizon**

The approximately 10 Hz rate supplies an expected period, not a trustworthy deadline.

Choose:

\[
T_{\text{status}}=kT_{\text{period}}+J_{\text{status}}+D_{\text{transport}},
\]

with an explicit allowed missed-message count and measured delay budget. [A] Three periods plus measured allowance is a reasonable initial simulation candidate, not an approved safety limit.

Measure source age and receipt liveness separately. Validate source identity/order/epoch; repeated old “armed” messages must not refresh trust. Keep the status-loss deadline separate from the bounded emergency-continuation deadline.

**Contract E — amend measurement definitions and clock-discontinuity rules**

**E1. Reserve \(M\)**

Use a stated **fixed reserve** for the simulation operating envelope, supported by measured worst observed gaps and stress tests. A percentile alone leaves the tail unprotected. [A] A 20 ms reserve is a possible initial experiment, not evidence of adequacy on this host.

Two accounting corrections are necessary:

1. Receiver inter-command gaps measure the **whole publication/transport path**, not `T_transport` alone. Do not add that complete gap to timer and processing terms.
2. A periodic check that republishes only once command age exceeds one period can permit nearly **two timer periods** between commands. Use that bound unless the implementation schedules against an exact publication deadline.

[V] Current `cmd_age` is `now − command.header.stamp` in [standin_node.cpp](/root/ros2_ws/reef_ros2/src/reef_fc_standin/src/standin_node.cpp:194). It does **not** measure receiver inter-arrival gaps. Add receipt-time instrumentation while preserving source-age reporting.

Score the observed receiver gap directly against `100 ms − M`; use separately measured components to explain the budget. Passing runs establish empirical margin, not a hard scheduling guarantee.

**E2. Forward jumps and unobservable pauses**

Both need explicit rules:

- **Known pause:** freeze simulation-action time and simulation-input ages only with explicit, fresh pause confirmation.
- **Clock stops without confirmed pause:** classify clock liveness as unknown/failed using monotonic time; do not suspend freshness indefinitely.
- **Large forward discontinuity:** expire overdue inputs and absolute action deadlines immediately; do not integrate the jump or emit catch-up commands. Requalify affected state/epochs.
- **Normal positive advancement:** process normally within the specified timing/outage bounds.
- **Resume:** reject pre-pause queued data that fails ordering/freshness checks.

A stopped clock alone cannot distinguish pause from failure. ROS time explicitly supports pauses and discontinuities, so the adapter must supply that distinction. [ROS clock design](https://design.ros2.org/articles/clock_and_time.html).

**Confirmation status:** B1 is accepted for the simulation stand-in. **Final confirmation to implement all contracts is withheld pending the specific D1 amendment and reconciliation of C’s termination publication, landing criterion, and E’s timing measurements.** No wider architectural review is needed for these amendments.