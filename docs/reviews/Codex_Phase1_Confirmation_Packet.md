# Codex targeted confirmation: P09.5 phase-1 fallback contracts

**Scope: narrow by design.** You reviewed this design twice
([pass 1](Codex_Phase1_Review.md), [pass 2](Codex_Phase1_Review_V2.md)) and
said you would support implementation "with these contracts reconciled". This
pass asks **only** whether the reconciled contracts are now precise enough to
implement and score. It is **not** a request for another architectural
review: the layering, the correction set, the interaction tests and the phase
boundary are settled unless you believe one of them breaks a contract below.

Everything else is in the register
([../CORRECTIONS.md](../CORRECTIONS.md), revision 3) and the criteria
([../ACCEPTANCE.md](../ACCEPTANCE.md), "`corrections` (P09.5 phase 1)",
revision 3). Still no code.

**Your pass-2 findings were all accepted; three were against claims of ours,
and all three stand.** We refuted ourselves on the XY process-noise defect
(the contribution already grows linearly with elapsed `dt`, and `xy_Q` has no
documented units — removed from C8, registered as research item C11), on
50 Hz as a "margin" (a frequency ratio), and on `bounded_descent` bounding a
descent rate. Full table in
[Codex_Phase1_Resolutions.md](Codex_Phase1_Resolutions.md) §1.

**Decisions taken since pass 2** (USER, 2026-10-06, fixed):

| Decision | Effect |
|---|---|
| Terminal action is a model-based **descent command**; speed **REPORTED, not bounded** | no descent-rate guarantee is claimed or scored |
| `hold` = frozen thrust, **rejected** as a terminal policy | a configuration naming it is refused at startup |
| Contact and disarm authority defined **without truth** | see contract C below |
| C8 cut to `dt` bounding plus a separate outage boundary | the legacy noise mathematics is not guessed at |
| **Staged activation**: toggles land default false, a capstone commit flips the phase-1 defaults | partially assembled defaults never ship |

---

## Contract A — terminal action and its envelope

```
descent_command:
  attitude      = level (roll 0, pitch 0), yaw rate 0
  thrust        = a value from the stand-in's characterized map chosen to produce descent,
                  bounded to [thrust_min_cmd, thrust_descent_cmd] (command values, not force)
  duration      = bounded by descent_max_s when no vertical state exists (contract C)
  resulting descent speed = REPORTED, never judged
  truth         = may score this behaviour, may never drive it
handoff:  unavailable until P10 supplies a verified command and failsafe contract;
          a failed handoff falls to UNCONTAINED
hold:     defined as frozen thrust and REJECTED; naming it is a startup error
```

**Questions.**
A1. With the speed unbounded and only command values bounded, what is the
minimum reportable evidence that `descent_command` is *better than nothing*
in simulation? We propose: the descent is monotonic in intent (no commanded
climb), attitude stays level within a stated tolerance, and the trajectory to
ground contact is recorded with its initial conditions. Is that the right
bar, or does a finite-horizon envelope need to be established before this
ships even as a simulation default?
A2. Bounding the thrust *command* interval is our stand-in for bounding
force. On the stand-in's linear map that is a defensible proxy; is there a
failure mode where a bounded command interval still produces an unacceptable
force, given the map is characterized but the vehicle state is not?

## Contract B — invalid-command path and UNCONTAINED

```
1. validate the normal candidate at the single gate (NC3)
2. on failure: latch the reason, select a capability-compatible fallback
3. validate the fallback at the SAME gate
4. on failure: terminal failure response, state = UNCONTAINED
   - recorded and reported as uncontained; never described as safe
   - no hardware output path exists (sink refuses until P10)
   - no invalid command is ever published to satisfy continuity
   - fallback parameters are validated at STARTUP, and fallback failure is tested
   - no recursion, no bouncing between validator and supervisor
```

**Questions.**
B1. In UNCONTAINED with no valid command available, the choices are: stop
publishing (the stand-in then drops the vehicle after 100 ms), or publish the
last *validated* command knowing it may be unsuitable. We propose **stop
publishing and record UNCONTAINED**, because publishing an unsuitable command
suppresses the only downstream failsafe we have. Do you agree, and does that
change before P10?
B2. Is "capability-compatible fallback" well enough defined by the NC1b
capability set, or does step 2 need an explicit per-capability fallback table
to be implementable without judgement at the keyboard?

## Contract C — contact, termination and disarm authority

```
altitude capability healthy:
  contact when range-derived height < contact_height_m
        AND |height rate| < contact_rate_mps
        for contact_samples consecutive FUSED observations
  then: thrust = 0, publish disarm REQUEST
no vertical state:
  descent_command bounded by descent_max_s
  then: thrust = thrust_min_cmd, publish disarm REQUEST, state = UNCONTAINED
always:
  the disarm request is PUBLISHED; honouring it belongs to firmware (P10)
  or, in simulation, to the labelled stand-in
  the controller never asserts disarm authority it does not have
  truth is never consulted
IMU-based contact detection: registered as a later candidate, not relied on here
```

**Questions.**
C1. Is a published disarm *request* the right contract, given the controller
has no arming authority (the stand-in arms, and `status` is an input)? The
alternative is that terminal behaviour simply ends at minimum thrust and
leaves disarming entirely to the firmware.
C2. With no vertical state, `descent_max_s` is the only bound on the descent.
How should it be chosen when the altitude at entry is unknown — from the
operating envelope's maximum permitted altitude, or should entry into that
branch require a last-known altitude within a stated validity horizon?
C3. Does range-derived contact detection reintroduce the dependency you
warned about for a range-only fallback? Our reading: no, because it is used
only while `altitude` capability is already healthy and only to *stop*, never
to control. Confirm or correct.

## Contract D — arming states

```
ARMED, fresh status         -> normal authority rules
DISARMED, fresh status      -> publish nothing; block entry into autonomous control
ARMING_UNKNOWN              -> entered when arming status is stale or never received
                               no thrust-generating command is published
                               reported; requires a fresh POSITIVE status to leave
after confirmed disarm      -> a late queued command must not regain authority
```

**Questions.**
D1. Is "no thrust-generating command" the right ARMING_UNKNOWN behaviour
while **already flying** with a stale status? It is the one case where
withholding thrust is itself hazardous, and where a stale "armed" may be the
truth.
D2. What freshness horizon makes an arming status trustworthy, given it
arrives at about 10 Hz in simulation?

## Contract E — timing budget

```
T_timer + J_scheduling + T_processing + T_transport  <  T_watchdog − M

T_watchdog = 100 ms (offboard_timeout_ms, stand-in)
T_timer    = 1 / command_rate_hz          (candidate 50 Hz; chosen FROM this inequality)
J, T_proc  = measured from the run
T_transport= measured as inter-command gaps AT THE RECEIVER (the stand-in records cmd_age)
M          = an explicit stated reserve
```

Scope limit, accepted from pass 2: this covers **estimator loss while the
controller lives**. Process death (`controller_restart`, 3.7 s) is out of
scope and registered for P10; that scenario keeps only its legacy
characterization.

**Questions.**
E1. What value of `M` would you consider defensible on a non-real-time host,
expressed how — a fixed reserve, a multiple of measured jitter, or a
percentile of the measured gap distribution?
E2. Pause semantics: while the simulation clock is paused and the pause is
observable, we accrue no liveness age; a **backward** jump invalidates the
epoch and requires readiness again. Is there a third case — a *forward* jump,
or a pause that is not observable to the adapter — that needs its own rule?

---

## How to respond

For each contract: **implementable as written**, or the specific change
needed. If every contract passes, say so plainly — that is the signal we need
to start coding. If one does not, we would rather hear it as a contract
amendment than as a general concern.

Please keep [A]/[V] labelling, state assumptions, and flag anything you
cannot verify from this packet plus the register and criteria. If you think
this pass should have been wider, say that too, but please still answer the
contracts.
