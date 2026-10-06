# Contract amendments for sign-off (phase 1, revision 4)

Your targeted confirmation ([pass 3](Codex_Phase1_Review_V3.md)) accepted B1
for simulation and withheld final confirmation pending a D1 amendment and
reconciliation of C's termination publication, the landing criterion, and E's
timing measurements. Those amendments are below, with nothing else changed:
the correction set, IDs, landing order, interaction tests and phase boundary
are as in revision 3. Full text in
[../CORRECTIONS.md](../CORRECTIONS.md) and
[../ACCEPTANCE.md](../ACCEPTANCE.md) (revision 4).

**Both of your code claims were verified here.** [V] The stand-in clamps each
rotor force individually after allocation, so clipping changes delivered
collective force and torques, and the attitude-error torques can produce
nonzero rotor thrust at low collective
([standin.cpp:61-71](../../src/reef_fc_standin/src/standin.cpp#L61-L71)).
[V] `cmd_age` is `now − command.header.stamp`
([standin_node.cpp:193](../../src/reef_fc_standin/src/standin_node.cpp#L193),
stamp at [:107](../../src/reef_fc_standin/src/standin_node.cpp#L107)) —
source age, not an inter-arrival gap.

---

## D — arming states, split by history (the blocking amendment)

Rev 3's single ARMING_UNKNOWN state is **withdrawn**.

| History when status becomes unknown | Behaviour |
|---|---|
| never confirmed armed, or last confirmed disarmed | publish no flight commands; do not initiate autonomous thrust |
| **previously confirmed armed with active control** | **block mission continuation; continue a bounded emergency policy** while commands remain valid and required capabilities survive. Preserves previously established emergency authority; grants no new normal-flight authority from a stale status |
| emergency budget expired, or no valid fallback | UNCONTAINED, with publication suppression |

Leaving uncertainty requires **fresh authoritative status**: a fresh DISARMED
immediately terminates publication; a fresh ARMED permits requalification but
never auto-resumes a latched terminal mission.

`T_status = k·T_period + J_status + D_transport`, with an explicit allowed
missed-message count `k` and measured delays. [A] Three periods plus a
measured allowance is the initial candidate, not a safety limit. Source age
and receipt liveness are measured separately; identity, order and epoch are
validated so repeated old "armed" messages cannot refresh trust. The
status-loss deadline stays separate from the emergency-continuation deadline.

## C — termination, landing criterion, disarm

- **Renamed a range-based landing criterion**, not proof of contact.
  Prerequisites: valid, fresh range from the designated source; known sensor
  offset; usable surface and tilt geometry; **distinct advancing observations
  over a minimum dwell**, never repeated fusion of one reading; a valid rate
  estimate; detector reset after invalid or gapped observations; entry only
  during landing or terminal descent. The envelope states the maximum
  residual height and drop accepted when zero thrust is requested. Tests:
  low stationary hover, frozen range, false close surface.
- **Termination publication reconciled with B1.** At timed termination the
  minimum-thrust command is a **single validated transition command**,
  followed by suppression — never an indefinite minimum-thrust stream — and
  is omitted if it fails validation.
- **Disarm is a request with a contract**: defined recipient, acknowledgment
  by fresh status, bounded retries, and **priority over a scenario runner's
  arm request**. Publication is not completion: the state stays terminal
  until a fresh DISARMED status confirms, otherwise failure to acknowledge is
  reported. Zero collective thrust is **not** evidence of stopped motors;
  shutdown is confirmed after the stand-in accepts the disarm.
- **`descent_max_s` is a maximum emergency-action budget** from the permitted
  flight envelope, not an inferred time to landing. A qualified last-known
  altitude may refine it only with a stated age horizon and uncertainty
  growth, and is never *required* to enter emergency descent. Expiry means
  the budget ended, not that landing completed. Timers are absolute;
  capability flicker cannot reset them. A **separate timeout** covers failure
  to obtain the criterion while `altitude` is nominally healthy.

## E — timing measurements and clock rules

- **Scored quantity:** the observed receiver inter-command gap against
  `T_watchdog − M`. The component terms (timer period, jitter, processing,
  transport) explain the budget and are **not** summed with the gap, which
  already covers the whole path — rev 3 double-counted.
- **Design bound:** two timer periods between commands, unless publication is
  scheduled against an exact deadline, which is now the preferred
  implementation.
- **`M` is a stated fixed reserve**, justified by worst observed gaps and
  stress tests, not a percentile. [A] 20 ms is an initial experiment.
- **Instrumentation prerequisite:** receipt-time measurement added to the
  stand-in, preserving the existing source-age field (task NC1a-I; requires
  `reef_check.sh control` by project rule).
- **Five clock rules:** confirmed pause freezes action time and input ages
  only with explicit fresh confirmation; a clock stop without confirmation
  classifies clock liveness as unknown or failed on monotonic time and does
  not suspend freshness indefinitely; a large forward discontinuity expires
  overdue inputs and absolute deadlines immediately, with no catch-up
  commands and requalification afterwards; a backward jump invalidates the
  epoch and requires readiness again; on resume, pre-pause queued data
  failing ordering or freshness is rejected.

## B — fallback table and UNCONTAINED

- **An explicit deterministic fallback table** replaces "capability-compatible
  fallback": per capability-loss combination, the eligible action, its
  prerequisites, the command fields and permitted thrust source, the expiry,
  precedence, and the invalid-fallback response. A disqualified prerequisite
  takes the next eligible row, and **absent attitude authority can never
  select a level-attitude action**.
- **UNCONTAINED** suppresses actuator-command publication, latches, continues
  health reporting, cancels scheduled republication, and never revives a
  cached command because it once validated; a test asserts that queued or
  in-flight commands cannot restart the stream. It is recorded as an
  **uncontained failure** — not a safe landing, not a guaranteed motor
  shutdown — and is a simulation policy that does not transfer to hardware
  until P10 establishes what silence does there.

## A — envelope and evidence

- **Requested collective force and delivered rotor forces are reported
  separately**, with rotor saturation and the actual attitude response
  throughout every descent, because command bounds do not bound delivered
  force.
- **Activation of the simulation default requires comparison**: each descent
  case inside a stated finite envelope and duration is recorded alongside the
  **command-suppression case from equivalent initial conditions**, including
  initial height, velocities, tilt and failure cause; commands, attitude,
  saturation, trajectory, termination time and disarm acknowledgment; peak
  descent speed, touchdown speed, horizontal displacement and contact
  outcome; and the conditions under which descent **worsens** the outcome or
  never reaches the criterion. The mechanism may be implemented first; the
  default and the word "safer" wait for the comparison.
- **"No commanded climb" is an intent statement**, not monotonic descent.

---

## What we are asking

Confirm that these amendments resolve D1 and the C and E reconciliations, or
name the residual change. We are not seeking a wider review, and if the
amendments stand we will begin implementation in the revision-3 landing
order, with every toggle defaulting off until the phase-1 capstone.

Two points we would flag as still weakest, in case you disagree with how we
have bounded them: [A] the bounded emergency continuation on stale arming
status has no verified firmware counterpart until P10, and [A] the
`descent_max_s` budget is chosen from an envelope we assert rather than one
we have measured.
