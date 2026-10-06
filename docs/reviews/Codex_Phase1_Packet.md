# Codex review packet: P09.5 phase 1 (safety)

**What this asks for.** An independent safety and architecture review of four
proposed changes to a ROS 2 flight-control stack, **before** the code is
written. We want the failure analysis checked, the proposed designs
challenged, and the six open questions in §5 answered. We are not asking for
a code review: the code does not exist yet.

**Status of the thing being changed.** `sim-baseline-v0.1.0` (`main`
`7590a97`) is a bit-exact ROS 2 port of a ROS 1 research stack, verified in
simulation only. It reproduces the originals including their defects, by
decision. No hardware output exists. The goal that motivates this phase is
physical flight, so safety behaviour the originals never had is now in scope.

**Ground rules that constrain any proposal.**
1. Bit-exact parity with the originals must stay reachable: every correction
   has a named boolean parameter, default on, whose off state reproduces the
   legacy path exactly (253/253 estimator streams, 106/106 controller items).
2. Legacy goldens and reference tests are immutable historical evidence.
3. No safety behaviour may depend on simulation truth, on a test hook, or on
   anything unavailable on hardware.
4. Numerical cores stay free of ROS types; ROS lives in node and adapter
   layers.
5. Invalid parameters are fatal at startup, never silently replaced.

---

## 1. Architecture in one page

```
Gazebo (sim)                       REEF estimator                 REEF controller            low-level
  IMU 250 Hz  ──► imu_noise ──►  xyz_estimator (EKF)  ──►  controller + PID    ──►  stand-in loop ──► motors
  range 20 Hz ──► range_sensor ─►   z + xy filters      xyz_estimate 250 Hz      250 Hz      (NOT ROSflight)
  truth 100 Hz ─► idealized velocity observations ─►          ▲
  truth ───────► idealized attitude ─────────────────────────┘   desired_state 50 Hz (scenario runner)
```

- **Estimator** (`src/reef_estimator`): a loosely coupled EKF. `z_estimator`
  holds (z, ż, bias) driven by the IMU z-acceleration and corrected by range;
  `xy_estimator` holds body-level velocities and biases, corrected by
  velocity observations. Both run in `xyz_estimator::sensorUpdate` on the
  executor thread, single-threaded, no reorder buffer. Observations pass a
  Mahalanobis gate per source.
- **Controller** (`src/reef_control`): cascaded PIDs. Velocity mode is the
  one used in simulation; position mode uses an idealized mocap pose. Output
  is an attitude + thrust command at the estimate rate.
- **Stand-in** (`src/reef_fc_standin`): a *labelled development tool* that
  closes the loop in simulation in place of ROSflight firmware. It arms,
  applies a linear thrust map, and drops the vehicle if commands stop for
  more than 100 ms. It is **not** evidence about hardware.
- **Inputs labelled idealized**: attitude and the velocity observations come
  from simulation truth; range is modelled; an IMU vibration overlay is a
  scenario assumption, not a sensor model. There is no attitude estimator.

What the simulation has demonstrated: nominal flight (26/26 criteria,
estimate age p99 10–20 ms), bit-exact parity with the originals, and 11 fault
scenarios whose outcomes are documented — including crashes where the legacy
system has no protection.

## 2. The four phase-1 failure classes

Each is observed in simulation [V] unless marked [A].

### A. No freshness checks anywhere in the chain

The controller consumes `xyz_estimate` and `desired_state` with no notion of
age. `control_node.cpp` records the last estimate stamp only to stamp the
outgoing command.

| Observed | Result |
|---|---|
| estimator stops publishing (`dropout_long`) | the controller integrates against a frozen state; **the vehicle crashes** |
| velocity observations lost (`velocity_loss`) | the controller faithfully follows REEF's drifting estimate: **2.7 m in 10 s** |
| setpoint source stops (`setpoint_stale`) | a 6 s stale setpoint **keeps the vehicle moving** at the last commanded velocity |
| commands stop for > 100 ms | the stand-in **drops the vehicle**; a controller restart takes 3.7 s |

The last row matters for the design: *not publishing* is not a safe state.
Whatever a degraded mode does, it must keep publishing a defined command.

### B. Non-finite values latch permanently

`SimplePID::computePID` returns 0 on a non-finite error but stores the
non-finite state first:

```cpp
if (dt == 0.0 || !std::isfinite(error)) {
  last_error_ = error;     // stores NaN
  last_state_ = x;         // stores NaN
  return 0.0;
}
```

With `kd > 0` the differentiator then stays NaN for the rest of the flight,
the output becomes NaN, and `std::min`/`std::max` pass NaN through, so
saturation does not contain it. One bad sample is unrecoverable.

The estimator has the matching gap: only the acceleration is NaN-checked
(`xyz_estimator.cpp:131`); a NaN attitude quaternion propagates into the
rotation matrix. And because the NaN guard returns *before* advancing
`last_time_stamp`, every skipped sample makes the next integration step
cover twice the true interval.

### C. The observation gate accepts non-finite data

Every gate has the form

```cpp
Mahalanobis_D_hat_square(0) = (z - h).transpose() * S.inverse() * (z - h);
if (Mahalanobis_D_hat_square(0) > threshold) { /* reject */ }
```

A NaN `D²` makes the comparison false, so the observation is **accepted**.
Neither the measurement nor its covariance is checked for finiteness or for a
non-positive variance. The P05 review flagged this; **no test covers it**, so
the baseline's behaviour here is unverified rather than merely wrong.

### D. Covariance parameters are not validated for definiteness

`covarianceError` checks square, finite, exactly symmetric, non-negative
diagonal. An indefinite matrix passes all four and reaches the filter.

## 3. Proposed fixes (the thing to challenge)

| ID | Fix | Default |
|---|---|---|
| NC1 | node computes estimate age, core decides; age > 0.2 s enters a degraded state: level attitude, zero yaw rate, altitude channel holding the last valid thrust instead of integrating; recovery needs a fresh estimate; transitions logged once and published on `status` | on |
| NC2 | setpoint age > 0.5 s substitutes zero horizontal velocity, zero yaw rate, last valid altitude — stop and hold | on |
| KC5 | publish nothing until armed **and** one fresh estimate **and** one fresh setpoint; integrators stay cleared while inhibited | on |
| KC6 | never store a non-finite input, never integrate it, return 0, count it; explicit comparison clamps so NaN cannot reach the output; automatic recovery on the next finite sample | on |
| C7 | reject a non-finite quaternion or one with norm outside 1 ± 1e-3, same semantics as the existing NaN-acceleration skip; normalize valid quaternions (separate toggle, since it changes valid-input numbers) | on |
| C8 | advance `last_time_stamp` on every rejected sample; reject dt outside (0, 10 × `estimator_dt`] and report it | on |
| C9 | validate measurement and covariance (finite, symmetric, positive definite) before the gate; require `D²` finite **and** within threshold to accept; count rejections per source | on |
| C10 | add a minimum-eigenvalue check to covariance parameters; fatal at startup, naming the parameter and the eigenvalue | not toggleable |

Full entries, with failure scenarios, expected results, tests and parity
impact, are in [../CORRECTIONS.md](../CORRECTIONS.md). Proposed acceptance
criteria are in [../ACCEPTANCE.md](../ACCEPTANCE.md), section
"`corrections` (P09.5 phase 1)".

**Deliberately excluded from phase 1:** everything that changes control
tuning — the D-term sign error (K1), the inactive anti-windup (K2), heading
wrapping (K9), derivative kick and first-dt behaviour (K3, K4), missing
resets (K11). The simulation gains were tuned *with* those defects present,
so correcting them changes the closed-loop response and needs re-tuning. That
is phase 2.

## 4. What we would like reviewed

1. **Is the failure analysis right?** Particularly B and C: we claim one NaN
   is unrecoverable and that a NaN `D²` is accepted. Check the reasoning.
2. **Are there phase-1-class hazards we have missed** — a path where invalid
   or stale data reaches the motors that is not in §2? We are more worried
   about an omission than about a wrong threshold.
3. **Is the degraded-state design sound**, given that silence drops the
   vehicle and that there is no attitude estimator (attitude is idealized in
   simulation, and on hardware would come from the flight controller)?
4. **Is the layering right** — ages computed in the node, policy in the core?
   Alternatives: all in the node (core stays purely numerical), or the core
   takes timestamps and owns the clock (harder to test, violates our
   ROS-free-core rule).
5. **Ordering risk.** C7 makes skips more frequent, which makes C8 (double
   dt) matter more. Is it safe to ship C7 without C8, or must they land
   together? Similar coupling between KC5 and KC6 via the first armed step.
6. **Toggle architecture.** Eight independent booleans give 256 combinations.
   We propose testing each correction in both states plus two profiles
   (all-off = legacy parity, all-on = shipped default), not the cross
   product. Is that sufficient, and should the toggles instead be one
   `legacy_mode` switch with per-correction overrides?

## 5. Specific open questions

1. **NC1 degraded command.** Hold last valid thrust (our proposal), a
   parameterized descent, or level-and-hold-altitude on range alone? This
   decides what happens on hardware when the estimator dies in flight.
2. **NC1/NC2 thresholds.** 0.2 s (≈ 50 estimate periods at 250 Hz) and
   0.5 s. Too tight given that observed estimate age p99 is 10–20 ms and the
   host is not real-time? Too loose for a 1 m hover?
3. **KC5 inhibition.** Publish nothing, or publish an explicit zero/safe
   command? Not publishing is safe with the stand-in while disarmed, but the
   hardware command contract is unverified until P10.
4. **C9 on rejection.** Reject the observation and continue (our proposal),
   or treat repeated invalid observations as a sensor failure and enter the
   NC1 degraded state? If the latter, after how many?
5. **C10 strictness.** Require positive semidefinite and warn on singular
   (our proposal), or require strict positive definiteness? A singular `P0`
   is legitimate for a perfectly known initial state.
6. **Recovery hysteresis.** Should leaving a degraded state require N
   consecutive fresh samples rather than one, to avoid oscillating at the
   threshold?

## 6. Context a reviewer should know

- **Known hardware gaps (not phase 1):** the ROSflight `Command.u[3]`
  semantics in mode 2 are unverified [A]; `MIN_THROTTLE`, RC override and the
  real failsafe are unverified; the controller's `is_flying` input is
  unconnected, as in the legacy launch files, so integrators run whenever
  armed (approved for the baseline); a MAVLink unused-channel value reads as
  "RC switch on".
- **Timing:** the estimate→command path has ~2 ms resolution in sim time and
  a 20 ms p99 limit that needs an idle host. A correction that adds work to
  that path is budgeted at ≤ 2 ms of p99.
- **Correction already shipped:** C1 clears the XY flag after a partial
  update, fixing a defect that re-fused the last velocity observation at
  every IMU step and caused a permanent gate lock-out after a dropout. It is
  on by default and off for parity runs — the pattern every correction here
  follows.
- **Evidence labels used in our docs:** **[V]** observed in this project,
  **[A]** assumption.

## 7. How to respond

Most useful to us, in order: missed hazards (§4.2), a verdict on the
degraded-state design (§4.3 and §5.1), then the ordering and layering
questions, then the thresholds. If you disagree with the phase boundary —
for example if you think K1/K2 are a safety issue rather than a tuning issue
— say so explicitly, because that would move work from phase 2 into phase 1.

Please state assumptions you had to make, and flag anything in this packet
you could not verify from it alone.
