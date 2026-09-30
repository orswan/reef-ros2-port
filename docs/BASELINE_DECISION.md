# Estimator baseline decision (P02)

Status: **decided. R1 (2026-09-30): C1 approved and on by default; C2–C6 deferred.**
Date 2026-09-30. Evidence: `scripts/reef_check.sh baseline` (section 10). The
reference harness is described in [baseline/README.md](../baseline/README.md).

## 1. Decision

| | |
|---|---|
| **Selected baseline** | `reef_estimator` **master `e4179f48c3f26e22bd1366b71ee1e117ce2f5f7f`** (2021-03-08), with `reef_msgs` `7fb63ff93269040316b71d346dbc32919da1f63d` |
| Retained comparison reference | simulation branch `95987b5118b624208910d9e51424300022e1f512` (2019-11-22), runnable in the same harness |
| What "baseline" means | The ROS 2 port must reproduce this revision's behaviour for identical ordered inputs and initialization, within the tolerances in section 8. |
| Approved algorithmic deviations | none in P02. **C1 approved by the USER after R1 (2026-09-30)**, on by default in the port (`correction_c1_clear_xy_flag`); false reproduces master exactly (section 7) |
| Characterized legacy defects | D1–D10 (section 6). Each is kept in the reference, and each has a regression that asserts it |
| Candidate corrections | C1–C6 (section 7). **USER decision 2026-09-30: all deferred to the independent review R1**, so that the initial ROS 2 port reproduces master exactly. If R1 approves any, each is added afterwards as a separately documented, separately tested deviation. None is blended in silently |

## 2. What the two revisions actually are

`git merge-base` of the two commits is `ff29edc` (2019-08-02, "adding fail
safe"). From there:

| Side | Commits | What changed relative to `ff29edc` |
|---|---|---|
| simulation branch | `95987b5` "removed measurement rejection for gazebo sim" (Prashant Ganesh, 2019-11-22), a single commit | **only** comments out the four χ² gates (`src/xyz_estimator.cpp` lines 234, 252, 266, 284 at `95987b51`) |
| master | `5acb228` (dependencies), `39a4c6b` (`wren_camera.yaml`), `f23b932` (LICENSE), **`e4179f4` "changes for dt error"** (Humberto Ramos, 2021-03-08) | `e4179f4` made every other behavioural difference: the per-step dt model rebuild, the F bias-coupling sign, gravity 9.81, the XY flag handling, and the Z noise retune |

So the "simulation revision" is **the 2019 ancestor with rejection disabled
for Gazebo Classic**, not a separately developed algorithm. Master is that
ancestor plus the lab's later deliberate fix. The 2023 hardware bundle
(`reef_estimator_bundle` `a20b4e1d`) pins master; the 2022 simulation bundle
pins `95987b51`.

## 3. Reasons for selecting master

1. **It is the lab's latest deliberate revision** and the one pinned by the
   hardware bundle, so it is the most likely to reflect flown behaviour.
2. **Its time-step handling is correct at any IMU rate.** `e4179f4` rebuilds
   F, B, and Q from the measured IMU interval. The simulation revision fixes
   the Z model at dt = 2 ms. At the 250 Hz IMU of this project's X3 simulation,
   that doubles the estimated vertical velocity (fixture `s09`: velocity ratio
   2.013 vs 1.000).
3. **Its measurement rejection is intact.** Without the gates, a 3.5 m
   outlier corrupted the simulation revision's vertical velocity (−0.46 m/s),
   and a REP 117 `-inf` range (which this project's range sensor emits on the
   ground) made its state NaN **permanently** (`s10`).
4. Master's own defects (section 6) are narrow, reproducible, and
   characterized. D1 in particular is almost certainly an accidental side
   effect of `e4179f4` and is a correction candidate (C1).

What master is **not**: an unquestionable specification. Fidelity to it
answers "did we port it correctly"; whether its behaviour is right is a
separate question (sections 6 and 7).

## 4. Specification of the baseline (master `e4179f48`)

Source references are file:line at `e4179f48` unless stated.

### 4.1 Frames and conventions
- **World:** NED. **Body:** FRD. **Body-level:** NED rotated by yaw only; this
  is the frame of the XY velocity states and the mocap/RGB-D velocity inputs.
- **IMU:** `linear_acceleration` is **specific force** in FRD (at rest,
  level: (0, 0, −g)). `orientation` is the body attitude quaternion.
- `reef_msgs::quaternion_to_rotation(q)` returns I − 2w[v×] + 2[v×]²
  (`reef_msgs/src/dynamics.cpp:67`). That is the transpose of the standard
  body-to-world matrix, i.e. **C_NED→body**.
- **Sonar:** positive range; the Z measurement is z = −range (NED,
  `xyz_estimator.cpp:262`). No tilt compensation (the TODO at line 167).
- **Mocap pose:** NED z, used directly (line 294).

### 4.2 Vertical filter (`ZEstimator`, linear KF)
- **State** x = [z, ż, b]ᵀ (NED; m, m/s, m/s²). b is the additive offset of
  the NED-z specific force, so true acceleration = u − b.
- **Input:** u = (C_NED→bodyᵀ f)_z + g_init (line 177).
- **Model**, rebuilt on every propagation from the measured dt
  (`z_estimator.cpp:94-107`, called at `xyz_estimator.cpp:184`):
  F = [[1, dt, 0], [0, 1, −dt], [0, 0, 1]], B = [0, dt, 0]ᵀ,
  G = [[0, 0], [1, 0], [0, 1]], Q = diag(z_Q) · dt.
- **Propagate:** x ← Fx + Bu; P ← FPFᵀ + GQGᵀ (`estimator.cpp:29-33`).
- H = [1, 0, 0]; R = R0 on the ground, R_flying after takeoff.

### 4.3 Horizontal filter (`XYEstimator`, EKF)
- **State** x = [vx, vy, θ_b, φ_b, b_ax, b_ay]ᵀ: body-level velocity (m/s),
  **pitch bias then roll bias** (rad), and accelerometer biases (m/s²). The code
  uses `xHat(2)` as the pitch bias (`xy_estimator.cpp:56-57`); the comment in
  `params/xy_est_params.yaml` lists roll first, which is wrong.
- **Propagation** (`xy_estimator.cpp:52-126`):
  - roll and pitch come from C (`roll = atan2(C12, C22)`, `pitch = −asin(C02)`);
  - θ_e = pitch − θ_b and φ_e = roll − φ_b;
  - C_bl→body is built from θ_e and φ_e;
  - input s = f + C·[0, 0, g_init] + [b_ax, b_ay, (C·[0, 0, b_z])_z], where
    b_z is the Z filter's bias **passed as float32**. The bias enters with a
    **plus** sign (compare D2);
  - v ← v + (C_bl→bodyᵀ s)_{0:2} · dt.
  - The covariance is integrated in continuous-time Euler form,
    P ← P + (FPᵀ + PFᵀ + GQGᵀ) dt, with the Jacobian F and noise mapping G of
    lines 112-122.
- Q = xy_Q · estimator_dt², **scaled once** at startup (`xyz_estimator.cpp:76`),
  not per step.
- H = [I₂, 0]; R starts at xy_R0; each accepted mocap/RGB-D message
  overwrites R(0,0) and R(1,1) with its covariance entries [0] and [7].

### 4.4 Initialization
- **Accelerometer init:** the first 20 valid IMU samples are averaged and not
  propagated (lines 122-142, 156-160). g_init is then **overridden to 9.81**
  (line 137).
- x0, P0, P0_flying, Q, R0, R_flying, and β are loaded from parameters. A
  vector of length n is read as a diagonal matrix, one of length n² as a full
  matrix (`reef_msgs/matrix_operation.h:41-64`). Missing parameters give
  **zeros** with a warning; wrong lengths leave the matrix **uninitialized**
  with an error.
- **Partial-update weights:** α = 1 − β and Γ = diag(α) (`estimator.cpp:17-27`).
- **Takeoff** (`checkTakeoffState`, lines 441-498): requires the
  accelerometer-magnitude sample variance over 20 samples ≥ 0.5 **and** the
  current Z measurement z(0) ≤ −0.25 (range ≥ 0.25 m). On takeoff, R ← R_flying
  and P ← P0_flying. **Landing** (both conditions false): R ← R0, P ← P0,
  x ← x0.
- **On the ground,** every 10th propagation resets the XY state and P to x0/P0,
  and resets Z's P to P0 and ż to x0 (lines 200-212).

### 4.5 Order of operations per IMU message (`sensorUpdate(Imu)`)
1. Skip the message if ‖f‖ is NaN (attitude is not checked).
2. Accelerometer init (section 4.4) until 20 samples have been seen.
3. dt = stamp − last_stamp; last_stamp ← stamp (ROS `toSec()` difference).
4. Propagate XY with the **pre-propagation** Z bias, then Z.
5. Landing reset if on the ground and the counter reaches 10.
6. If the XY flag is set: partial update when `enable_partial_update` (the
   default), otherwise a full update and clear the flag.
7. If the Z flag is set: partial or full update; always clear the flag.
8. Publish; then the takeoff/landing check, which may overwrite R, P, and x.

Measurements only store z and R and set a flag. Fusion happens at the next
IMU message, whatever the measurement's own stamp.

### 4.6 Time step and noise semantics
- dt is the difference between consecutive **processed** IMU stamps. A NaN
  sample does not update last_stamp, so the next dt spans both intervals
  (`s11`).
- `estimator_dt` (default 0.002) only scales xy_Q (Q_xy = xy_Q · estimator_dt²)
  and sets the initial dt. Z uses Q = z_Q · dt(measured) per step, so the units
  of xy_Q and z_Q differ. This is preserved.

### 4.7 Updates
- **Full:** K = PHᵀ(HPHᵀ + R)⁻¹; x ← x + K(z − Hx); P ← (I − KH)P (not the Joseph
  form; measured asymmetry ≤ 6e−16, section 8).
- **Partial** (Brink, "Partial-Update Schmidt–Kalman Filter",
  doi:10.2514/1.G002808): compute the full update x⁺, P⁺; then
  x ← α⊙x⁻ + β⊙x⁺ and P ← Γ(P⁻ − P⁺)Γ + P⁺ (`estimator.cpp:49-74`).
- **Shipped β:** XY [1, 1, 0.03, 0.03, 0.001, 0.001] (velocities fully
  updated, biases partially); Z [1, 1, 0.5].

### 4.8 Rejection rules

| Input | Delivered when | Accepted when |
|---|---|---|
| Sonar | `enable_sonar` | not `useMocapZ`, `range <= max_range` (float32), **and** D² = (−range_f32 − z)² / (P00 + R) ≤ `mahalanobis_d_sonar` (shipped 20) |
| Mocap Z | `enable_mocap_z` | `useMocapZ` and D² with float32(z) ≤ `mahalanobis_d_mocap_z` (20) |
| Mocap XY | `enable_mocap_xy` | `useMocapXY` and D² ≤ `mahalanobis_d_mocap_velocity` (50), with S from the **current (previous) R** |
| RGB-D | `enable_rgbd` | parameter `enable_measurements` (read each call; shipped **false**), not `useMocapXY`, and D² ≤ `mahalanobis_d_rgbd_velocity` (80) |
| RC | `enable_mocap_switch` | channel `mocap_override_channel` > 1500 sets `useMocapXY` / `useMocapZ` (if enabled); ≤ 1500 reverts (if RGB-D / sonar enabled) |

`useMocapXY = enable_mocap_xy && !enable_rgbd` and
`useMocapZ = enable_mocap_z && !enable_sonar` initially. A gate's `>` comparison
rejects +inf D², so `-inf` ranges are rejected; a NaN D² would be accepted.

### 4.9 Defaults (code defaults → shipped `params/*.yaml` at `e4179f48`)

| Parameter | Code default | Shipped |
|---|---|---|
| `enable_partial_update` | true | true |
| `estimator_dt` | 0.002 | (not set) |
| `mahalanobis_d_sonar` / `_mocap_z` / `_mocap_velocity` / `_rgbd_velocity` | 20 / 20 / 20 / 20 | 20 / 20 / 50 / 80 |
| `enable_measurements` | true | false |
| `enable_mocap_switch` | false | true (**disabled for simulation** in all fixtures) |
| z_P0 / z_P0_flying | — | diag(0.025, 1.0, 0.01) / diag(0.25, 1, 0.01) |
| z_Q, z_R0, z_R_flying, z_x0, z_beta | — | [0.03, 0.0001], 0.04, 0.00016, [−0.25, 0, 0], [1, 1, 0.5] |
| xy_P0, xy_Q, xy_R0, xy_beta | — | diag(0.01, 0.01, 0.0031, 0.0031, 0.141, 0.141), diag(0, 0, 0.03, 0.03, 0.1, 0.1), diag(0.02, 0.02), [1, 1, 0.03, 0.03, 0.001, 0.001] |

## 5. Master vs simulation revision: every behavioural difference

Traced with `git diff e4179f48 95987b51`. Column "origin" is the commit that
introduced the master behaviour.

| # | Area | master `e4179f48` | sim `95987b51` | Origin | Fixture evidence |
|---|---|---|---|---|---|
| 1 | χ² gates (sonar, mocap Z, mocap XY, RGB-D) | active (`xyz_estimator.cpp:242, 260, 274, 292`) | commented out (`:234, 252, 266, 284`) | sim `95987b5` | `s10`: sim accepts 3.5 m and `-inf` (state NaN); `s15`: sim can start airborne |
| 2 | Z model rebuild | F, B, Q from measured dt every step (`z_estimator.cpp:94-107`, `xyz_estimator.cpp:88, 184`) | built once with dt = 0.002 (`z_estimator.cpp:10-20`); Q = z_Q · estimator_dt once (`xyz_estimator.cpp:87`) | master `e4179f4` | `s09` (250 Hz): sim ż 2.01× truth |
| 3 | F bias coupling | F(1,2) = −dt, F(0,2) = 0 | F(1,2) = +dt, F(0,2) = dt²/2 | master `e4179f4` | `s04`: b → +0.095 vs −0.095 for a +0.2 offset |
| 4 | Gravity init | measured, then overridden to 9.81 (`:136-138`) | measured magnitude | master `e4179f4` | `s08`: master bias absorbs 0.00335 |
| 5 | XY flag after a partial update | **not cleared** (`:214-222`) | cleared (`:208-214`) | master `e4179f4` (brace restructuring) | `s07`: master re-fuses 999/999 steps; P_vx 7.4e−4 vs 0.599 |
| 6 | Z noise and covariance (shipped) | P0 and P0_flying bias 0.01; z_Q bias 0.0001; z_R_flying 0.00016 | 0.09; 0.001; 0.0016 | master `e4179f4` | shipped-parameter runs |
| 7 | `enable_measurements` (shipped) | false (RGB-D dropped) | true | master `e4179f4` | `s13` uses a launch-level override (true) for both |
| 8 | Files | `LICENSE` (MIT), `params/wren_camera.yaml` | absent | master `f23b932`, `39a4c6b` | none (no behaviour) |

Unchanged between them: `estimator.cpp`, `xy_estimator.*`, `sensor_manager.*`,
`xyz_estimator.h`, `xy_est_params.yaml`, and the launch files.

## 6. Legacy defects and limitations in the baseline (characterized, preserved)

| ID | Defect or limitation | Where | Regression |
|---|---|---|---|
| D1 | After a **partial** XY update the flag is never cleared, so the last mocap/RGB-D measurement is re-fused at every IMU step until a new one arrives. The velocity estimate is pinned to it and the covariance becomes overconfident. Likely accidental (brace change in `e4179f4`) | `xyz_estimator.cpp:214-222` | `s07` |
| D2 | Z bias convention (true acceleration = u − b) is the opposite of the XY filter's use of the same bias (added into s). Since `e4179f4` the two filters disagree; the coupling scales with tilt | `z_estimator.cpp:97-99` vs `xy_estimator.cpp:73-85` | `s04` (convention); tilt coupling not yet quantified |
| D3 | Gravity is hard-coded to 9.81. A different local g or accelerometer scale error is absorbed into the Z bias | `:137` | `s08` |
| D4 | Cannot start in the air: the first range is gated against z0 = −0.25 and rejected, so takeoff never triggers | `:300-325`, `:443` | `s15` |
| D5 | XY gates use the previous message's R, not the new one's covariance | `:327-351, 382-408` | covered by the independent gate check |
| D6 | Range is used as vertical altitude (no tilt compensation) | `:167, 262` | `s05` (2.35 cm at 10°) |
| D7 | float32 truncations: range and the gating arguments (`float` parameters), the Z bias passed into XY | `:258-263, 300, 353`; `xy_estimator.cpp:52` | reproduced exactly by the independent check |
| D8 | Only the acceleration is NaN-checked; a NaN attitude is not | `:150` | none |
| D9 | A skipped NaN sample makes the next dt twice as long | `:156-164` | `s11` |
| D10 | Wrong-length matrix parameters leave the matrix uninitialized (logged, not fatal) | `matrix_operation.h:60-63` | not preserved: the port rejects invalid parameters at startup (P03, `test_helpers_legacy`, `test_ros_parameters`) |

## 7. Approved deviations and correction candidates

**Approved for the port (middleware only, no effect on the numbers):** ROS 2
parameters, subscriptions, publishers, and logging; `sensor_msgs`/`reef_msgs`
ROS 2 types; transient-local QoS for the formerly latched outputs; an explicit
RC switch parameter disabled in simulation; a single-threaded executor with
one mutually exclusive callback group. P03 refined the last point
([INTERFACES.md §3.8](INTERFACES.md#38-execution-model)): the live node
processes messages in executor order without a reorder buffer, as ROS 1 did,
and fidelity tests feed the ROS-free core in fixture order (stamp order).
P03 also made invalid parameters fatal instead of zero-filled or
uninitialized (D10); for valid parameters nothing changes.

P04 (vertical filter) adds, without changing any estimate for the same
ordered inputs: horizontal output fields are NaN until the horizontal
filter is ported; a `~/reset` service and a reset on a backward ROS time
jump (both only when triggered); throttled rejection logs; logged (not
altered) IMU stamp anomalies. Evidence that the vertical port reproduces
master: `check_vertical.py` (since P05 `check_port.py`), 40 event streams bit-identical per event
(docs/reviews/P04.md).

P05 (combined estimator) adds, again without changing any estimate for the
same ordered inputs: a `diagnostics` topic (callback timing, observation
accounting), and an optimized default build (-O2, as the reference harness).
Correction **C1 is implemented opt-in** (`correction_c1_clear_xy_flag`,
default false, NOT APPROVED) so that R1 can evaluate it; with it off, all 50
parity streams are bit-identical to master. P05 evidence for C1 (faults
F1/F11): with master's D1, a velocity dropout during motion keeps the stale
observation fused, so the velocity variance stays small and, when
observations return, every one is rejected by the gate (a permanent
lock-out: 2629 of 2629 rejected in the simulation fault case, error 0.51 m/s
RMS); with C1 the first observation after the dropout is accepted and the
error returns to 0.015 m/s RMS.

**Must be preserved for fidelity:** everything in section 4, including D1–D9
(D10 concerns invalid input only; see above) and the float32 conversions (they affect gating decisions exactly).

**R1 decisions (USER, 2026-09-30, after a Claude self-review of `49f7073`,
STATUS §5f):** C1 approved and on by default; C2–C5 deferred; the IMU
vibration assumption stays; C6 not decided (deferred). R1 finding 1 (the
node published after the takeoff check, master before it) was a port defect,
fixed: messages carry the state at master's publish point, and the published
messages are now compared with what the original published (harness A6).

**Correction candidates (before R1).** USER decision, 2026-09-30: "We will
defer all algorithm corrections (C1-C5) to the independent review (R1) to
guarantee our initial ROS 2 port is a bit-exact match of the legacy master
branch." P03–P05 therefore implement none of them; the port is scored against
the unmodified master golden (tolerances in section 8: discrete fields exact,
continuous fields within 1e−9 of field scale, which covers only FMA/ordering
rounding). After R1, each approved candidate gets its own commit, tests, and
golden variant:

| ID | Candidate | Evidence it matters | Evaluation needed |
|---|---|---|---|
| C1 | Clear the XY flag after a partial update (restores the pre-`e4179f4` behaviour). **APPROVED at R1 (USER, 2026-09-30); default on.** Verified against the independent step-wise model with C1 on all 50 streams; off reproduces master bit for bit | `s07`; P05 faults F1/F11: with D1 the filter locks out after a velocity dropout | fidelity golden for "master + C1"; estimator quality on X3 data (P05: error after a dropout 0.015 vs 0.51 m/s) |
| C2 | Make the XY use of the Z bias consistent with master's Z convention | D2 | quantify the tilt coupling; check sign against the REEF paper |
| C3 | Use the measured gravity magnitude (or a parameter) instead of 9.81 | `s08` | effect on Z bias convergence |
| C4 | Allow airborne initialization (for example, skip the gate until the first accepted range) | `s15` | relevant for restarts in flight |
| C5 | Tilt-compensate the range (the source's own TODO) | `s05` | the X3 range data is a slant range |
| C6 | Revise takeoff detection: the accelerometer-magnitude variance threshold (0.5 (m/s²)² over 20 samples, `ACC_TAKEOFF_VARIANCE`) only fires with strong vibration. Added by the USER 2026-09-30 | P04 characterization: on the vibration-free simulated IMU takeoff is never declared and the landing reset pins ż at 0; the simulation runs keep a 1.0 m/s² vibration assumption instead | detector behaviour on measured or modelled vibration; effect on landing detection |

## 8. Numerical tolerances (fixed before any port exists)

| Comparison | Criterion | Measured | Justification |
|---|---|---|---|
| Independent re-derivation vs reference, per step | \|pred − ref\| ≤ 1e−14 + 1e−12·\|ref\| | **0** (bit-identical) over 266,796 steps and 41,200 gate decisions | step-level rounding is a few ULP; 1e−12 still detects terms such as the dt²/2 bias coupling (≈ 1e−10 relative) |
| **Port vs reference, whole run** | continuous fields: \|port − ref\| ≤ **1e−9 · max_run\|ref_field\|**; gate decisions, flags, takeoff/landing, reset counter: **exact**; NaN/inf classification identical | floor: `-O0` vs `-O2` **0**; FMA + `-O3 -march=native` vs `-O2` **2.05e−12** of field scale, no discrete divergence | about 500× above the measured floor; any real algorithmic difference is far larger (master vs sim ≥ **1.08e9×** the tolerance in every flight scenario) |
| Covariance sanity | ‖P − Pᵀ‖ ≤ 1e−12·‖P‖; min eig(sym P) ≥ −1e−12·‖P‖; finite | 5.9e−16; 0 (no negative eigenvalue) | the non-Joseph update keeps symmetry to ULP level in these fixtures |
| Golden (reference stability) | decimated outputs within the port tolerance, discrete exact | 0; 60/60 bit-identical full outputs | detects any change of compiler, Eigen, or harness behaviour |

Fidelity (these tolerances) and filter correctness (sections 6-7, and
ACCEPTANCE `estimator`) remain separate questions.

## 9. Reference harness and its adaptations

`baseline/` compiles the **unmodified** pinned sources, extracted from the
Git objects and checked against `baseline/provenance.json`, in their original
directory layout. Adaptations:
- **A1:** access keywords relaxed for REEF classes only, after all library
  headers (`harness/access_prelude.h`), so internals can be read.
- **A2:** minimal ROS 1 stand-in headers (`harness/shim/`) with ROS
  semantics for `Time::toSec`, `param`, `getParam`, and an enabled `ROS_ASSERT`.
- **A3:** the uninitialized diagnostic member `Mahalanobis_D_hat_square` is
  set to NaN before the first event. It is never read before being written.
- **A4:** events reach a callback only if the node would have subscribed.
- **A5:** events are delivered in fixture order to the `SensorManager`
  callbacks. That means no queues, drops, or threading: the ROS 1 queue depths
  (1 or 10) and callback concurrency are not modelled.

Not reproduced: publisher latching and timing, wall-clock effects, and the
unused hypothesis-test rejector. The `rosflight_io` launch paths are out of
scope.

## 10. Evidence and how to reproduce

```bash
scripts/reef_check.sh baseline            # full check (about 8 minutes)
scripts/reef_check.sh baseline --floor    # also rebuild with -O0 and FMA and measure the floor
```

Report: `build/baseline/report/summary.md`. The fixtures (15 scenarios × 2
parameter sets, locked by `baseline/fixtures.lock.json`) are generated
deterministically by `baseline/tools/fixtures.py`. The golden outputs are in
`baseline/golden/` (written once from the unmodified originals; any rewrite
needs `--update-golden "<reason>"`, and the reason is stored).

## 11. Remaining uncertainty

- **Which exact configuration flew.** The hardware bundle pins master, but
  the parameter files used on each vehicle are not known.
- **Corrections C1–C6** are deferred to R1 (USER, 2026-09-30). The port
  targets master as is.
- **Partial-update paper.** The formula is checked against the code and the
  cited paper's structure, not against a full reading of
  `reference/reef_estimator/docs/Partial_Update.pdf`.
- **D2's size** under realistic tilt has not been quantified.
- **Queue and threading behaviour of ROS 1** (for example, sonar messages
  dropped by a queue depth of 1) is not modelled; the fixtures assume ordered,
  lossless delivery.
