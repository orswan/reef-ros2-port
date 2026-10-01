# REEF RGB-D velocity chain (P08)

Status: specification from source, written 2026-10-01 before the P08 port
code. Labels **[V]** read/run here, **[A]** assumption. USER decisions and
criteria: [ACCEPTANCE.md](ACCEPTANCE.md) (`vision`, P08).

## 1. Legacy chain and pins

| Component | Pin | Role | P08 |
|---|---|---|---|
| Astra camera driver (`ros_astra_camera`) | `597756cf` | RGB + depth | replaced by a Gazebo `rgbd_camera` |
| `demo_rgbd` (DEMO-RGBD, Ji Zhang, LGPL) | `934f7295` | visual odometry → `/cam_to_init` (`nav_msgs/Odometry`, camera convention: x left, y up, z forward) | **replaced** by a compact OpenCV odometry (USER, option B); not a port |
| `rgbd_to_velocity` | `b7637198` | odometry → `reef_msgs/DeltaToVel` (body-level velocity) | **ported, bit-exact** (§2) |
| REEF estimator | (P05) | `rgbd_velocity_body_frame` (`enable_rgbd`, `enable_measurements`) | unchanged |

## 2. `rgbd_to_velocity` (b7637198): equations and order [V source]

Parameters (private): `alpha` (1.0), `x_vel_covariance` (0.01),
`y_vel_covariance` (0.01), `body_to_camera_quat` (4: x, y, z, w),
`body_to_camera_trans` (3; read, **never used**). Shipped:
`params/kiwi_camera.yaml`, `params/rgbd_to_velocity_params.yaml`.

Constructor: `previous_position_init` and `previous_velocity_init` = 0,
`previous_time_stamp` = 0, then `previous_position_init` = the position of a
member `PoseStamped` that is never set (all zero).

Per odometry message (stamp t = sec + 1e-9 nsec, position p, quaternion
q = (x, y, z, w) of the camera in DEMO's init frame):

1. `counterOfSamples += 1`; `DT = t − previous_time_stamp`.
2. **Gate**: process only if `DT >= 1.0/30.0` (double); otherwise nothing
   else changes.
3. `C_init_to_cam = I − 2 w [v]× + 2 [v]×²` (v = (x, y, z)), then
   213 angles of it: `yaw = atan2(C20, C22)`, `pitch = −asin(C21)`,
   `roll = atan2(C01, C11)`.
4. `C_init_to_cam_level = [[cos ψ, 0, −sin ψ], [0, 1, 0], [sin ψ, 0, cos ψ]]`
   (ψ = that yaw); `C_init_to_NED_level = M · C_init_to_cam_level` with
   `M = [[0, 0, 1], [−1, 0, 0], [0, −1, 0]]`.
5. `C_body_to_cam` = same quaternion formula on `body_to_camera_quat`;
   `C_cam_level_to_body = C_body_to_camᵀ · diag(−1, −1, 1) · C_init_to_cam ·
   C_init_to_NED_levelᵀ`; its 321 angles: `pitch = −asin(C02)`,
   `roll = atan2(C02, C22)` (sic), `yaw = atan2(C01, C00)`.
6. `C_level = [[cos ψ, sin ψ, 0], [−sin ψ, cos ψ, 0], [0, 0, 1]]` (ψ = the 321 yaw).
7. `v_est = (p − p_prev) · (1/DT)`; `v_filt = α v_est + (1 − α) v_prev`;
   then `p_prev = p`, `previous_time_stamp = t`, `v_prev = v_filt`.
8. Init-frame message: `vel.header.stamp = stamp`, `vel.linear = v_filt`;
   **published now** (on `rgbd_to_velocity/init_frame`).
9. `v_body = C_level · C_init_to_NED_level · v_filt`; body-level message:
   stamp, `vel.linear = v_body`.
10. `Σ_init = diag(x_cov, y_cov, 0)`; `Σ_body = R Σ_init Rᵀ` with
    `R = C_level · C_init_to_NED_level`; `σ = sqrt(diag Σ_body)`;
    `covariance[0] = x_cov`, `[7] = y_cov`, `[14] = Σ_body(2,2)`;
    `S_upper = v_body + 3 (sqrt(x_cov), sqrt(y_cov), σ_z)`, `S_lower` with −.
11. Body-level message **published** (`rgbd_to_velocity/body_level_frame`,
    latched, queue 1); `counterOfSamples = 0`.

## 3. Legacy behaviour kept by the port (Q-list)

| ID | Behaviour |
|---|---|
| Q1 | The gate `DT >= 1.0/30.0` in double rejects any spacing below 33 333 333.3 ns. With nanosecond-rounded 30 Hz stamps the spacing cycles 33 333 333 / 33 333 334 / 33 333 333 ns, so **one frame in three** is rejected (2/3 accepted, effectively 20 Hz) [V fixture r01]; which frames are rejected depends on how the driver rounds its stamps (a float-derived stamp sequence alternated in an early harness test). *Corrected 2026-10-01 from "every other frame" after the fixture run.* |
| Q2 | First message: `previous_time_stamp` = 0, so DT = the stamp (large) and the first velocity is the position divided by the stamp (≈ 0) |
| Q3 | `previous_position_init` is set from a member pose that is never written (always zero) |
| Q4 | The published x/y covariances and 3σ bounds use the **unrotated** init-frame values (`x_cov`, `y_cov`), although the rotated body-level matrix is computed (only its z element is published) |
| Q5 | The 321 roll formula is wrong (`atan2(C02, C22)`); the roll is unused |
| Q6 | The init-frame message is published before the covariances and bounds are set and reuses the message object: it carries the **previous** message's covariances and bounds |
| Q7 | The velocity is a finite difference of camera positions: the camera's lever arm (`body_to_camera_trans`, read but unused) and rotation-induced velocity are ignored |
| Q8 | Rejected messages change only the counter; the next DT is measured from the last accepted stamp; backward stamps are rejected (negative DT) |
| Q9 | A NaN position latches the **velocity**: `(1 − α)·v_prev` is NaN even for α = 1 once v_prev is NaN, so every later velocity is NaN; the stored position becomes finite again at the next accepted message [V fixture r09]. *Corrected 2026-10-01 (the position does not stay NaN).* |
| Q10 | Missing or wrongly sized `body_to_camera_*` parameters gave zeros (with a warning) or uninitialized values in the original; the port **rejects** them at startup (AGENTS rule; plumbing deviation) |

## 4. Port plan

Package `src/rgbd_to_velocity` (history imported, renames, then plumbing
edits only): a ROS-free core with the original statement order and Eigen
expressions; an rclcpp node (`cam_to_init` in;
`rgbd_to_velocity/init_frame`, `rgbd_to_velocity/body_level_frame` out,
reliable + transient local depth 1 as the latched ROS 1 publishers);
parameters as ROS 2 parameters (§2; Q10); an event-replay tool writing the
harness columns. Verified by `baseline/rgbd/check_rgbd.py` (bit-exact parity
core and node, independent model, Q1–Q9, negative control).

Initialization fix (USER, 2026-10-01). The original's constructor assigns an
uninitialized `Eigen::VectorXd(3)` to `translation_body_to_camera` (sized for
the ROS 1 parameter helper, which then overwrote it). A fresh GCC 13 build
reports this as `-Wmaybe-uninitialized` [V]. The port uses `setZero()`
instead. All three elements are set from the parameter on the next line, and
the translation reaches no output (Q7), so outputs are unchanged [V]:
`check_rgbd.py` 53/53 bit-exact after the change, and a fresh build has no
warning. The pinned original in the harness is unmodified.

## 5. Camera and scene (P08 step 3) [V]

Generated by `scripts/make_vision_assets.py` (deterministic; `--check`
verifies the committed files; `test_vision_assets.py`):

| Item | Value |
|---|---|
| Camera | Gazebo `rgbd_camera` on `X3/base_link` of `reef_x3_rgbd` (= `reef_x3` + camera), 320×240 at 15 Hz (USER profile; fallback 160×120 at 10 Hz), hfov 60°, clip 0.3–8.0 m, `gz_frame_id` `x3/camera_optical` |
| Mounting | kiwi's position (FRD 0.1355, 0.0175, −0.0455 m) with an ideal forward orientation; converter extrinsics `config/x3_sim_camera.yaml` (`body_to_camera_quat` (0.5, −0.5, 0.5, −0.5)); the real kiwi calibration is a few degrees off this |
| Scene | `reef_vision_scene` (own content): a wall at x = 4 m, textured for y ∈ [−3, 1.5] (procedural multi-scale blocks, seeded) and plain grey for y ∈ [1.5, 6] (weak texture); the ground plane is plain (weak texture); a red emissive sphere at (3.5, −1.0, 1.4) for the projection check |
| Worlds | `x3_vision.sdf` (stock controller, open loop) and `x3_closed_loop_vision.sdf`, generated from `x3_flight.sdf` / `x3_closed_loop.sdf` with the Sensors system (ogre2); the server runs `-s --headless-rendering` (EGL, software rendering here) |
| ROS topics | `/x3/camera/image` (`rgb8`), `/x3/camera/depth` (`32FC1`), `/x3/camera/camera_info` (own bridge, `config/bridge_camera.yaml`) |

Measured conventions (run `recordings/p08_vision_iface`, `camera_check.json`):
fx = fy = 277.128 = w / (2 tan(hfov/2)); principal point (160, 120) with
pixel (i, j) covering [j, j+1) (the target's centroid + 0.5 px matches the
pinhole projection through the true pose within **0.03 px**); no
distortion; depth **planar** (along the optical axis) in metres (wall
3.8645 m = truth); invalid depth **−inf** nearer than the near clip and
**+inf** beyond the far clip (no NaN); RGB and depth share stamps; frames
every 33 physics steps (0.066 s, 15.15 Hz, sim time). Raw images are not
recorded (about 3.5 MB/s); `camera_check` saves sample frames.

Vision flight (`run_x3_scenario.sh --vision`, `config/x3_vision.yaml`; the
stock truth-fed controller flies, vision is not in the loop): textured
legs (forward, back, right, left at 0.3 m/s), weak texture (left to y ≈ 4.0 m:
only the plain wall in view; the stock controller reaches about 92 % of the
commanded displacement), depth loss (back to x ≈ −4.8 m: the wall about
8.7 m away, beyond the far clip), descent to 0.4 m.

## 6. Replacement odometry (P08 step 4): design, fixed before the code

**A replacement, not a port** of `demo_rgbd` (USER, option B). Package
`src/reef_rgbd_odometry`: a ROS-free OpenCV 4 core and an rclcpp node.

| Stage | Design |
|---|---|
| Input | `/x3/camera/image` (rgb8 → grey), `/x3/camera/depth` (32FC1 planar m), `/x3/camera/camera_info` (K); RGB and depth paired by identical stamp |
| Features | Shi-Tomasi corners (≤ 300, quality 0.01, min distance 8 px), replenished below 120 tracks, masked around existing tracks (DEMO's front-end also tracks corners with KLT) |
| Tracking | pyramidal Lucas–Kanade (21×21, 3 levels) with a forward-backward check (≤ 1 px) |
| Depth | **(rev. 1)** bilinear over the 4 neighbouring pixel centres, in the previous and the current frame; rejected if any neighbour is invalid (outside [near, far]: −inf/+inf/NaN never used) or they differ by more than 5 % (a discontinuity). (Rev. 0: the previous frame's nearest pixel.) |
| Motion | **(rev. 1)** 3-D (previous) to 3-D (current) rigid motion: RANSAC over 3-point samples (Kabsch/Umeyama, 200 iterations, seeded), inlier if the 3-D residual ≤ 0.1 % of the depth (3.9 mm at 3.9 m ≈ 0.28 px laterally), then Kabsch on all inliers. (Rev. 0: 3-D to 2-D PnP-RANSAC, 2 px, LM refine; see the revision note below.) |
| Pose | chained in the first frame's optical frame (x right, y down, z forward); output in **DEMO's convention** (x left, y up, z forward): p_demo = F p, R_demo = F R F with F = diag(−1, −1, 1); orientation = the camera's orientation in the init frame (the converter's formula turns it into C_init→cam) |
| Health | LOST if < 40 depth-valid tracks (depth loss), < 25 RANSAC inliers (weak texture), or a step > 0.5 m or > 0.3 rad; while LOST **nothing is published** (no fallback, never truth); publishing resumes after 2 consecutive good frames |
| Recovery | the motion while LOST is unknown: the chain resumes from the last published pose, so the converter (legacy) computes one low velocity from the first message after a loss (documented; REEF's gate and covariances decide) |
| Outputs | `cam_to_init` (`nav_msgs/Odometry`, stamp = the frame's stamp); `vo/health` (`diagnostic_msgs/DiagnosticArray`: state, tracks, depth-valid tracks, inliers, reprojection RMS, processing time [wall ms], frame age [sim ms], counters) |
| Test hooks (labelled, off by default) | `fault_schedule`: `delay PHASE OFFSET SPAN D` holds frames stamped from OFFSET to OFFSET + SPAN s after the start of scenario phase PHASE until stamp + D (sim time); `drop PHASE OFFSET SPAN` discards them (phases from `/x3/scenario/phase`) |

**Revision 1 (2026-10-01, after the first scored run; criteria and limits
unchanged).** [V] The first run (`recordings/p08_vision_open1`) passed the
steady-segment limits but failed three degraded-case items. A diagnostic run
with raw frames (`REEF_X3_RECORD_CAMERA=1`, replayed offline with
`vo_replay`) showed:
- Rendering is exact: the red target matches the truth projection at the
  image stamp within about 2 ms (0.08 px) while moving.
- Rev. 0 underestimated motion parallel to the image plane: lateral −5 %,
  ascent −20 %. PnP on the single fronto-parallel wall explained part of the
  translation as a rotation: a spurious pan of 0.215 °/s × 3.86 m = 0.0145
  m/s, against a lateral deficit of 0.013 m/s. After a loss, with a small or
  far feature set, the errors reached 1 m/s.
- The current frame's depth separates translation from rotation (3-D to 3-D).
  The inlier threshold must lie below the per-frame motion (≈ 1.6 cm at
  0.25 m/s and 15 Hz). At 0.5 % of depth a wrong tilt hypothesis won the
  ascent.

Replay of the same frames, mean error per phase:
- rev. 0: up to 0.05 m/s (textured) and 1 m/s (after depth loss);
- rev. 1: ≤ 0.008 m/s in every phase, error std ≤ 0.013 m/s.

**[A] Limitation:** the 0.1 % threshold relies on the simulated depth being
exact (Gazebo adds no depth noise). A real sensor needs a threshold matched to
its depth noise. Revision 1 is judged by a fresh live run, not by the replay.

Simulation chain for vision runs: camera → odometry → `rgbd_to_velocity`
(`config/x3_sim_camera.yaml`) → REEF estimator with `enable_rgbd`,
`enable_measurements` true and **`enable_mocap_xy` false** (no truth-derived
velocity; criteria: ACCEPTANCE vision).

## 7. Scoring definitions (P08 steps 5–7), fixed before the first scored run

`ros2 run reef_sim analyze_vision RUN_DIR` (run by `run_x3_scenario.sh
--vision`) turns the ACCEPTANCE `vision` rows into measurable windows. These
are definitions of terms the criteria leave open (“steady segments”, “the
loss”, “after the segment”); the limits are ACCEPTANCE's, unchanged.
Truth is used for scoring and for the scene geometry only.

| Term | Definition |
|---|---|
| Steady segments | phases with the rich texture in view and the wall inside the clip range: hover_start, forward, hover_fwd, back, hover_back, right, hover_right, left, hover_left, hover_ret, hover_end, hover_low, each without its first 1.0 s (controller transient). REEF samples additionally from 2 s after REEF's takeoff (as P05) |
| Truth velocity | body-level (x forward, y right, yaw-aligned) from `/x3/truth/odom`, interpolated to each message stamp (`analyze_reef.truth_at`) |
| Rate | vision velocity messages (`rgbd_to_velocity/body_level_frame`) per sim second in the steady segments |
| Sign/frame | forward leg → mean vision x > +0.1 m/s, back → < −0.1, right → mean y > +0.1, left → < −0.1, cross axis below half of the primary |
| Weak-texture event / end | event: first time in weak_left at which the right image edge ray meets the wall beyond the rich region (y > 1.5 m; no rich texture in view); end: first time in weak_return at which the optical axis meets the rich region again |
| Depth-loss event / end | event: first time in far_back at which the planar depth of the wall at the image centre exceeds the far clip (8 m); end: first time in far_return at which it is back within |
| Loss shown | a health message (stamp = image stamp) with state LOST between the segment's phase start and event + 0.5 s |
| No publication while lost | no `cam_to_init` and no `rgbd_to_velocity` message stamped strictly inside any LOST interval (first LOST stamp → next published stamp), whole run |
| Variance grows | REEF's σ(ẋ) and σ(ẏ) at the last estimate before the resume exceed those at the loss |
| Resume | the last LOST → published transition between the segment's phase start and the end of the following hover phase, at most 1.0 s after the segment end |
| REEF recovered | REEF velocity RMSE per axis over [end + 3 s, end + 4 s] ≤ 0.10 m/s |
| Faults run | `REEF_X3_VISION_FAULTS=1` merges `config/x3_vision_faults.yaml` (`delay right 0 5 0.2`, `drop left 2 1`). Per window [phase start + offset, + span] extended by 2 s: every vision velocity finite and per-axis error ≤ 0.3 m/s. Latency (bag receive − stamp, both sim) and rate are REPORTED; the nominal items are REPORTED only in that run |
| REPORTED (never counted as PASS) | noise vs configured covariance, latency, gate and fusion counts, performance (odometry wall ms per frame, processing share of one core, image rate, RTF) |

## 8. Results (P08 steps 4–7, odometry rev. 1) [V]

Container: original (`ros2_novnc_container`), headless, idle machine.
Branch `p08-rgbd`, uncommitted rev. 1 working tree. The stock controller
flies on truth; REEF is open loop. REEF's attitude input is the truth
attitude (idealized); its only horizontal velocity input is vision.

Runs:
- `recordings/p08_vision_open1`: rev. 0, the first scored run. 17/20: the
  weak-texture REEF recovery, the depth-loss resume (+1.14 s) and the
  depth-loss REEF recovery failed.
- `recordings/p08_vision_open2`: rev. 1, nominal, **20/20**.
- `recordings/p08_vision_faults2`: rev. 1, faults, **24/24**.
- `reef_check.sh vision` (`log/checks/reef_check_vision_20261001_193029`): exit 0; its own
  nominal run 20/20 and faults run 24/24 (vision RMSE x 0.0004, y 0.0013 m/s; REEF
  x 0.028, y 0.033 m/s; depth-loss resume +0.72 s).

An earlier faults run used the overlay key `reef_rgbd_odometry`, which does
not reach the node in `/x3/reef`, so the hooks never acted. That analyzer
version still passed the fault items vacuously. The analyzer now judges
"the test hook acted" and cross-checks the manifest, which fails that run
(kept in the scratchpad, not in `recordings/`).

| Item (ACCEPTANCE vision) | Nominal (open2) | Faults run (faults2) |
|---|---|---|
| Vision velocity vs truth, steady (RMSE, bias) | x 0.0004 / +0.0000, y 0.0014 / −0.0000 m/s | x 0.0005, y 0.0012 m/s (REPORTED) |
| Rate in steady segments | 13.61 Hz (≥ 10) | 13.00 Hz |
| Sign/frame (forward +x, back −x, right +y, left −y) | 4/4 (means ±0.274–0.275 vs truth ±0.274) | 4/4 |
| REEF on vision only, steady (RMSE) | x 0.031, y 0.030 m/s (≤ 0.10) | x 0.026, y 0.029 m/s |
| Data path (ROS graph) | REEF fed by `/x3_imu_adapter`, `/range_sensor`, `/x3/reef/rgbd_to_velocity_node` only | same |
| Weak texture: loss shown / resume / σ grows / REEF recovered | −0.86 s / −6.75 s (on partial texture) / 0.15 → 2.90 m/s / x 0.029, y 0.040 m/s | −0.88 s / −6.91 s / 0.13 → 2.82 / 0.031, 0.035 |
| Depth loss: loss shown / resume / σ grows / REEF recovered | −0.18 s / +0.49 s / 0.05 → 2.83 / x 0.021, y 0.031 | −0.21 s / +0.45 s / 0.11 → 2.80 / 0.021, 0.020 |
| No publication while LOST | 0 messages inside 2 LOST intervals | 0 inside 3 |
| Delay 200 ms (right, 5 s) | — | hook acted (median latency 216 ms); max error 0.021 m/s (≤ 0.3) |
| Drop 1 s (left) | — | hook acted (15 frames, stamp gap 1.056 s); max error 0.004 m/s |

REPORTED (not counted):
- **Noise vs covariance.** Measured error std is 0.0004 (x) and 0.0014 (y)
  m/s; the converter publishes σ = 0.1 m/s (`x/y_vel_covariance` 0.01, the
  kiwi value), 70–250× conservative for the simulated camera. The REEF error
  (0.03 m/s) is dominated by REEF, not by the vision input.
  [A] Contributors: measurement latency (below) and REEF's propagation with
  the IMU vibration assumption.
- **Latency** (bag receive − image stamp, sim time): median 102 ms, p99
  408 ms. Rendering is asynchronous in sim time. REEF fuses each measurement
  on arrival as current.
- **Gate counts (nominal):** 1409 gate evaluations, all 1409 accepted (none
  rejected; `mahalanobis_d_rgbd_velocity` 80), 1373 fused.
- **Performance:** odometry 9.3 ms mean, p95 15.7 ms per frame wall time;
  about 11 % of one core; camera 15.2 Hz; odometry 13.5 Hz processed (sim);
  RTF 0.88.

Characterizations (USER 2026-10-01: kept as documented behaviour; no sample
suppression and no boundary filters) [V]:

**V1. Near-zero first sample after a recovery.**
- Mechanism: while LOST the odometry keeps its last published pose and
  re-detects features. Tracking from the re-detected frame is chained but
  not published until `recover_frames` (2) consecutive good frames. The first
  published pose therefore differs from the last one before the loss only by
  the motion of those frames.
- The converter (legacy) differentiates across the whole gap (DT = stamp
  gap, 8.8–9.5 s here; `alpha` 1, so no filter memory).
- Observed: the first sample is 1–3 % of the true speed:
  - open2: +0.008 vs truth 0.275 m/s (y), and +0.005 vs 0.367 m/s (x);
  - reef_check run: +0.004 vs 0.275, and +0.008 vs 0.368 m/s.
- The next sample is normal: within 0.02 m/s (y, weak texture) and 0.001 m/s
  (x, depth loss).
- REEF accepts the sample (its variance is large after the loss; the gate
  rejected none). REEF's error peaks at 0.39–0.51 m/s within 1 s after the
  resume, partly drift accumulated while lost, and is ≤ 0.042 m/s over
  [end + 3 s, end + 4 s] (judged).

**V2. Wrong pose shortly before a depth loss (intermittent).**
- As the wall approaches the far clip, depth-valid tracks and inliers fall:
  open2 94.05–94.25 s, inliers 95 → 38, still above `min_inliers` 25, until
  LOST at 94.32 s.
- In open2 one pose in this tail was wrong by about 3 cm laterally. It
  produced a pair of samples with opposite errors: y −0.449 m/s at 94.184 s,
  then +0.512 at 94.250 s, against truth +0.002.
- The `reef_check` run had no such pose (max error 0.023 m/s before its loss
  at 94.78 s), nor did faults2 (max 0.008 m/s before its loss at 93.59 s).
- REEF's error stayed ≤ 0.09 m/s (both samples accepted). The ACCEPTANCE
  0.3 m/s error limit applies to the delayed and missing frame cases only.
