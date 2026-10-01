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

## 6. Replacement odometry

Written with its implementation (P08 step 4).
