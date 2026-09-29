# REEF Estimator: ROS 1 → ROS 2 Jazzy migration notes

Status: **starter phase.** The reference sources are inspected, the environment
is characterized, and the sim-time bridge is demonstrated. No REEF code has
been ported yet.

Evidence labels used throughout:

- **[V] Verified**: observed directly in this container or in the pinned
  upstream sources on 2026-09-29.
- **[A] Assumption**: plausible but not yet tested here. Treat it as a hypothesis.

---

## 1. Working environment

| Item | Value | |
|---|---|---|
| Host | Intel Mac, Docker container `ros2_novnc_container` | [A] as reported by user; container name not queried from inside |
| OS / arch | Ubuntu 24.04.4 LTS, x86_64, kernel 7.0.12-linuxkit | [V] |
| ROS | ROS 2 Jazzy, `/opt/ros/jazzy` | [V] |
| Gazebo | Harmonic, `gz sim` 8.15.0 (via `gz_*_vendor` packages in `/opt/ros/jazzy/opt`) | [V] |
| ROS–Gazebo | `ros_gz_bridge`, `ros_gz_sim`, `ros_gz_interfaces`, `actuator_msgs` installed | [V] |
| Display | Xvfb `:99` → x11vnc → websockify `:8080` → http://localhost:8080/vnc.html, started by `/root/start_vnc.sh` | [V] processes running; fluxbox not running at inspection time (not required) |
| Rendering | `LIBGL_ALWAYS_SOFTWARE=1`, `MESA_GL_VERSION_OVERRIDE=3.3` (software GL) | [V] Gazebo GUI window mapped on `:99` |
| Workspace | `/root/ros2_ws` bind-mounted from macOS (virtiofs) | [V] mount type; not itself a Git repo |
| Git identity | `user.name=orswan`, `user.email=orswan@stanford.edu` (global) | [V] |
| Network | github.com and fuel.gazebosim.org reachable | [V] |
| Recordings | None available | [V] as stated; so offline validation must use simulation |

### Environment hazards found

1. **[V] Agent shells inherit an unrelated overlay.** `~/.bashrc` sources
   `/root/ros2_ws/install/setup.bash`, which contains tutorial packages. The agent's
   shell already had those on `AMENT_PREFIX_PATH`, and it had `DISPLAY=:1`, not
   `:99`. All `scripts/*.sh` therefore re-exec under `env -i` and source only
   `/opt/ros/jazzy` (plus this repo's own `install/` once it exists).
   See `scripts/env.sh`.
2. **[V] Jazzy always sets `ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET`** (from the
   `ros_environment` hook). The scripts override it to `LOCALHOST`. Set
   `REEF_DISCOVERY_RANGE` to change that.
3. **[V] The parent workspace crawls into this repo.** `colcon list` run from
   `/root/ros2_ws` discovers packages under `reef_ros2/`. With
   `reference/COLCON_IGNORE` removed, it picked up the ROS 1
   `reef_estimator` (`ros.catkin`). The marker is essential. Packages later
   added under `reef_ros2/src/` **will** also be built by a `colcon build` in
   `/root/ros2_ws`. Always build from `/root/ros2_ws/reef_ros2`.
4. **[V] `gz sim` puts its server and GUI in separate process groups.**
   Killing the launch's process group leaves them running.
   `scripts/check_clock_demo.sh` tears down by session ID instead.
5. **[A] macOS bind mount.** The underlying filesystem may be case-insensitive,
   and file-watching and permissions may differ from native Linux. Keep file
   names case-unique.

---

## 2. Reference sources (inspection only; never built)

The repositories are cloned into `reference/`. They are ignored by Git and by colcon (`COLCON_IGNORE`).
Submodules were **not** fetched. Bundle manifests were read from the
`.gitmodules` files and the gitlink pins, and individual `package.xml` files were
fetched at the pinned commits.

| Repository | Branch | Checked-out commit | Commit date / subject |
|---|---|---|---|
| uf-reef-avl/reef_estimator | master | `e4179f48c3f26e22bd1366b71ee1e117ce2f5f7f` | 2021-03-08 "changes for dt error" |
| uf-reef-avl/reef_estimator_bundle | master | `a20b4e1d81f0c559bbf7e45108332b11b5d27d26` | 2023-10-09 "Update README.md" |
| uf-reef-avl/reef_estimator_sim_bundle | master | `1b5724f1d08a9d7f7d2bf4f20a9f291719f950b6` | 2022-07-06 "Merge pull request #3 …" |

`reef_estimator_2` is **not** a ROS 2 port and is not used as a starting point.

### Submodule pins [V]

| Submodule | hardware bundle pin | sim bundle pin |
|---|---|---|
| reef_estimator | `e4179f48` (= master) | **`95987b51`** (branch `simulation`, 2019-11-22 "removed measurement rejection for gazebo sim") |
| reef_msgs | `7fb63ff9` | `7fb63ff9` |
| reef_control | `12237b76` | `fa4ffa39` |
| rosflight (rosflight/rosflight) | `44e5f37e` (2019-10-21) | `44e5f37e` |
| position_to_velocity | `126dae14` | `414fcdc8` |
| reef_teleop | `d13b0523` | n/a |
| rgbd_to_velocity | `b7637198` | n/a |
| demo_rgbd | `934f7295` | n/a |
| ros_astra_camera | `597756cf` | n/a |
| ros_vrpn_client | `f48e2725` | n/a |
| sim_helper | n/a | `df8f1705` |
| setpoint_generator | n/a | `aa28ee42` |
| dubins_path | n/a | `5a740a6d` |

**[V] The estimator versions differ.** The sim bundle's estimator is the
`simulation` branch. Compared with master, its diff touches measurement rejection
in `xyz_estimator.cpp`/`z_estimator.cpp` and `z_est_params.yaml`. Before
porting, decide which behavior is the baseline. The recommendation is to port master
and make the Gazebo-specific rejection change a parameter.

---

## 3. Upstream estimator facts [V]

- Single C++ executable `reef_estimator` (catkin; C++ with Eigen). Classes:
  `Estimator` (KF with partial update) → `XYEstimator` (EKF), `ZEstimator` →
  `XYZEstimator` (data flow, outlier rejection) → `SensorManager` (callbacks).
- `sensor_manager.h` includes `../../reef_msgs/include/reef_msgs/dynamics.h`
  by **relative path**, which assumes the bundle's directory layout. The port must use
  a proper exported include.
- Interfaces (default names; many are remapped in launch files):

| Direction | Topic | Type | Condition |
|---|---|---|---|
| sub | `imu/data` | `sensor_msgs/Imu` | always |
| sub | `sonar` | `sensor_msgs/Range` | `enable_sonar` |
| sub | `rc_raw` | `rosflight_msgs/RCRaw` | `enable_mocap_switch` |
| sub | `mocap_ned` (param `mocap_pose_topic`) | `geometry_msgs/PoseStamped` | mocap Z |
| sub | `mocap_velocity/body_level_frame` (param) | `geometry_msgs/TwistWithCovarianceStamped` | mocap XY |
| sub | `rgbd_velocity_body_frame` (param) | `reef_msgs/DeltaToVel` | `enable_rgbd` |
| pub | `xyz_estimate` | `reef_msgs/XYZEstimate` (latched) | always |
| pub | `xyz_debug_estimate` | `reef_msgs/XYZDebugEstimate` (latched) | debug |
| pub | `is_flying_reef` | `std_msgs/Bool` (latched) | always |
| pub | `sonar_ned` | `sensor_msgs/Range` | sonar |

- Parameters come from `params/{basic_params,xy_est_params,z_est_params}.yaml`
  (flat matrices as lists) and the camera-to-body YAMLs.
- Conventions: NED world frame, and "body-level" (yaw-only-rotated) velocity frame.
  Gazebo is ENU/FLU, so every simulated input needs an explicit frame conversion.

The ROS 1 simulation (`sim_helper/launch/sim_estimator.launch`, sim pin) ran
`reef_estimator` with `enable_sonar: true, enable_mocap_xy: true,
enable_mocap_z: false`, fed by `position_to_velocity` from
`multirotor/truth/NED`. It also ran `reef_control_node` and a setpoint generator. `Master.py` launches
rosflight SIL (`rosflight_io _udp:=true`) and an RC override script. [A] The
vehicle and physics came from Gazebo Classic `rosflight_sim`. The
`camera_multirotor.launch` contents have not been read yet.

---

## 4. Minimum dependencies

### 4.1 Estimation (smallest useful port)

| ROS 1 dependency | ROS 2 Jazzy replacement | Status |
|---|---|---|
| catkin, roscpp | ament_cmake, rclcpp | [V] installed |
| rospy (only `scripts/verify_estimates.py`, plotting) | rclpy | [V] installed. Scripts are not needed for the core node |
| geometry_msgs, sensor_msgs, std_msgs | same names | [V] installed |
| tf2_eigen | tf2_eigen (header is now `tf2_eigen/tf2_eigen.hpp`) | [V] installed; [A] header rename applies |
| tf_conversions | drop; use tf2 / Eigen directly | [A] only lightly used |
| Eigen 3 | libeigen3-dev 3.4.0 + eigen3_cmake_module | [V] installed |
| **reef_msgs** (11 msgs + `dynamics`/`matrix_operation` lib) | port to ROS 2 (`rosidl_default_generators` + a C++ library) | [V] message list read at pin `7fb63ff9`; **must be ported first** |
| **rosflight_msgs** (`RCRaw` only, for the mocap switch) | option (a): make it optional/compile-out; option (b): `rosflight_msgs` 2.0.0 from rosflight/rosflight_ros_pkgs `main` (`5ef20134`) | [V] not in Jazzy apt; [V] upstream ROS 2 package exists; [A] `RCRaw` still defined there |

Minimum estimation stack = **`reef_msgs` (ROS 2) + `reef_estimator` (ROS 2)**.
Leave the `RCRaw` mocap switch behind a build option.

### 4.2 Control

| Package | ROS 1 dependencies | ROS 2 notes |
|---|---|---|
| reef_control (sim pin `fa4ffa39`) | roscpp, rospy, geometry_msgs, reef_msgs, rosflight_msgs, dynamic_reconfigure, message_generation | Publishes `rosflight_msgs/Command` on `command`. Subscribes `desired_state`, `xyz_estimate`, `is_flying`, `status`, `rc_raw`, `pose_stamped`. `dynamic_reconfigure` → ROS 2 parameters + `on_set_parameters` callback. [V] manifest + topics |

[A] For Gazebo Harmonic without rosflight, `rosflight_msgs/Command` (attitude
+ throttle) has no native consumer. Section 6 covers the options.

### 4.3 Simulation

| Need | ROS 1 (sim bundle) | ROS 2 Jazzy / Harmonic |
|---|---|---|
| Simulator | Gazebo Classic + rosflight_sim [A] | Gazebo Harmonic 8.15 [V] |
| Clock | `/use_sim_time` + Gazebo Classic | `ros_gz_bridge` `/clock` + `use_sim_time:=true` [V] demonstrated |
| Vehicle + flight control | rosflight SIL firmware | Harmonic `MulticopterMotorModel` + `MulticopterVelocityControl` systems [V] present in example; [A] suitable |
| Ground truth | `multirotor/truth/NED` | `OdometryPublisher` → `/model/x3/odometry` (ENU) bridged + converted to NED [V] plugin in example; [A] conversion node needed |
| IMU | rosflight IMU | Harmonic `Imu` system + `<sensor type="imu">`. **Must be added: the X3 model has no sensors** [V] |
| Altimeter (sonar) | sim sonar | Harmonic `Altimeter`/`AirPressure` sensor or a downward range sensor; [A] a thin adapter to `sensor_msgs/Range` |
| Mocap | truth → `position_to_velocity` | same pattern from bridged odometry [A] |
| Setpoints | setpoint_generator / dubins_path (rospy) | out of scope for minimum; port later |

---

## 5. Sim-time bridge demonstration [V]

Files: `sim/worlds/clock_demo.sdf`, `sim/launch/clock_demo.launch.py`,
`scripts/run_clock_demo.sh`, `scripts/check_clock_demo.sh`, and
`scripts/clock_check.py`.

`clock_check.py` subscribes to `/clock` for a wall-clock window. It asserts that
messages arrive, that sim time increases monotonically, and that a node with
`use_sim_time=True` sees `get_clock().now()` advance.

Results on 2026-09-29 (run from an agent shell that had the leaked overlay and
`DISPLAY=:1`):

| Mode | Window | /clock msgs | Sim Δt | RTF | Result |
|---|---|---|---|---|---|
| headless (`REEF_HEADLESS=1`) | 5 s | 4791 (958 Hz) | 4.889 s | 0.98 | PASS |
| GUI on `:99` | 10 s | 9302 (930 Hz) | 9.454 s | 0.95 | PASS |
| GUI on `:99`, job-control caller | 10 s | 8763 (876 Hz) | 8.930 s | 0.89 | PASS, no leftover processes |
| negative: no simulator | 3 s timeout | 0 | n/a | n/a | FAIL (exit 1), as expected |

The GUI window ("Gazebo Sim", `gz-sim-gui`, 1000×845) was confirmed mapped on
`:99` through `xwininfo`. No screenshot tool is installed, so the image was not
captured. View the window in the browser at http://localhost:8080/vnc.html.

---

## 6. Candidate Harmonic multicopter example for the next task

**Recommended: `multicopter_velocity_control.sdf`** (in
`/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/`). [V] file present.
The simpler `quadcopter.sdf` has only motor models, so it has no closed-loop
controller and no odometry.

What it contains [V]:

- World `multicopter`, 4 ms physics step, systems: Physics, SceneBroadcaster,
  UserCommands, Sensors (ogre2).
- Model **X3** (quad) from Fuel
  `https://fuel.gazebosim.org/1.0/OpenRobotics/models/X3 UAV/4`, plus **X4**
  (hexacopter) from `…/X4 UAV Config 1`. For our purposes, drop X4.
- Controller dependencies for X3:
  - 4× `gz-sim-multicopter-motor-model-system` (`robotNamespace` X3,
    `commandSubTopic gazebo/command/motor_speed`, velocity motors).
  - `gz-sim-multicopter-control-system` (`MulticopterVelocityControl`):
    input `gz.msgs.Twist` on `/X3/gazebo/command/twist` (body-frame linear
    velocity + yaw rate). The enable topic is `/X3/enable` (`gz.msgs.Boolean`); [A] it may need
    `true` before the vehicle responds. Gains are in the SDF.
- Ground-truth dependencies:
  - `gz-sim-odometry-publisher-system` (3D) publishes `gz.msgs.Odometry` on
    `/model/x3/odometry` (ENU world frame; [A] child frame FLU body).
  - Pose on `/model/x3/pose` per the file header comment [A] (verify with `gz topic -l`).
- Fuel model download: [V] `gz fuel download` of X3 succeeded (≈93 MB).
  [V] The X3 `model.sdf` contains **no `<sensor>` elements**. The IMU and
  altimeter must be added (the `Imu` and `Altimeter` systems, plus sensors on
  `X3/base_link`). [A] The first run will fetch the models from Fuel into
  `~/.gz/fuel`, so vendor the model or pre-fetch it for offline repeatability.

How it maps onto REEF [A]:
- The velocity controller stands in for `reef_control` + rosflight, and REEF's
  `desired_state` velocity requests map naturally onto `Twist` commands. Porting
  `reef_control` against a rosflight-free actuator path is a later decision.
- Estimator inputs: bridge the IMU (`sensor_msgs/Imu`), the altimeter (→ `Range`),
  and odometry (→ mocap-like PoseStamped/Twist after ENU→NED and body-level
  conversion). Compare `xyz_estimate` against the bridged ground truth.

---

## 7. Open questions / decisions pending

1. Which estimator behavior is the baseline: master `e4179f48` or simulation `95987b51`?
2. Should `rosflight_msgs` be a dependency (upstream ROS 2 `rosflight_ros_pkgs`)
   or compiled out for the Gazebo path?
3. Which frame should be used at the ROS 2 boundary: keep REEF's NED/body-level internally with
   converters at the Gazebo bridge (recommended), or switch to REP-103?
4. Should the X3 model be vendored into `sim/models/` for offline, deterministic runs?

## 8. Proposed next steps

1. Bring up `multicopter_velocity_control` (X3 only) with the IMU/altimeter added,
   and bridge `/clock`, IMU, odometry, and twist command. Verify hover and step commands.
2. Port `reef_msgs` to `src/reef_msgs` (ament_cmake, rosidl).
3. Port `reef_estimator` to `src/reef_estimator` (rclcpp, parameters, QoS
   transient_local for the formerly latched topics), preserving the filter math
   unchanged.
4. Add frame-conversion adapters and a closed-loop estimator-vs-truth check.
