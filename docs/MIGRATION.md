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
| Mac-host commands | `docker exec ros2_novnc_container …` and the browser URL (README) | [A] written for the Mac host; not executed, since this work ran inside the container with no host access |

### Environment hazards found

1. **[V] Agent shells inherit an unrelated overlay.** `~/.bashrc` sources
   `/root/ros2_ws/install/setup.bash`, which contains tutorial packages. The agent's
   shell already had those on `AMENT_PREFIX_PATH`, and it had `DISPLAY=:1`, not
   `:99`. All `scripts/*.sh` therefore re-exec under `env -i` and source only
   `/opt/ros/jazzy` (plus this repo's own `install/` once it exists).
   See `scripts/env.sh`.
2. **[V] Jazzy defaults `ROS_AUTOMATIC_DISCOVERY_RANGE` to `SUBNET` only if
   it is unset.** The `ros_environment` hook is
   `set-if-unset;ROS_AUTOMATIC_DISCOVERY_RANGE;SUBNET`. Sourcing
   `/opt/ros/jazzy/setup.bash` with the variable unset gives `SUBNET`, and with
   `LOCALHOST` preset it keeps `LOCALHOST`. (The first version of this document
   wrongly said the hook always sets it.) After the scripts' `env -i` re-exec the
   variable is always unset, so the scripts set it explicitly: `LOCALHOST` by default,
   or `REEF_DISCOVERY_RANGE` if given. A caller's own value is not passed through,
   because any sourced shell carries `SUBNET` from this same default and that
   value does not show intent. `LOCALHOST` limits discovery reach; it is **not**
   test isolation (see §5).
3. **[V] The parent workspace crawls into this repo.** `colcon list` run from
   `/root/ros2_ws` discovers packages under `reef_ros2/`. With
   `reference/COLCON_IGNORE` removed, it picked up the ROS 1
   `reef_estimator` (`ros.catkin`). The marker is essential. Packages later
   added under `reef_ros2/src/` **will** also be built by a `colcon build` in
   `/root/ros2_ws`. Always build from `/root/ros2_ws/reef_ros2`.
4. **[V] `gz sim` puts its server and GUI in separate process groups.**
   Killing the launch's process group leaves them running.
   `scripts/check_clock_demo.sh` tears down by session ID instead, and only the
   session it created. Headless `gz sim -s` runs the server inside the
   `gz sim -r -s …` process; the GUI mode forks `gz sim server` and `gz sim gui`.
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

### Estimator differences: master `e4179f48` vs sim pin `95987b51` [V]

The sim bundle's estimator is the `simulation` branch. From
`git diff e4179f48 95987b51` (9 files, +24/−80), plus reading the
surrounding code in both commits, the differences go well beyond measurement
rejection. A single rejection toggle does **not** reproduce the simulation branch.
This task only records the differences. The choice of baseline is still open (§7).

| Area | master `e4179f48` | sim pin `95987b51` |
|---|---|---|
| Z process model `F` | `[[1, dt, 0], [0, 1, −dt], [0, 0, 1]]`: the bias enters velocity only, with a negative sign | `[[1, dt, dt²/2], [0, 1, +dt], [0, 0, 1]]`: the bias enters velocity with a positive sign, and also enters position |
| Z model timing | `ZEstimator::updateLinearModel()` rebuilds `F`, `B`, and `Q = Q0·dt` at init and **on every IMU propagation**, using `dt` measured from IMU stamps. The constructor's `dt = 0.005` is overwritten by the `estimator_dt` param (default 0.002) before first use | `updateLinearModel()` and `Q0` removed. `F`/`B` are built **once** in the constructor with hard-coded `dt = 0.002`. The runtime `dt` from `estimator_dt` and from IMU stamps is still assigned but never rebuilds the Z matrices. `Q = z_Q · estimator_dt` is applied once at init |
| Gravity initialization | Averages the initial accelerometer samples, then **overrides** the magnitude with `9.81` (and logs it) | Uses the **measured** average accelerometer magnitude |
| XY update flag | With `enable_partial_update: true` (the shipped default), `newRgbdMeasurement` is **not cleared** after `partialUpdate()`. After the first XY measurement (RGB-D or mocap XY), the stored measurement is re-applied on every IMU step. Only the full-update branch clears it | Cleared after both partial and full updates |
| χ² (Mahalanobis) gating | Applied to RGB-D velocity, sonar, mocap XY, and mocap Z | **Disabled** (commented out) for all four. The sonar `range <= max_range` check remains. The `mahalanobis_d_*` params are still read but no longer used for these gates |
| `enable_measurements` (`basic_params.yaml`) | `false`. This flag gates only `rgbdTwistCallback`, so RGB-D velocity updates are dropped | `true` |
| `z_P0`, `z_P0_flying` bias variance | 0.01 | 0.09 |
| `z_Q` (accel noise, bias random walk) | `[0.03, 0.0001]` | `[0.03, 0.001]` |
| `z_R_flying` | 0.00016 | 0.0016 |
| `z_x0`, `z_beta`, `z_R0` | same numeric values (only formatting changed) | same |
| Build | `add_dependencies(... ${catkin_EXPORTED_TARGETS})` | adds `reef_msgs_generate_messages_cpp reef_msgs` |
| Files | has `LICENSE` (MIT, 2020) and `params/wren_camera.yaml` | both absent |

Unchanged between the two: `estimator.cpp` (KF propagate/update/partialUpdate),
`xy_estimator.*`, `sensor_manager.*`, `xyz_estimator.h`, `xy_est_params.yaml`,
and all launch files.

---

## 3. Upstream estimator facts [V]

- Single C++ executable `reef_estimator` (catkin; C++ with Eigen). Classes:
  `Estimator` (KF with partial update) → `XYEstimator` (EKF), `ZEstimator` →
  `XYZEstimator` (data flow, outlier rejection) → `SensorManager` (callbacks).
- `include/xy_estimator.h:9` includes
  `../../reef_msgs/include/reef_msgs/dynamics.h` by **relative path**, which
  assumes the bundle's directory layout. (`xyz_estimator.h:23` uses the normal
  `<reef_msgs/dynamics.h>`.) The port must use a proper exported include.
- ROSflight references (master): the core estimator path (`src/`, `include/`)
  uses `rosflight_msgs` only for `RCRaw`, in `SensorManager::rcRawCallback`
  (the mocap-override switch). Beyond that path, `package.xml` depends on both
  `rosflight` and `rosflight_msgs`, `CMakeLists.txt` finds `rosflight_msgs` and
  exports `rosflight`/`rosflight_msgs` in `CATKIN_DEPENDS`, and the recording
  launch files `estimator_record.launch`, `record_raw.launch`, and
  `record_stable_raw.launch` start `rosflight_io`. Pinned `reef_msgs` has no
  ROSflight dependency.
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
| **rosflight_msgs** (core estimator path uses `RCRaw` only, for the mocap switch; upstream build metadata and recording launches reference more, see §3) | option (a): make it optional/compile-out; option (b): `rosflight_msgs` 2.0.0 from rosflight/rosflight_ros_pkgs `main` (`5ef20134`) | [V] not in Jazzy apt; [V] upstream ROS 2 package exists; [A] `RCRaw` still defined there |

Minimum estimation stack = **`reef_msgs` (ROS 2) + `reef_estimator` (ROS 2)**.
Leave the `RCRaw` mocap switch behind a build option. [A] The recording launch
files are not part of the minimum port. They would need `rosflight_io` or a
replacement.

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
| Vehicle + flight control | rosflight SIL firmware | Harmonic `MulticopterMotorModel` + `MulticopterVelocityControl` systems [V] present in example; [V] the controller's feedback is simulator ground truth (§6), so it suits open-loop estimator evaluation, not REEF-in-the-loop control |
| Ground truth | `multirotor/truth/NED` | `OdometryPublisher` → `/model/x3/odometry` (ENU) bridged + converted to NED [V] plugin in example; [A] conversion node needed |
| IMU | rosflight IMU | Harmonic `Imu` system + `<sensor type="imu">`. **Must be added: the X3 model has no sensors** [V] |
| Altimeter (sonar) | sim sonar | Harmonic `Altimeter`/`AirPressure` sensor or a downward range sensor; [A] a thin adapter to `sensor_msgs/Range` |
| Mocap | truth → `position_to_velocity` | same pattern from bridged odometry [A] |
| Setpoints | setpoint_generator / dubins_path (rospy) | out of scope for minimum; port later |

---

## 5. Sim-time bridge demonstration [V]

Files: `sim/worlds/clock_demo.sdf`, `sim/launch/clock_demo.launch.py`
(args `world`, `headless`, `clock_topic`), `scripts/run_clock_demo.sh`,
`scripts/check_clock_demo.sh`, `scripts/clock_check.py`, and
`scripts/regress_clock_check.sh`.

`clock_check.py` observes a clock topic for a wall-clock window. It asserts
that messages arrive, that sim time increases monotonically, and that a node with
`use_sim_time=True` sees `get_clock().now()` advance. The node's `/clock` is
remapped to `--topic`, including its time source.

### False pass in the first version (fixed in `3d6e779`)

Review found, and this project reproduced, that the checker at `243180a` could
report success when its own demo had failed. With an unrelated headless demo
publishing `/clock` in the default domain, the old checker run with
`REEF_DISPLAY=:197` (no such display) printed `PASS`, **exit 0**. Its own launch
had already died at the display check. The checker at `3d6e779` run in the
same situation gives `FAIL owned demo launch exited early`, **exit 2**. The old
checker observed a shared `/clock`, used the caller's ROS domain, dropped
`GZ_PARTITION` in the clean re-exec, and never checked that its launch was alive.

### How the checker now owns what it observes

1. **Gazebo layer:** a per-run `GZ_PARTITION` (default
   `reef_clock_check_<token>`) is exported before launch, so the owned bridge
   can hear only the owned gz server. `env.sh` now passes `GZ_PARTITION`
   through the clean re-exec for interactive use.
2. **ROS layer:** the bridge publishes on a per-run topic
   `/reef_clock_check_<token>/clock` (launch arg `clock_topic`). The observer
   requires exactly one publisher on it, named `clock_bridge`. A per-run
   `ROS_DOMAIN_ID` (random 1–101, or `REEF_TEST_ROS_DOMAIN_ID`) only reduces
   cross-talk. It is **not** relied on for uniqueness, and neither is `LOCALHOST`.
3. **Liveness:** the checker polls every 0.2 s. It fails with exit 2 as soon as
   the owned launch exits or a required process that was seen disappears.
   Required processes are the gz server, `parameter_bridge`, and `gz sim gui`
   unless headless. When each one first appears, the checker verifies that its
   `/proc/<pid>/environ` carries the test's `ROS_DOMAIN_ID` and `GZ_PARTITION`.

Layers 1 and 2 stop foreign clock data from reaching the observer. Layer 3
makes an owned failure end the check within a second or two, instead of
waiting for the observer's 60 s startup timeout. The reproduced false pass is
stopped by layer 3 directly, and by layers 1–2 independently (cases 4c, 4e, 4f
below).

Cleanup: the observer is a tracked child that is terminated (TERM, then KILL
after 5 s) and reaped. From the demo launch until both the demo's session id and
the observer's pid are registered, INT/TERM are only recorded and acted on
immediately afterwards, so cleanup always knows what to stop. As a backstop, if
the session id is still unknown, cleanup recovers it from the launch pid (only
that pid or its direct child is accepted as session leader). The demo runs in a session created by the checker, and
teardown uses `pkill -s <sid>` only after checking that the sid is not the
checker's own session. Traps cover normal exit, failure, timeout, SIGINT, and
SIGTERM. Exit statuses: 0 pass, 1 clock check failed, 2 owned demo failed or
bad input, 124 timeout, 130 SIGINT, 143 SIGTERM.

### Startup interruption leak (found in the second review, fixed in `81ae28a`)

At `3d6e779`, a signal that arrived after the demo was launched but before its
session id was registered ran cleanup with an empty sid. The checker exited 143
and left the whole demo running. Reproduced here 3/3 before the fix: exit 143
with 4 survivors each time (`ros2 launch`, the gz wrapper, `gz sim`, and
`parameter_bridge`). After the fix, the same reproduction gave exit 143 with 0
survivors, 3/3. Survivors were identified by each attempt's unique
`GZ_PARTITION` in `/proc/<pid>/environ`. The earlier interruption cases had
all waited until the observer was active, so they could not hit this window.

### Regression results (`scripts/regress_clock_check.sh`, 2026-09-29, run on code identical to `81ae28a`)

| # | Case | Expected | Actual | Time | Evidence |
|---|---|---|---|---|---|
| 1 | Headless success | 0 | 0 | 11 s | PASS |
| 2 | GUI success on existing `:99` | 0 | 0 | 10 s | PASS; server, GUI, and bridge env verified |
| 3 | No clock (fresh topic, nothing running) | 1 | 1 | 3 s | `FAIL no message … within 3s` |
| 4a | Sanity: unrelated sim is publishing `/clock` in domain D | 0 | 0 | 5 s | PASS |
| 4e | ROS layer: fresh topic in domain D while the unrelated `/clock` is live | 1 | 1 | 6 s | no message |
| 4f | Gazebo layer: lone bridge in a *different* partition | 1 | 1 | 8 s | no message |
| 4g | Control: lone bridge in the *same* partition | 0 | 0 | 3 s | PASS |
| 4b | Unrelated sim + checker with `REEF_DISPLAY=:197`, same domain D | 2 | 2 | 2 s | `FAIL owned demo launch exited early` |
| 4c | As 4b, and forced onto the unrelated sim's domain **and** partition | 2 | 2 | 1 s | `FAIL owned demo launch exited early` |
| 4d | Unrelated sim + valid headless checker in domain D | 0 | 0 | 11 s | PASS (coexists) |
| 5 | SIGTERM to the checker during observation | 143 | 143 | 4 s | observer gone; demo session 4 procs → 0 |
| 6 | SIGINT to the checker's process group (Ctrl-C) during observation | 130 | 130 | 5 s | observer gone; demo session 4 procs → 0 |
| 8 | Timeout: observer SIGSTOPped, deadline 20 s | 124 | 124 | 26 s | `FAIL timed out after 20s`; stopped observer killed |
| 9 | Owned bridge killed mid-run | 2 | 2 | 5 s | `FAIL required process 'parameter_bridge' exited` |
| 10 | SIGTERM before session registration (`REEF_TEST_REGISTER_DELAY=3` holds the window open; signalled once the demo session existed) | 143 | 143 | 4 s | 0 tagged survivors |
| 11 | SIGINT to the checker's group before registration (same hook) | 130 | 130 | 1 s | 0 tagged survivors |
| 12 | Unhooked SIGTERM sweep at 0, 0.1, 0.2, 0.3, 0.4, 0.6, 0.8, 1.0, 1.4, 1.8 s after start (≤0.4 s landed before registration, ≥0.6 s after) | 143 ×10 | 143 ×10 | 0–12 s | 0 tagged survivors in every run |
| 7 | Display services (Xvfb, x11vnc, websockify PIDs) | unchanged | unchanged | n/a | 2346 2358 2359 2534 before and after |

The suite checks ownership before every signal. The observer must be the
checker's child, and the demo session leader must be the checker's child or
grandchild. The suite's own unrelated sim is stopped the same way. After each
suite run, no `gz sim`, `parameter_bridge`, `clock_check`, or `ros2 launch`
processes remained. Times and clock rates are observations, not thresholds.
The slowest sweep case (1.4 s, 12 s total) was `gz sim` ignoring SIGINT while
still initializing: `ros2 launch` waited its standard 5 s before escalating to
SIGTERM. Teardown stayed bounded and complete. The launch-pid recovery backstop
in cleanup has not been exercised directly, because signal deferral keeps it from
being needed on every path that was tested.

Earlier observations with the first version (still representative of the
demo itself): headless RTF about 0.98 at about 960 Hz; GUI RTF 0.89–0.95. The GUI
window ("Gazebo Sim", `gz-sim-gui`) was confirmed mapped on `:99` via
`xwininfo`. No screenshot tool is installed, so no image was captured.
**On the Mac host**, view it at http://localhost:8080/vnc.html.

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
    velocity + yaw rate). Enable topic `/X3/enable` (`gz.msgs.Boolean`). The
    controller starts **enabled** (`controllerActive{true}` in gz-sim 8.15.0
    `MulticopterVelocityControl.hh`), and `false` disables it. Gains are in the SDF.
  - **The controller's feedback is simulator ground truth.** On every update it
    reads the X3 base link's `WorldPose`, `WorldLinearVelocity`, and
    `AngularVelocity` components straight from the physics state. See
    `getFrameData()` in `src/systems/multicopter_control/Common.cc` and
    its call in `MulticopterVelocityControl.cc` (tag `gz-sim8_8.15.0`). It
    then computes rotor speeds from those values. This is separate from, and does
    not use, the `OdometryPublisher` output. Optional velocity noise can be set in
    the SDF. In this world only the X4 controller sets it; X3 gets noise-free truth.
- Ground-truth dependencies:
  - `gz-sim-odometry-publisher-system` (3D) publishes `gz.msgs.Odometry` on
    `/model/x3/odometry` (ENU world frame; [A] child frame FLU body).
  - Pose on `/model/x3/pose` per the file header comment [A] (verify with `gz topic -l`).
- Fuel model download: [V] `gz fuel download` of X3 succeeded (≈93 MB),
  into a throwaway `HOME` during the first session; it has not been re-run since.
  [V] The X3 `model.sdf` contains **no `<sensor>` elements**. The IMU and
  altimeter must be added (the `Imu` and `Altimeter` systems, plus sensors on
  `X3/base_link`). [A] The first run will fetch the models from Fuel into
  `~/.gz/fuel`, so vendor the model or pre-fetch it for offline repeatability.

How it maps onto REEF [A]:
- **Validation limit:** as shipped, the vehicle is flown by a controller that
  reads ground truth. REEF can run *alongside* it and be scored against truth
  (open-loop estimation), but this setup does **not** show REEF-in-the-loop
  feedback control. Closing the loop on `xyz_estimate` needs a different
  controller path, either ported `reef_control` or a custom Gazebo system or
  bridge that takes attitude/thrust or rotor commands.
- The velocity controller's `Twist` input is a natural stand-in for REEF's
  `desired_state` velocity requests. Porting `reef_control` against a
  rosflight-free actuator path is a later decision.
- Estimator inputs: bridge the IMU (`sensor_msgs/Imu`), the altimeter (→ `Range`),
  and odometry (→ mocap-like PoseStamped/Twist after ENU→NED and body-level
  conversion). Compare `xyz_estimate` against the bridged ground truth.

---

## 7. Open questions / decisions pending

1. Which estimator behavior is the baseline: master `e4179f48` or simulation `95987b51`?
   The differences cover the Z model, timing, initialization, noise, flags, and
   gating (see the table in §2), not only rejection. Not decided in this phase.
2. Should `rosflight_msgs` be a dependency (upstream ROS 2 `rosflight_ros_pkgs`)
   or compiled out for the Gazebo path?
3. Which frame should be used at the ROS 2 boundary: keep REEF's NED/body-level internally with
   converters at the Gazebo bridge (recommended), or switch to REP-103?
4. Should the X3 model be vendored into `sim/models/` for offline, deterministic runs?
5. Which controller path should close the loop on REEF estimates (ported `reef_control`,
   or a custom Gazebo system), given that the stock velocity controller uses truth?

## 8. Proposed next steps

1. Bring up `multicopter_velocity_control` (X3 only) with the IMU/altimeter added,
   and bridge `/clock`, IMU, odometry, and twist command. Verify hover and step commands.
2. Port `reef_msgs` to `src/reef_msgs` (ament_cmake, rosidl).
3. Port `reef_estimator` to `src/reef_estimator` (rclcpp, parameters, QoS
   transient_local for the formerly latched topics), preserving the filter math
   unchanged.
4. Add frame-conversion adapters and an estimator-vs-truth comparison, with the
   vehicle still flown by the truth-fed controller (see §6 validation limit).
