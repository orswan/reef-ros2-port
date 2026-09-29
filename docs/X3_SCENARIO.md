# X3 scenario: simulated data for the REEF estimator port

A bounded, repeatable Gazebo Harmonic quadrotor flight that produces ROS 2
ground truth, IMU, and idealized downward-range data, records it with
rosbag2, and checks and plots the recording. It is the data source for porting
and testing the REEF estimator. It is **not** REEF-in-the-loop flight.

> **Two limits to keep in mind**
> 1. The vehicle is flown by Gazebo's `MulticopterVelocityControl` system,
>    which uses **simulation ground truth** internally. It reads the base link's
>    world pose, world linear velocity, and body angular velocity from Gazebo
>    components (gz-sim 8.15.0 `src/systems/multicopter_control/Common.cc`,
>    `getFrameData`). It does not use the IMU, the range, or any estimate.
>    REEF feedback control is a later deliverable.
> 2. The sensors are synthetic, and the range sensor is idealized. Agreement
>    between these measurements and truth checks the pipeline, not any hardware.

Package: `src/reef_sim` (ament_python). Commands are **container terminal**
commands, run from `/root/ros2_ws/reef_ros2` (see the README for Mac-terminal
equivalents).

```bash
scripts/setup_assets.py            # once: download and verify the pinned X3 model (~22 MB)
scripts/run_x3_scenario.sh         # headless flight + recording + analysis (~80 s)
scripts/run_x3_scenario.sh --gui   # same, with Gazebo shown on the browser desktop
scripts/regress_x3_scenario.sh     # regression suite (~6-7 min)
```

## 1. Vehicle, world, and assets

| Item | Source | Changes |
|---|---|---|
| World `worlds/x3_flight.sdf` | gz-sim 8.15.0 `examples/worlds/multicopter_velocity_control.sdf` (Apache-2.0; installed sha256 `836751d4…`) | X4 removed; rendering Sensors system removed (no GPU sensors, so it runs headless); Imu system added; physics step 4 ms → 2 ms; X3 loaded from local `model://reef_x3`; OdometryPublisher given explicit frames, rate, and topics. The motor-model and velocity-controller parameters are copied unchanged |
| Vehicle `models/reef_x3` | Fuel **X3 UAV v4** `model.sdf` (sha256 `b76c0852…`) by Open Robotics (Carlos Agüero, Cole Biesemeyer), **CC BY 4.0** | Renamed; noise-free IMU added to `X3/base_link`. Meshes still come from the upstream asset (`model://x3/...`) |
| Meshes and textures | Fuel X3 UAV v4 archive, sha256 `c995a307…` (22,158,656 bytes, 26 files) | none |

The asset manifest `src/reef_sim/assets/x3_uav_v4.json` records the source
URL, Fuel version, license and its source, authors, archive SHA-256, and the
SHA-256 of every file. Fuel's tip version at capture was 6; the example world
and this project pin **4**.

- **Setup:** `scripts/setup_assets.py` downloads the pinned archive, checks
  its SHA-256, extracts it, checks every file, and moves it atomically to
  **`assets/models/x3/`**, adding `ATTRIBUTION.txt` and `REEF_ASSET.json`.
  `--verify` checks only and never uses the network.
- **Persistence:** `assets/` is inside the bind-mounted project, so it is kept
  on the Mac across container replacement. It is ignored by Git; only the
  manifest is committed. `REEF_ASSETS_DIR` overrides the location.
- **Offline guarantee:** every run verifies the assets first. It then gives
  Gazebo an **empty per-run Fuel cache** (`GZ_FUEL_CACHE_PATH`) and an
  unreachable HTTP(S) proxy, and it **fails if any file appeared in that cache**.
  The world and model reference only `model://` URIs on the local resource path.
  Nothing uses the original container's `~/.gz` cache.

## 2. Interfaces

All stamps are **simulation time**. The recorder uses `--use-sim-time`, so bag
receive times are sim time too.

| Topic | Type | Category | Publisher | Frame | Rate (configured) | Contents and units |
|---|---|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | sim time | bridge (Gazebo) | — | every physics step (500 Hz) | s |
| `/x3/truth/odom` | `nav_msgs/Odometry` | **TRUTH** | bridge (OdometryPublisher) | `world` → `x3/base_link` | 100 Hz | pose: exact world pose of the model (m, quaternion). twist: body frame (FLU), m/s and rad/s, **finite difference of pose with a 10-sample moving average** (about 10 ms lag at a 2 ms step). Covariances unset |
| `/x3/imu` | `sensor_msgs/Imu` | MEASUREMENT | `imu_noise` | `x3/base_link` (FLU) | 250 Hz | specific force (m/s²) and angular rate (rad/s) with seeded white noise. Covariance diagonals = σ². **Orientation not provided**: `orientation_covariance[0] = -1`, quaternion all zeros |
| `/x3/range` | `sensor_msgs/Range` | MEASUREMENT (**idealized, derived from truth**) | `range_sensor` | `x3/range_link` | 20 Hz | slant range along body −Z (m); REP 117 ±inf outside limits; see §4 |
| `/x3/cmd_vel` | `geometry_msgs/Twist` | COMMAND | `scenario_runner` | body FLU | 20 Hz | velocity command to the truth-fed controller (m/s, rad/s) |
| `/x3/scenario/phase` | `std_msgs/String` | label | `scenario_runner` | — | once per phase change (transient local) | phase name |
| `/x3/sim/imu_noise_free` | `sensor_msgs/Imu` | simulator-internal (**not recorded**) | bridge (Gazebo IMU) | `x3/base_link` | 250 Hz | input to `imu_noise` |

Gazebo-side topics are namespaced `/reef/x3/...` and bridged by
`config/bridge.yaml`. `/x3/cmd_vel` goes to Gazebo `/X3/gazebo/command/twist`.

Frames (not published on `/tf`):
- `world`: Gazebo world, REP 103 ENU convention (x east, y north, z up). Ground
  plane at z = 0.
- `x3/base_link`: vehicle body, FLU (x forward, y left, z up). At the base link
  origin, which sits 0.055 m above the ground when landed.
- `x3/range_link`: range sensor origin at (0, 0, −0.055) m in `x3/base_link`
  (bottom face of the body), beam along its −Z axis, axes aligned with
  `x3/base_link`.

### IMU convention (verified from recordings)

The IMU reports **specific force** f = Rᵀ(a − g) in the body frame, where a is
the body's acceleration in the world and g = (0, 0, −9.81) m/s² (ENU). At
rest a = 0, so f = (0, 0, **+9.81**) m/s² in FLU. Recorded settle-phase mean:
(0.00, 0.00, 9.80) m/s², σ ≈ 0.02. In steady hover |f| ≈ g as well (thrust per
unit mass). Gyro: body angular rate, FLU, rad/s; it matches the truth body rate
to about 5 mrad/s RMS (≈ the noise). Noise: white Gaussian, no bias,
σ_a = 0.02 m/s², σ_ω = 0.002 rad/s.

Why the noise is added in ROS: Gazebo's sensor noise draws from gz-math's
**global** random generator. Other systems (for example OdometryPublisher,
which draws even with zero noise) use it concurrently in gz-sim 8's parallel
`PostUpdate`, so the sequence differs between runs even with `gz sim --seed`.
This was measured: two runs with the same seed gave differences of √2·σ, that
is, independent noise. `imu_noise` instead keys each sample's generator by
`(seed, stamp)`. Two runs then agree to 8·10⁻⁵ m/s² RMS; the residue comes
from the noise-free signal, not the noise.

## 3. Attitude input for future REEF integration

REEF's estimator needs attitude (roll and pitch, to build `C_NED_to_body`) in
addition to IMU data. On hardware this came from ROSflight's onboard attitude
estimator (`attitude` topic). **This project has no attitude estimator.** The
only attitude available is the **idealized** truth orientation in
`/x3/truth/odom`. A REEF adapter that uses it must label it as truth. Options
for later: an attitude estimate from the IMU (for example a complementary
filter), or rosflight's SIL. The IMU's orientation field is deliberately not
populated, so truth cannot leak through a measurement topic.

## 4. Idealized downward range

`range_sensor` computes the range from the truth pose. It is not a Gazebo
sensor, and it adds no rendering dependency.

- **Geometry:** sensor origin o = p + R·r_s with r_s = (0, 0, −0.055) m;
  beam d = R·(0, 0, −1). Range = (0 − o_z)/d_z, the exact slant range to
  the plane z = 0. **Tilt is included:** at tilt θ the slant range is
  height / cos θ, not the vertical altitude.
- **Ground assumption:** flat, infinite, at world z = 0; no obstacles.
- **Beam:** a single ray (`field_of_view = 0`): no cone, multipath, latency,
  temperature effects, or dropouts. `radiation_type = ULTRASOUND` is nominal
  (it stands in for REEF's MaxBotix MB1242 sonar).
- **Limits** (MB1242-like): `min_range` 0.20 m, `max_range` 7.65 m. REP 117:
  **−inf** when the true range is below `min_range` (on the ground), **+inf**
  above `max_range` or if the beam points at or above the horizon.
- **Noise:** Gaussian σ = 0.01 m on in-range values, keyed by
  `(seed, stamp)`, then clamped to the limits.
- **Timing:** decimated from truth to a fixed 20 Hz sim-time grid, stamped
  with the truth sample's time.

In this scenario the maximum tilt was about 5.5°, where the slant range exceeds
the vertical sensor height by up to about 9 mm. The analysis plots both.

## 5. Conversions REEF integration will need

REEF uses NED world and FRD body frames, plus a "body-level" frame (NED rotated
by yaw only). These are data transformations, not frame renames:

| Quantity | From (this project) | To (REEF) |
|---|---|---|
| IMU specific force and angular rate | FLU `(x, y, z)` | FRD `(x, −y, −z)`. Specific force at rest becomes (0, 0, **−g**). REEF computes `accel_z_NED + g` (≈ 0 at rest), which matches this sign |
| Position | ENU `(x, y, z)` | NED `(y, x, −z)` |
| Attitude | R_ENU←FLU (truth quaternion) | R_NED←FRD = T · R_ENU←FLU · B, with T = [[0,1,0],[1,0,0],[0,0,−1]] and B = diag(1, −1, −1) |
| Velocity | odom twist: body FLU | world NED: v_NED = T · R_ENU←FLU · v_FLU. Body-level: v_level = R_z(ψ)ᵀ · v_NED |
| Range | positive slant range, ±inf out of limits | REEF's sonar path uses `z = −range` as NED altitude **without tilt compensation**, and accepts `range <= max_range`. It would accept **−inf** (below `min_range`), so the adapter must drop non-finite readings. Whether to tilt-compensate (height = range · cos θ) is an adapter decision to make and document |
| Mocap-like inputs | derived from truth | REEF's mocap mode uses `PoseStamped` (NED) and `TwistWithCovarianceStamped` (body-level). Any such stream derived from truth must be labelled idealized |

## 6. Scenario

`config/x3_scenario.yaml` (copied into every recording):

| Phase | Duration (s) | Command (body FLU, m/s) | Expected displacement |
|---|---|---|---|
| settle | 5 | 0 | on the ground (stationary IMU segment) |
| ascend / hover_high | 5 / 4 | vz = +0.4 / 0 | +2.0 m up |
| forward / hover_fwd | 5 / 3 | vx = +0.4 / 0 | +2.0 m x |
| left / hover_left | 5 / 3 | vy = +0.4 / 0 | +2.0 m y |
| back / hover_back | 5 / 3 | vx = vy = −0.4 / 0 | back to the start column |
| descend / hover_low | 4 / 4 | vz = −0.3 / 0 | −1.2 m (ends ~0.85 m up) |

Total 46 s of sim time. The controller settles to about 0.37 m/s for a 0.4 m/s
horizontal command (a steady-state error of the stock controller), so
horizontal displacements come out at about 0.92 of the commanded value.
Seeds: `imu_noise.seed` 7, `range_sensor.seed` 42.

## 7. Running, recording, and analysis

`scripts/run_x3_scenario.sh [--gui]` does the following:
1. Builds `reef_sim` (`colcon build --base-paths src --symlink-install`).
2. Verifies the assets.
3. Creates `recordings/x3_<time>_<id>/` (ignored by Git) and writes `manifest.yaml`.
4. Launches the scenario in an owned session with per-run `GZ_PARTITION` and
   ROS domain, and monitors required processes. A process's first appearance
   is checked against the run's domain and partition.
5. Waits for the scenario to finish and the bag to finalize.
6. Checks that nothing was fetched.
7. Runs the analysis.

With `--gui`, the Gazebo server still runs headless (`-s`) and the GUI is a
separate viewer (`gz sim -g`). gz-sim 8's combined mode makes the server wait
for a world-path handshake from the GUI (`wait_gui`), and a missed handshake
left the server stuck before loading the world in 1 of 4 early GUI runs. The
viewer is optional: if it never starts, the run warns, and the recording is
unaffected. In the GUI the vehicle looks small at the default camera distance;
zoom with the scroll wheel.

Run directory:

| File | Contents |
|---|---|
| `manifest.yaml` | scenario, git commit, branch and uncommitted changes, container/image identity, gz/ros-gz/rosbag2 versions, world/model/asset hashes and license, seeds, all parameters, isolation settings, command, outcome (`result`, `exit_status`, `fuel_files_fetched`, bag size) |
| `x3_scenario.yaml` | the parameters used |
| `bag/` | rosbag2, MCAP: `/clock`, `/x3/truth/odom`, `/x3/imu`, `/x3/range`, `/x3/cmd_vel`, `/x3/scenario/phase` (about 9.5 MB) |
| `scenario_result.json` | status and sim-time boundaries of every phase |
| `launch.log`, `analysis.log` | process output |
| `analysis/` | `truth_altitude_velocity.png`, `range_vs_geometry.png`, `imu.png`, `validation.json` |

To re-analyze a run: `ros2 run reef_sim analyze_x3_bag recordings/<run>` (after
sourcing `install/setup.bash`, or through `scripts/env.sh`).

Exit status: 0 pass; 1 analysis check failed (including a missing stream);
2 setup failed or the owned simulation failed (including assets missing or
modified, a required process exited, the bag not finalized, a Fuel fetch);
3 scenario runner failed (startup timeout, sim-time stall, more than one
publisher on a stream); 124 timeout; 130 SIGINT; 143 SIGTERM.

### Acceptance criteria and why

The data is synthetic and the controller is truth-fed, so the checks ask
whether the pipeline works as documented, not how good any sensor is. No exact
message rate or real-time factor is required.

| Check | Criterion | Rationale |
|---|---|---|
| Streams | each required topic present with ≥ 50 % of the nominal count (phase labels: one per phase) | catches missing or badly broken streams without depending on the RTF |
| Timestamps | finite, monotonic, spanning ≥ 90 % of the scenario, p99 \|bag − header\| < 0.5 s | usable, consistent sim-time stamps |
| Finite values, frames | no NaN; frames exactly as documented; range values in limits or ±inf | interface contract |
| Response | for each commanded phase (with the hover after it): displacement / (v·T) in [0.7, 1.3], cross-axis drift < 0.3 m; settle keeps z < 0.1 m | the vehicle does what it was told, within the stock controller's lag and steady-state error (observed ratios 0.91–1.00) |
| IMU at rest | \|mean − (0, 0, 9.807)\| < 0.2 m/s², σ < 0.1 per axis, \|mean gyro\| < 0.02 rad/s | specific-force sign, frame, and noise level |
| IMU in hover | \|f\| ∈ [9.3, 10.3] m/s²; bounds \|f\| < 30, \|ω\| < 5; gyro vs truth rate RMS < 0.05 rad/s | physically plausible, same body frame as truth |
| Range | residual vs geometry recomputed independently from truth: \|mean\| < 3σ/√n, std within 30 % of σ; below-min → −inf | the idealized model is implemented as specified |
| Truth | odom twist vs d(pose)/dt RMS < 0.1 m/s | truth pose and twist agree |

## 8. Replay

The bag contains its own `/clock`. Replay it and give consumers
`use_sim_time:=true`. Use a separate ROS domain, and exclude the command topic
so a replay can never drive a live simulation (**container terminal**):

```bash
source scripts/env.sh && reef_setup_env        # or open a new container terminal
export ROS_DOMAIN_ID=42                         # any domain not used by a live run
ros2 bag play recordings/<run>/bag --exclude-topics /x3/cmd_vel
# in another terminal (same domain):
ros2 topic echo /x3/range --once
ros2 run <pkg> <node> --ros-args -p use_sim_time:=true
```

Do not also pass `--clock` to `ros2 bag play`, because that would create a
second clock source. Regression case 10 replays a bag and checks that a
`use_sim_time` node sees sim time advance.

## 9. Validation performed (2026-09-29)

This ran in `ros2_novnc_container`, whose Gazebo/ROS package versions match
the dev image's recipe. It has **not yet run in `reef_ros2_dev`**; the README
gives the Mac-terminal commands.

| Check | Result |
|---|---|
| `scripts/regress_x3_scenario.sh` (final run) | 13/13 PASS: offline asset verify; headless run (analysis passed, 3 plots, 0 Fuel files); GUI run (window mapped, snapshot shows the X3); same-seed IMU noise reproducible (RMS diff 8·10⁻⁵ m/s², 2·10⁻¹⁵ rad/s); missing range stream → 1, and the analysis names it; missing assets → 2 before any simulation; gz server killed → 2; SIGTERM → 143; SIGINT → 130, with 0 tagged processes left in each; replay drives sim time; display services unchanged |
| GUI flight repeated after the server/GUI split | 3/3 PASS (90–100 s each) |
| Earlier suites after this change | clock checks 28/28, desktop tests 12/12 |
| shellcheck 0.9.0 / hadolint 2.12.0 | 0 findings |

## 10. Known limitations

- The controller is truth-fed; there is no REEF-in-the-loop flight.
- The range sensor is idealized: flat ground, a single ray, no sonar artefacts.
- The IMU has white noise only: no bias, bias drift, scale factor,
  misalignment, vibration, or latency.
- The truth twist is a smoothed finite difference (about 10 ms lag), not the
  physics engine's velocity.
- `/clock` is recorded at 500 Hz, which makes up most of the bag's messages.
- Gazebo writes its console logs to `$HOME/.gz` (`sim/log/<time>/`,
  `auto_default.log`) in whichever container runs it. In the dev container that
  is the `reef_ros2_gz` volume. `GZ_HOMEDIR` is not honoured by this Gazebo
  version (tested), so these logs are not in the run directory.
- The vehicle starts on the ground, and the rotors engage when the first
  command arrives (timing depends on ROS delivery), so the noise-free signal
  differs slightly between runs.
- The older clock demo (`sim/launch/clock_demo.launch.py`) still uses combined
  GUI mode, which has the same `wait_gui` handshake race. It has passed its
  suite every time so far, but should be switched to the `-s` + `-g` split.
