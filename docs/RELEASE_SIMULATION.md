# REEF ROS 2 simulation release `reef-sim-v0.1.0`

Status: **PENDING**. The release gates were run in the original container
(§4), and the human reproduction H11 is pending (§6). The local tag
`reef-sim-v0.1.0` is applied only after H11 is accepted, and is never
published (USER, P09).

Licence: open source. The project's own packages are MIT or Apache-2.0; the
UF REEF AVL ports are MIT; the vendored `rosflight_msgs` is BSD-3-Clause; the
X3 model is CC BY 4.0. [NOTICE.md](../NOTICE.md) lists every component and
the evidence for its licence.

**Simulation only.** No mode has run on hardware. Physical-drone support is
pending until P10 supplies the target hardware.

## 1. Release identity

| | |
|---|---|
| Version | `reef-sim-v0.1.0` (local tag, after H11) |
| Commit | PENDING (the commit the §4 gates ran on) |
| Criteria | [ACCEPTANCE.md `release` (P09)](ACCEPTANCE.md) |
| Evidence | §4, [reviews/P09.md](reviews/P09.md), review packet [reviews/P09_packet.md](reviews/P09_packet.md) |

## 2. Tested configuration

[V] Measured in the original container, which runs the same distribution
packages as the dev image but is not built from the `Dockerfile`:

| Component | Version |
|---|---|
| OS | Ubuntu 24.04.4 LTS, x86_64 |
| ROS 2 | Jazzy (`rclcpp` 28.1.21, `rclpy` 7.1.11) |
| Gazebo | Harmonic, `gz sim` 8.15.0 |
| `ros_gz` | `ros_gz_sim`, `ros_gz_bridge` 1.0.24 |
| rosbag2 | 0.26.11 (MCAP storage) |
| Compiler | GCC 13.3.0 |
| OpenCV, Eigen, NumPy | 4.6.0, 3.4.0, 1.26.4 |
| Dev image | `Dockerfile` (base `ros:jazzy@sha256:c3706ef0…`); its exact package list is recorded at build time in `/etc/reef-image-packages.txt`. The H11 image's package-list hash: PENDING (§6) |

Every run records its own versions and inputs in its outputs (§7).

## 3. Profiles and what they cover

| Profile | Command | Covers | Wall time (idle machine) |
|---|---|---|---|
| **core** (default release gate) | `scripts/reef_check.sh release` | environment; pinned resources; code quality; the `interfaces`, `baseline`, `estimator` and `control` targets; `rgbd_to_velocity` parity | about 75 min |
| **vision** (separate gate) | `scripts/reef_check.sh release --profile vision` | core plus the `vision` target: camera interface, replacement odometry, REEF on vision, degraded and fault cases, closed loop on vision | about 100 min |
| fast CI | `scripts/ci.sh` (`--full` adds fresh-build warnings and the sanitizers) | code-quality fast gates, interfaces, the three parity checks, one short headless scenario | about 30 min |
| fresh-clone reproduction | `scripts/reproduce_release.sh [--profile …]` | clone of the committed HEAD → documented setup (online) → release gate **offline** in the clone | core profile plus setup |

Code quality (`scripts/check_code_quality.sh`) checks:
- `shellcheck -x` on every script;
- pyflakes on the project's Python;
- every lint suppression listed, each with a same-line reason;
- zero compiler warnings in a fresh build of all packages
  (`-Wall -Wextra -Wpedantic`; `src/third_party/` excluded);
- the AddressSanitizer + UndefinedBehaviorSanitizer gtests
  (`scripts/check_sanitizers.sh`).

The rendering-heavy and long suites run only on request: `faults` (about 30
min), `vision`, the GUI demos.

### Capability matrix (summary of [INTERFACES.md §5](INTERFACES.md))

| Mode | Status | Profile | Evidence |
|---|---|---|---|
| IMU (accelerometer, gyro) | supported in simulation (seeded noise, vibration assumption) | core | `estimator`, `control` |
| Attitude, range for REEF | supported with idealized input (from truth) | core | `estimator` |
| Mocap body-level velocity | supported with idealized input (truth plus noise; the `position_to_velocity` producer is not ported) | core | `estimator`, `control` |
| RGB-D body-level velocity | supported in simulation (Gazebo camera → replacement odometry → ported `rgbd_to_velocity`) | vision | `vision` |
| Mocap z, RC override switch | port tested only (fixtures, unit tests) | core | `baseline`, gtests |
| Controller velocity mode | supported in simulation (stand-in low-level loop) | core; vision with RGB-D | `control`, `vision` |
| Controller position mode, `face_target` | supported with idealized input (truth-derived mocap pose) | (`faults`, on request) | `faults` |
| VRPN mocap, joystick teleop | Unsupported/Deferred (USER) | — | — |
| Astra driver, `demo_rgbd` | Unsupported/Deferred (replaced in simulation) | — | VISION.md §6 |
| `setpoint_generator`, `dubins_path` | Unsupported/Deferred (not ported) | — | — |
| ROSflight firmware, `rosflight_io`, hardware output | Unsupported/Deferred (P10); the stand-in is a development tool | — | CONTROL_CHAIN.md §7 |

## 4. Gates and results

PENDING: the commands, exit codes and logs of the release runs (core,
vision), the fresh-clone reproduction and CI, filled in from their
`release_summary.json` and `reproduction_summary.txt`.

## 5. Known limits

- **Idealized inputs.** REEF's attitude and range come from simulation
  truth. The vision release replaces only the horizontal velocity input with
  vision.
- **Not ROSflight.** The low-level loop is a stand-in (development tool).
  The controller and estimator are bit-identical ports, but no ROSflight
  firmware runs.
- **Timing.** Closed-loop staleness (estimate age p99 ≤ 20 ms) needs an idle
  machine: p99 14 ms here, 12–24 ms over the P07 runs. The Python nodes
  `imu_noise` and `range_sensor` still spend about 0.6–0.7 of a core each on
  `/clock` (accepted, USER).
- **Exact simulated depth.** The replacement odometry's inlier threshold
  relies on Gazebo's noise-free depth (VISION.md §6).
- **No failsafes.** In line with the legacy system: crashes in the dropout
  and stand-in-exit cases and the weak-texture wall contact are documented
  characterizations.
- **Offline gate, agent side.** The containers allow no network namespace.
  The agent's offline run blocks downloads with `REEF_OFFLINE=1` and proves
  the block with negative controls; H11 on the Mac can disconnect the
  network for real.

## 6. Human reproduction (H11)

Mac terminal and container steps: [README "Reproducing the release"](../README.md).

1. Clone the release commit into a **separate** directory on the Mac. Build a
   fresh image with `docker compose --env-file docker/h11.env build --no-cache`
   (instance `reef_ros2_h11`, port 8082, its own model-cache volume). The
   working dev container and `ros2_novnc_container` stay untouched.
2. In that container: `scripts/setup_assets.py`, `baseline/fetch_sources.sh`
   (the only steps that need the network). Confirm the X3 meshes and textures
   (`assets/models/x3/`) and the camera calibration files
   (`src/rgbd_to_velocity/config/`) are present.
3. Optionally disconnect the Mac from the network. Then run
   `scripts/reef_check.sh release`, which should exit 0 with
   `release_summary.json` in `log/checks/reef_check_release_*/`.
4. Run one demo and its scorer, `scripts/reef_demo.sh closed-loop`, and open
   `recordings/<run>/analysis_closed_loop/report.txt`, `closed_loop.png` and
   `manifest.yaml`.
5. Check that the logs are findable and failures interpretable, that NOTICE.md
   and this document match what you saw, and record the image's package-list
   hash (`sha256sum /etc/reef-image-packages.txt`) here.
6. `docker compose --env-file docker/h11.env down` from the clone.

## 7. Data, configuration and manifests

| File | Records |
|---|---|
| `recordings/<run>/manifest.yaml` | git commit, branch, uncommitted changes, container and image identity, gz/ros_gz/rosbag2 versions, world/vehicle/asset hashes and licences, seeds, parameters, outcome |
| `src/reef_sim/assets/*.json` | third-party model pins: URL, version, licence, SHA-256 |
| `baseline/provenance.json`, `baseline/*/provenance.json` | the pinned upstream commits and file checksums of the reference harnesses |
| `src/third_party/*/UPSTREAM.json` | vendored package pins |
| `baseline/**/fixtures*.lock.json`, `baseline/golden/index.json` | locked fixtures and golden outputs (changed only with a recorded reason) |
| `log/checks/reef_check_release_*/release_summary.json` | release gate: commit, profile, image hash, versions, every step with its exit code |
| `build/release_repro/<time>/reproduction_summary.{txt,json}` | fresh-clone reproduction |
| `log/ci/ci_*/summary.txt`, `junit.xml` | CI runs |

## 8. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `BLOCKED: X3 assets missing or modified` | run `scripts/setup_assets.py` (network once); a modified asset fails its SHA-256 |
| `BLOCKED: pinned upstream sources unavailable` | run `baseline/fetch_sources.sh` (network once) |
| `BLOCKED: shellcheck not found` | the dev image has it; elsewhere set `REEF_SHELLCHECK=<path>` |
| closed-loop **staleness** FAIL (estimate age p99 > 20 ms) | CPU contention: rerun on an idle machine (no other simulation or build running, in either container) |
| `ERROR: unrelated overlay /root/ros2_ws/install is on AMENT_PREFIX_PATH` | an interactive shell sourced another workspace; use the project scripts, which re-exec in a clean environment |
| a build fails after an image change | build trees are per environment (`build/colcon_env/<hash>`); a new image gets a new tree automatically. Old trees can be deleted |
| `--gui` checks: `browser desktop not ready` | start or wait for the desktop: `scripts/check_display.sh --start` (dev container: it starts with the container) |
| port 8081 or 8082 already in use on the Mac | another instance is up: `docker compose ls`; use the matching `--env-file` |
| where are the logs? | each check prints `logs in log/checks/reef_check_<target>_<time>/`; each run prints `run directory: recordings/<run>` |
