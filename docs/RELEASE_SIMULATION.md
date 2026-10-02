# REEF ROS 2 simulation baseline release `sim-baseline-v0.1.0`

Status: **gates PASS, H11 pending**. The release gates, the fresh-clone
offline reproduction and `faults` passed in the original container (§4).
The human reproduction H11 on a fresh image is pending (§6). The annotated tag
`sim-baseline-v0.1.0` is applied only after H11 is accepted (USER, P09).

This checkpoint is the **faithful baseline**. The ported components
reproduce the original code bit for bit, including its documented legacy
defects (§1a, §1b). The tag preserves it; there is no separate legacy branch
(USER). Corrections of the legacy behaviour follow in **P09.5
Modernization** on `main`. Each correction keeps the legacy tests and golden
outputs as historical evidence and adds distinct corrected tests.

Licence: open source, MIT, Copyright (c) 2026 Hunter Swan
([LICENSE](../LICENSE)), for the repository and its new packages. Components
under their own licences:
- the UF REEF AVL ports: MIT, Copyright (c) 2020 University of Florida REEF
  Autonomous Vehicles Lab;
- the vendored `rosflight_msgs`: BSD-3-Clause;
- the X3 model and its derivatives: CC BY 4.0;
- the Gazebo-derived worlds: Apache-2.0.

[NOTICE.md](../NOTICE.md) lists every component with the evidence for its
licence.

**Simulation only.** No mode has run on hardware. Physical-drone support is
pending until P10 supplies the target hardware.

## 1. Release identity

| | |
|---|---|
| Version | `sim-baseline-v0.1.0` (annotated tag, after H11) |
| Commit | code verified at **`132c27c`** (the fresh-clone reproduction, §4). The later commits on `p09-release` change documentation only |
| Criteria | [ACCEPTANCE.md `release` (P09)](ACCEPTANCE.md) |
| Evidence | §4, [reviews/P09.md](reviews/P09.md), review packet [reviews/P09_packet.md](reviews/P09_packet.md) |

## 1a. What "bit-exact parity" covers

Bit-exact parity holds only for the **ported** components. Each is compared
with the pinned original source, compiled unmodified in a reference
harness: every output field, on locked fixtures, as both the ROS-free core
and the ROS node.

| Component | Reference | Parity evidence |
|---|---|---|
| `reef_estimator` | master `e4179f48` | 50 event streams, all Z and XY fields, with `correction_c1_clear_xy_flag: false` (master semantics). The default output (C1 on) is checked against an independent model |
| `reef_control` | `12237b76` | 20 streams plus a recorded stream; K1–K12 asserted |
| `rgbd_to_velocity` | `b7637198` | 53 assertions; Q1–Q10 asserted |
| `reef_msgs` helpers | `7fb63ff9` | 2930 recorded helper cases |

The other components are **not** ports and claim no parity:
- **`reef_rgbd_odometry`** is a **modern replacement** for `demo_rgbd` (an
  OpenCV front-end of our own, not a port). It is judged against simulation
  truth (VISION.md §6–8).
- **`reef_fc_standin`** is a **stand-in** for the ROSflight firmware's command
  mux, attitude loop and mixer: a development tool, not ROSflight. It is
  judged by the closed-loop criteria (CONTROL_CHAIN.md §7).
- **`reef_sim`** and **`reef_x3_adapter`** provide the simulation and its
  idealized inputs. They are judged against truth and their interface
  checks.

## 1b. Legacy configuration of this checkpoint

| Item | Setting in this release |
|---|---|
| Estimator correction **C1** (`correction_c1_clear_xy_flag`) | **`true` by default** (approved at R1). It corrects legacy defect **D1** (after a partial XY update the last measurement was re-fused at every IMU step). **`false`** reproduces master exactly; every parity run uses `false` |
| Estimator corrections C2–C6 | **not implemented** (deferred). Legacy behaviour D2–D9 is kept: Z/XY bias sign convention, hard-coded g = 9.81, no airborne start, gates on the previous R, no tilt compensation of range, float32 truncations, NaN-check of acceleration only, double dt after a NaN |
| Controller legacy behaviour **K1–K12** | **kept and asserted** (CONTROL_CHAIN.md §5): D term on the state with the wrong sign (K1), ineffective anti-windup (K2), first-sample derivative kick and huge first dt (K3, K4), no output inhibition (K5), a NaN latched in the differentiator (K6), heading without wrapping (K9), no integrator or differentiator resets (K11), and the rest |
| `rgbd_to_velocity` legacy behaviour **Q1–Q9** | **kept and asserted** (VISION.md §3) |
| Invalid parameters (D10, K13, Q10) | the **only behavioural deviations** from the originals, by the project rule "invalid parameters are errors at startup". The originals left a wrong-length matrix uninitialized (D10), clamped out-of-range gains silently (K13), or used zeros for missing extrinsics (Q10); the ports exit with an error naming the parameter |
| Odometry characterizations V1, V2 | kept, documented (VISION.md §8) |
| Simulation | IMU vibration assumption on (`x3_reef_overlay.yaml`); `enable_mocap_switch: false`; X3 controller gains `reef_control_x3_sim.yaml` (`dI` = 0) |

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

### Test accounting

Every reported item belongs to one of three classes, counted separately:

| Class | Meaning | Can it fail? | Counted as |
|---|---|---|---|
| **Acceptance check** | a criterion from ACCEPTANCE.md or a test of required behaviour: parity, limits, data paths, unit tests | yes | PASS / FAIL |
| **Characterization assertion** | asserts that a documented legacy behaviour is reproduced (D-, K-, Q-items, the P07b "CHARACTERIZATION" items). It is evidence of faithfulness, not of correctness, and is expected to change in P09.5 | yes: it fails if the behaviour changes | separately, as "characterizations asserted" |
| **REPORTED** | a measurement with no limit (noise, latency, performance, drift, the weak-texture outcome) | no | never counted as PASS |

An item that cannot fail is never counted as PASS. At P09 four such items
were found, each recorded as PASS by its check: the "range geometry note"
(`analyze_x3_bag`), and three "(reported)" items in the P07b scenarios
`controller_restart`, `range_loss` and `velocity_loss`. They are now
REPORTED (`e61c2ba`). A fifth suspect, `check_control.py`'s "recorded
estimate stream", is a real check: it fails when the stream is missing.

| Check (release run) | Items | Acceptance | Characterizations asserted | REPORTED (not counted) |
|---|---|---|---|---|
| P02 reference harness (`check_baseline.py`) | 36 | 29 | 7 (legacy D-items: bias sign, g = 9.81, re-fusion, outliers, airborne start, …) | — |
| estimator parity (`check_port.py`, 50 streams) | 253 | 252 | 1 (D1 re-fusion counted with C1 off) | — |
| estimator, recorded stream | 5 | 5 | — | — |
| controller (`check_control.py`) | 106 | 93 | 13 (K1–K13) | — |
| `rgbd_to_velocity` (`check_rgbd.py`) | 53 | 43 | 10 (Q1–Q10) | — |
| P07 closed loop: nominal / causality | 26 / 19 | 26 / 19 | — | — |
| vision: open loop / faults run / closed loop | 20 / 24 / 17 | 20 / 24 / 17 | — | 4 / 6 / 9 |
| P07b scenarios (`faults`, 11 runs) | 90 before the fix | 81 | 6 (dropout_long, estimator_reset, setpoint_stale, standin_exit, position_square, face_target) | 3 (since `e61c2ba`) |
| stock flight (`analyze_x3_bag`) | 30 | 29 | — | 1 (since `e61c2ba`) |

The colcon unit, interface and launch tests (175 test cases) are acceptance
tests.

Container commands, original container, headless, idle machine.

| Gate | Command | Exit | Result | Log |
|---|---|---|---|---|
| release, **vision** profile (includes every core gate) | `scripts/reef_check.sh release --profile vision` | **0** | PASS, 68 min, on `9ca23eb` plus 13 uncommitted licence and doc paths (no functional change) | `log/checks/reef_check_release_20261002_031331` |
| ↳ environment, pinned resources | (inside) | 0, 0 | PASS | |
| ↳ code quality | `scripts/check_code_quality.sh` | 0 | shellcheck 28 scripts, pyflakes 59 files, 69 suppressions with reasons, **0 compiler warnings** (fresh build, 9 packages), **130 ASan/UBSan gtests clean** | |
| ↳ `interfaces` | | 0 | 5/5 (vendored pin plus negative, helper vectors, colcon 175 test cases, golden unchanged) | `log/checks/reef_check_interfaces_20261002_032232` |
| ↳ `baseline` | | 0 | P02 36/36; port vs original 253/253 (50 streams, 282 490 events) | `log/checks/reef_check_baseline_20261002_032444` |
| ↳ `estimator` | | 0 | 6/6: plausibility vs truth, recorded-stream parity 5/5, replays | `log/checks/reef_check_estimator_20261002_035022` |
| ↳ `control` | | 0 | controller 106/106; P07 closed loop 26/26 (staleness p99 20 ms, at the limit); causality 19/19 (p99 14 ms) | `log/checks/reef_check_control_20261002_035732` |
| ↳ `vision` | | 0 | `rgbd_to_velocity` 53/53; open loop 20/20; faults 24/24; closed loop on vision 17/17 (staleness p99 16 ms) | `log/checks/reef_check_vision_20261002_040857` |
| fast CI | `scripts/ci.sh` | 0 | 6/6, about 30 min | `log/ci/ci_20261002_023200` |
| regressions | `scripts/regress_clock_check.sh`, `scripts/regress_x3_scenario.sh` | 0, 0 | all cases PASS (after the `env.sh` change) | |
| **fresh-clone reproduction**, core, **offline** | `scripts/reproduce_release.sh` | **0** | PASS, 75 min. Clone of `132c27c` (0 uncommitted paths; no build, assets, recordings or reference clones). Setup online by the documented steps. Offline guard proven (Git, curl, Python blocked). Release gate core PASS: code quality as above, interfaces, baseline 36/36 and 253/253, estimator, control 106/106 + 26/26 + 19/19, `rgbd_to_velocity` 53/53. No download during the offline phase | `build/release_repro/20261002_052145/reproduction_summary.txt` |
| `faults` (11 P07b scenarios; not in a profile), after the accounting fix | `scripts/reef_check.sh faults` | **0** | PASS: F1–F12 36/36; the 11 scenarios 87 judged (81 acceptance + 6 characterizations) + 3 REPORTED | `log/checks/reef_check_faults_20261002_044924` |
| `regress_x3_scenario.sh` (after the `reef_sim` accounting change) | | 0 | all cases PASS | |
| H11 (fresh image, Mac) | §6 | PENDING | | |

Representative scenario, P07 closed-loop nominal (REPORTED; simulation
timing is not deterministic):

| Metric | Clone (`132c27c`) | Reference run |
|---|---|---|
| judged items | 26/26 | 26/26 |
| REEF takeoff after arming | 4.15 s | 4.15 s |
| max tilt | 0.090 rad | 0.085 rad |
| worst velocity RMSE | 0.0088 m/s | 0.0085 m/s |
| worst altitude-hold RMSE | 0.015 m | 0.017 m |
| estimate age p99 | 16 ms | 20 ms |
| yaw rate (command 0.3) | 0.262 rad/s | 0.262 rad/s |

Skipped: none of the gates above. The P07 staleness at the limit (20 ms)
matches earlier P07 runs (12–24 ms) and is the documented idle-host
dependence (§5).

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
