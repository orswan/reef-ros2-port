# REEF ROS 2 migration — project handoff

Updated: 2026-09-29 (UTC). This is a conversation-derived handoff, not an inspection of the user's current repository. Reconcile it with Git, project instructions, and test reports before working. Do not claim later milestones are complete merely because prompts were provided.

Companion files: [implementation prompts](REEF_COMPLETION_PROMPTS.md) and [human testing and review](REEF_TESTING_AND_REVIEW.md).

## 1. Goal, user, and constraints

The user collaborates with a group associated with the University of Florida REEF Autonomous Vehicles Laboratory. They want a ROS 2 port of the REEF estimator flight stack for Gazebo and eventually physical multirotors, retaining the research approach described in the REEF papers.

- The user is comfortable with scientific programming but wants Claude to do most infrastructure and implementation work.
- OpenAI credits are limited. Prefer durable project documents, executable checks, small commits, and sparse independent reviews over reviewing every change with another paid model.
- Development is remote, on an Intel Mac using Docker Desktop. Do not assume Apple Silicon.
- Target: Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic in Linux containers.
- An older Ubuntu laptop exists but is optional. Do not make it a project dependency.
- Physical flight recordings and hardware access are not currently available. Begin with simulation-generated data.
- Eventual flight-controller hardware, firmware version, companion-computer architecture, sensors, and lab flight procedures are UNKNOWN.
- Do not substitute a different estimator or autopilot architecture without explicitly recording the change and its implications for the REEF objective.

## 2. Evidence and current status

Evidence labels: USER = user directly reported the result; REVIEW = pasted independent Codex review of a specified revision; IMPLEMENTER = pasted Claude completion report; PLANNED = guidance or prompts only.

| Item | Evidence/status |
|---|---|
| Original container runs Gazebo with browser graphics | USER confirmed |
| VS Code Dev Containers installed and used | USER reported |
| Initial starter project committed | REVIEW confirmed commit 243180ab3eb07e8c2db14f49e5874c628da6ddb0 on main |
| Real Gazebo clock reaches ROS 2 | REVIEW confirmed headless and GUI checks at that initial revision |
| Initial checker reliable in isolation/failure cases | NO: REVIEW demonstrated a false pass and SIGTERM observer leak |
| Fixes to those defects | Repair/re-review prompts supplied; final fix commits and review results were NOT pasted here. Reconcile once against current repo |
| Dockerfile and development-container configuration generated | USER reported complete |
| Mac-terminal and VS Code checks of rebuilt environment | USER reported all behaved as expected |
| Independent review of Dockerfile changes | Prompt supplied; no result recorded here |
| Moving X3 quadrotor, sensors, recordings | Prompts supplied; completion NOT established |
| REEF messages, helpers, estimator, controller port | NOT established |
| RGB-D pipeline, firmware-in-loop integration, real flight | NOT established |
| GitHub remote, PR, current branch/head, release tags | UNKNOWN; initial repo was local |

Do not rerun all historical checks just to reconstruct the conversation. Inspect existing evidence, close genuine gaps, and record the resulting status.

## 3. Paths and environment

| Purpose | Known value / qualification |
|---|---|
| Mac workspace | ~/ros2_ws |
| Original bind mount | ~/ros2_ws on Mac → /root/ros2_ws in container |
| Project on Mac | ~/ros2_ws/reef_ros2 |
| Project in container | /root/ros2_ws/reef_ros2 |
| Original working container | ros2_novnc_container — preserve |
| Original image | ros:jazzy, then manually installed packages |
| Original viewer | http://localhost:8080/vnc.html |
| Original startup script | /root/start_vnc.sh |
| Original display | DISPLAY=:99 |
| Software rendering | LIBGL_ALWAYS_SOFTWARE=1; MESA_GL_VERSION_OVERRIDE=3.3 |
| Proposed replacement name/port | reef_ros2_dev and 127.0.0.1:8081 were recommended; actual values must be read from current files |
| Observed old environment at initial review | Ubuntu 24.04.4, x86_64, Gazebo 8.15.0, virtiofs bind mount |

The working browser setup used ros-jazzy-ros-gz, Xvfb, x11vnc, noVNC, Fluxbox, and websockify. Confirm explicit dependencies in the new Dockerfile; indirect package installation is not a durable recipe. The initial review found Xvfb, x11vnc, and websockify healthy, while Fluxbox was not running. The rebuilt desktop should make window-manager behavior explicit.

VS Code can attach to the existing container using Dev Containers: Attach to Running Container. Later reproducible Dev Containers startup is a different workflow; follow the new README rather than accidentally creating or replacing the old container.

Docker control normally runs on the Mac. An agent inside a container may have no access to the host daemon. It should prepare changes and give exact host commands when needed, without pretending it ran them or installing Docker-in-Docker for convenience.

Keep the original container available. A snapshot command was suggested (docker commit … reef-jazzy-novnc:baseline), but the user reported it appeared to hang; completion was not confirmed here. Do not depend on that snapshot. Docker commits pause running processes by default and exclude mounted workspace contents. Avoid docker container prune: it removes stopped containers, including manually configured ones.

## 4. Project organization and known files

At the reviewed initial commit there were 15 tracked files; src/ was intentionally empty. Neither the parent workspace nor the project was previously a Git repository.

| Path | Role |
|---|---|
| README.md | Reproduction and usage |
| AGENTS.md | Shared agent instructions |
| CLAUDE.md | Imports AGENTS.md |
| docs/MIGRATION.md | Source audit and milestone plan |
| reference/ | Unmodified upstream ROS 1 repositories; excluded from top-level Git |
| reference/COLCON_IGNORE | Tracked marker excluding references from colcon discovery |
| src/ | Future ROS 2 packages |
| scripts/env.sh | Clean environment and display/discovery setup |
| scripts/check_env.sh | Environment check |
| scripts/check_display.sh | Display-services check |
| scripts/run_clock_demo.sh | Gazebo/bridge demonstration |
| scripts/check_clock_demo.sh | Launch-and-observe check, originally flawed |
| scripts/clock_check.py | Clock observer |

Git ignore rules covered generated build/install/log directories, reference clones, and recordings. Build from the project root with discovery limited to src/. A parent-workspace build can discover nested packages. Do not remove the reference ignore marker to make a ROS 1 package appear buildable under ROS 2.

The new Dockerfile/devcontainer paths and current revision are not available in this conversation; inspect them locally.

## 5. Upstream sources and pinned revisions

| Repository | Initial reviewed checkout |
|---|---|
| https://github.com/uf-reef-avl/reef_estimator | e4179f48c3f26e22bd1366b71ee1e117ce2f5f7f |
| https://github.com/uf-reef-avl/reef_estimator_bundle | a20b4e1d81f0c559bbf7e45108332b11b5d27d26 |
| https://github.com/uf-reef-avl/reef_estimator_sim_bundle | 1b5724f1d08a9d7f7d2bf4f20a9f291719f950b6 |

The simulation bundle pins another estimator revision beginning 95987b51. Resolve and record its full SHA locally. Branch labels can move; use commits as evidence.

The user investigated reef_estimator_2 and reported it is an abandoned ROS 1 project, not a ROS 2 port. Do not revisit it as the default starting point.

The full stack spans estimator, reef_msgs, reef_control, teleoperation, velocity-processing/vision packages, motion capture interfaces, and ROSflight. Port the dependencies actually required for each milestone. Do not recursively import every old submodule and attempt to build it under Jazzy.

## 6. Important algorithm differences — baseline is not settled

The independent review compared e4179f48 to 95987b51. Moving from master to the simulation pin reportedly:

1. Disables chi-square acceptance checks for RGB-D velocity, sonar, mocap XY, and mocap Z; retains the sonar maximum-range check.
2. Changes enable_measurements from false to true.
3. Changes Z-model initialization dt from 0.005 to 0.002, changes bias coupling in F, and removes per-update linear-model reconstruction and process-noise scaling.
4. Uses measured initial acceleration magnitude instead of overriding it with 9.81.
5. Clears newRgbdMeasurement after partial updates as well; master clears it only in the full-update branch.
6. Changes initial bias covariance and process/measurement noise parameters.

These are review findings to trace to exact source lines, not a complete independent mathematical assessment in this document. A rejection toggle alone cannot reproduce the simulation branch.

Master was recommended provisionally as a reference. The final algorithm baseline must be chosen explicitly, with deterministic input sequences and state/covariance comparisons. Keep compatibility changes separate from intentional algorithm corrections. Characterize suspected legacy bugs; do not silently preserve them as correct or silently change them during a middleware port.

## 7. ROSflight boundary

The review found that the core estimator directly uses rosflight_msgs/RCRaw for an optional RC switch. Audited reef_msgs helper sources did not add another ROSflight dependency. However, upstream estimator build metadata names rosflight and recording launch files start rosflight_io. Thus “RC-only” does not describe all launch paths.

Upstream https://github.com/rosflight/rosflight_ros_pkgs contains ROS 2 messages and interfaces. Prefer a pinned compatible upstream rosflight_msgs package over inventing a replacement message or deleting a feature merely because a binary package was not found. Verify fields and semantics; build only the needed package/dependencies initially. Disable unused RC switching explicitly in simulation.

Real firmware, board support, command modes, and low-level control compatibility remain future work. ROS 2 host-message support does not prove compatibility with a specific old flight controller.

## 8. What simulation proves

The candidate is a version-matched Harmonic multicopter example using X3. The audit reported that its model needs added IMU and altimeter inputs. A roughly 93 MB first download was an implementer observation, not independently reproduced.

Gazebo's MulticopterVelocityControl reads simulation truth internally to compute rotor speeds. Publishing odometry is a separate matter. A drone flying with that controller while REEF runs alongside it validates estimator observations, not REEF-driven flight control.

Future closed-loop validation must show the REEF controller consumes REEF estimates and drives the intended attitude/actuator interface, with the stock truth-based outer controller disabled. Use one authoritative physics integration and one owner of each control layer.

Initially, label ideal attitude and truth-derived/noisy measurements explicitly. Later RGB-D validation must process rendered images/depth; a noisy truth-derived velocity is not a vision algorithm. Preserve ground truth separately for scoring.

## 9. Initial independent review: defects and successful evidence

Codex's initial review of 243180ab… found:

- HIGH: an unrelated simulation's shared /clock could make the checker PASS when its own launch failed on DISPLAY=:197. LOCALHOST was not sufficient isolation; clean re-execution discarded GZ_PARTITION; launcher survival was unchecked.
- MEDIUM: SIGTERM removed the owned simulator/bridge session but left the Python clock observer running. Normal termination and tested Ctrl-C cases worked. Session-wide cleanup itself targeted a test-created session in inspected cases.
- MEDIUM: estimator-version differences were understated (see section 6).
- MEDIUM: multicopter docs omitted the controller's own truth feedback.
- LOW: docs claimed Jazzy setup always overwrote discovery. Tested setup instead defaulted an unset value to SUBNET and preserved LOCALHOST.
- LOW: container-only commands were described as usable from any shell.
- Minor: a dynamics.h include attributed to sensor_manager.h was actually in include/xy_estimator.h.

Positive evidence: clean reference checkouts; matching pins; correct ignore rules; real headless and GUI clock flow; a simulation-time node's clock advanced; direct no-clock observer failed as expected. Frequencies near 1 kHz and real-time factors around 0.9–1 were observations, not future acceptance thresholds.

Prompts requested fixes, targeted regressions, and a focused re-review. Do not infer completion without checking the resulting commits/reports. Repairs should isolate both ROS and Gazebo traffic, monitor owned launch/process failures, clean the observer, and preserve exit statuses without touching unrelated services.

## 10. Decisions and preferences to carry forward

- Preserve the working old container; develop/test replacements separately.
- Use repeatable simulation data before asking the REEF team for recordings.
- Keep original algorithm references pinned and separately runnable where practical.
- Keep source/data/frame/control contracts explicit. Do not claim full pose estimation when only velocity and altitude are available.
- Use a documented, versioned asset-fetch/cache procedure; do not depend on an undeclared cache in the old container.
- One editing agent at a time. Reviewers inspect before executing scripts and do not edit during review.
- Use automation at every implementation milestone; reserve independent model review for estimator correctness, actual closed-loop control, and hardware command interfaces.
- Human operators own actual hardware arming, firmware flashing, calibration actions, and flight authorization. No generic safe flight gains or vehicle-independent failsafe commands have been established.

## 11. Immediate next action

Read the current repository, identify the actual Dockerfile/fix commits and remaining review findings, and complete P00 in the companion prompt guide. Resume at the first incomplete milestone. Do not rebuild completed work just because a prompt is present.

## 12. Reference materials and access limits

User provided these documents in ChatGPT; they are not automatically available to a separate Claude/Codex session:

- GazeboMacDockerSetup.md — original noVNC recipe; useful history, verify against the current image.
- ROS2_Mac_Docker_Context.md — older ros2_container/XQuartz workflow; its display configuration is superseded for this project.
- REEF Estimator - A Simplified Open Src Estimator Cntllr for MRs - Ramos 19.pdf.
- IEEE 2023 REEF AFL - A collab hub expediting flt capes for DoD and Academics_PUBLISHED.pdf.

Earlier planning used public REEF documentation and an accessible 2019 paper. A complete review of the attached 2023 PDF was not established. Obtain relevant files from the user when needed; never claim to have read inaccessible material.

Primary technical references (checked during this conversation):

- [REEF estimator](https://github.com/uf-reef-avl/reef_estimator)
- [REEF bundle](https://github.com/uf-reef-avl/reef_estimator_bundle)
- [ROSflight ROS 2 repository](https://github.com/rosflight/rosflight_ros_pkgs)
- [Jazzy/Harmonic installation pairing](https://gazebosim.org/docs/harmonic/ros_installation/)
- [Harmonic multicopter controller and its truth feedback](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1MulticopterVelocityControl.html)
- [VS Code attach to an existing container](https://code.visualstudio.com/docs/devcontainers/attach-container)
- [Docker commit behavior](https://docs.docker.com/reference/cli/docker/container/commit/)
- [colcon package discovery](https://colcon.readthedocs.io/en/released/reference/discovery-arguments.html)

Future maintainer: update this handoff with a date, exact current revision, completed gates and links to evidence. Preserve the distinction between historical review findings, current facts, and planned work.
