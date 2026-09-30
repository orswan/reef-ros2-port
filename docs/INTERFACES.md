# Interfaces

Current contracts for the project's command line, simulation topics, and
(planned) REEF estimator. **Implemented** means exercised by a check in this
repository; **PLANNED** or **NOT DEFINED** means no ROS 2 implementation exists.

## 1. Command interface

Container terminal, from `/root/ros2_ws/reef_ros2`. Both wrappers set up
their own environment (`scripts/env.sh`); nothing needs sourcing first.

```bash
scripts/reef_check.sh help
scripts/reef_check.sh env
scripts/reef_check.sh clock [--gui] [--regress]
scripts/reef_check.sh sim-data [--gui] [--regress]
scripts/reef_check.sh baseline [--floor]
scripts/reef_demo.sh help
scripts/reef_demo.sh stock [--gui]
scripts/reef_demo.sh replay recordings/<run> [--rate R]
```

Not yet implemented (each says NOT IMPLEMENTED and exits 2):
`reef_check.sh estimator|faults|control|vision|release` and
`reef_demo.sh estimator|closed-loop|vision`. All demo modes are **simulation
only**; no mode can reach hardware.

### Exit status (wrapper boundary)

| Code | Meaning |
|---|---|
| 0 | PASS: every executed assertion passed (demo: completed) |
| 1 | FAIL: a check executed and failed |
| 2 | BLOCKED (prerequisite missing: assets, display), NOT IMPLEMENTED, or invalid invocation. Nothing was judged |
| 130 / 143 | interrupted by SIGINT / SIGTERM. The signal is forwarded to the running check, which cleans up its own processes |

The wrapped scripts keep their existing codes, which are mapped here:

| Script | Its codes | Wrapper result |
|---|---|---|
| `check_clock_demo.sh` | 0 pass, 1 clock check failed, 2 owned demo failed or bad input, 124 timeout | 0 → PASS; 1, 2, 124 → FAIL (prerequisites are checked by the wrapper first, so 2 here is an executed failure) |
| `run_x3_scenario.sh` | 0 pass, 1 analysis failed, 2 setup or owned simulation failed, 3 scenario runner failed, 124 timeout | 0 → PASS; others → FAIL (assets and display are prechecked; a missing one → BLOCKED 2) |
| `clock_check.py` (negative case) | 1 = no clock, as required | PASS only if it exits exactly 1 |
| `regress_*.sh` | 0 all cases matched, 1 otherwise | 0 → PASS, 1 → FAIL |
| `baseline/tools/check_baseline.py` | 0 all assertions passed, 1 a check failed, 2 sources unavailable | 0 → PASS, 1 → FAIL (sources are prechecked → BLOCKED 2) |

### Report

Each `reef_check.sh` run ends with:
- **source:** commit and branch, and the number of uncommitted or untracked paths.
- **environment:** dev image package-list hash or "not the dev image", and the gz version.
- **config:** SHA-256 of the configuration and input files of the target.
- **assertions:** each one with PASS/FAIL, its actual and expected exit code, and wall time.
- **sim time:** where relevant, the sim-time span observed or the scenario end time.
- **wall time**, and **artifact** paths: logs in `log/checks/reef_check_<target>_<time>/`,
  recordings in `recordings/`.

## 2. Simulation interfaces (implemented)

### Clock demo (`sim/`)

`/clock` (`rosgraph_msgs/Clock`), bridged from Gazebo. `check_clock_demo.sh`
remaps it to a per-run topic (`/reef_clock_check_<token>/clock`), with a
per-run `GZ_PARTITION` and ROS domain.

### X3 scenario (`src/reef_sim`)

The full specification is in [X3_SCENARIO.md](X3_SCENARIO.md). Summary:

| Topic | Type | Category | Frame | Rate | Time basis |
|---|---|---|---|---|---|
| `/x3/truth/odom` | `nav_msgs/Odometry` | TRUTH (twist: smoothed finite difference) | `world` (ENU) → `x3/base_link` (FLU) | 100 Hz | sim time, header stamp = Gazebo sample time |
| `/x3/imu` | `sensor_msgs/Imu` | measurement; specific force (+g on z at rest); orientation not provided | `x3/base_link` (FLU) | 250 Hz | sim time |
| `/x3/range` | `sensor_msgs/Range` | idealized measurement derived from truth; slant range; REP 117 ±inf | `x3/range_link` | 20 Hz | sim time (the truth sample it was computed from) |
| `/x3/cmd_vel` | `geometry_msgs/Twist` | command to the truth-fed stock controller | body FLU | 20 Hz | sim time |
| `/x3/scenario/phase` | `std_msgs/String` | phase label (transient local) | — | per phase | recorder receive time (sim) |

**Freshness:** no staleness detection is implemented on these topics. Consumers
must compare header stamps with the sim clock themselves. Freshness and
health semantics for REEF inputs are defined in P03.

**Attitude and yaw sources:** the only attitude or yaw in the simulation is
the **truth** orientation in `/x3/truth/odom`, which is **idealized**. No
attitude estimator is implemented. The IMU deliberately does not carry
orientation.

## 3. REEF estimator interface (PLANNED, NOT DEFINED for ROS 2)

The upstream ROS 1 interface ([MIGRATION.md §3](MIGRATION.md#3-upstream-estimator-facts-v))
is the starting point:

| Upstream (ROS 1) | Type | ROS 2 status |
|---|---|---|
| sub `imu/data` | `sensor_msgs/Imu` (REEF expects FRD / NED) | NOT DEFINED; needs FLU → FRD conversion from `/x3/imu` |
| sub `sonar` | `sensor_msgs/Range` (used as vertical altitude, no tilt compensation; accepts `range <= max_range`) | NOT DEFINED; the adapter must drop ±inf and choose a tilt policy |
| sub `rc_raw` | `rosflight_msgs/RCRaw` (mocap-override switch) | NOT DEFINED. In simulation there is no RC source; when ported, the switch must be an **explicit parameter, disabled in simulation**, keeping the hardware path |
| sub mocap pose and velocity | `PoseStamped` (NED), `TwistWithCovarianceStamped` (body-level) | NOT DEFINED; truth-derived versions must be labelled idealized |
| sub RGB-D velocity | `reef_msgs/DeltaToVel` | NOT DEFINED (P08) |
| attitude input | from ROSflight's attitude estimate (roll/pitch for `C_NED_to_body`) | NOT DEFINED; simulation can supply only idealized truth |
| pub `xyz_estimate`, `xyz_debug_estimate`, `is_flying_reef` | `reef_msgs/*`, `std_msgs/Bool` (latched in ROS 1) | NOT DEFINED; transient-local QoS planned |

REEF estimates altitude, vertical velocity, horizontal velocity, and
biases. It is **not** a full pose estimator, and a ROS 2 output must not
present unestimated components as measured. Units, frames, covariance
layout, QoS, health/reset behavior, and parameter validation will be
fixed in P03 and recorded here before implementation is scored.
