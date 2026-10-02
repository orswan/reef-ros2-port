# P09 review packet: reproduce the simulation baseline release

For an engineer new to the project. Goal: from nothing, reproduce the
release gate and one scored demo, and know where to look when something
fails. Everything is **simulation only**.

## What you are reproducing

- Commit: see [RELEASE_SIMULATION.md §1](../RELEASE_SIMULATION.md); tag
  `sim-baseline-v0.1.0` once accepted.
- A ROS 2 Jazzy port of the UF REEF estimator and controller.
  - Bit-identical to the original ROS 1 code, including its documented
    legacy defects.
  - It flies a Gazebo Harmonic X3 through a **stand-in** low-level loop (a
    development tool, not ROSflight).
- An RGB-D path: a **replacement** OpenCV odometry (not `demo_rgbd`) feeding
  the ported `rgbd_to_velocity`.
- REEF's attitude and range inputs are idealized: derived from simulation
  truth.

## Steps

1. **Image and container.** On a Mac with Docker, in a separate clone,
   follow README "Reproducing the release". It creates instance
   `reef_ros2_h11`, port 8082, separate from any working container.
2. **Setup** (container terminal; needs the network once):
   `scripts/setup_assets.py`, then `baseline/fetch_sources.sh`.
3. **Release gate:** `scripts/reef_check.sh release`.
   - Core profile, about 75 min on an idle machine; it should exit 0.
   - `--profile vision` adds the RGB-D gates (about 100 min).
4. **One demo and its scorer:** `scripts/reef_demo.sh closed-loop`, then
   read `recordings/<run>/analysis_closed_loop/report.txt` and
   `closed_loop.png`.
5. **Compare with the reference numbers:** RELEASE_SIMULATION.md §4. Exact
   timings differ run to run; the judged items must pass.

## Where things are

| Need | Look at |
|---|---|
| what a check does | `scripts/reef_check.sh help` |
| a check's logs | the `logs in log/checks/...` line it prints; `release_summary.json` |
| a run's inputs and versions | `recordings/<run>/manifest.yaml` |
| why a criterion is what it is | docs/ACCEPTANCE.md (criteria are fixed before code) |
| what is legacy behaviour, and where it is asserted | RELEASE_SIMULATION.md §1b; BASELINE_DECISION.md (D/C), CONTROL_CHAIN.md §5 (K), VISION.md §3 (Q) |
| what is supported | INTERFACES.md §5 (capability matrix) |
| licences | NOTICE.md |
| common failures | RELEASE_SIMULATION.md §8 |

## Interpreting a failure

- Exit **2** means BLOCKED (setup missing, a pin mismatch); nothing was
  judged.
- Exit **1** means a check ran and failed. The FAIL line names the
  criterion and the measured value.
- A **staleness** FAIL (estimate age p99 > 20 ms) almost always means
  another process was using the CPU. Rerun on an idle machine.
- A **parity** FAIL means the port no longer reproduces the original. It is
  never fixed by loosening tolerances, which are fixed in ACCEPTANCE.md §4.
