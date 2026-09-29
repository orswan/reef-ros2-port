# REEF ROS 2

Port of the [REEF Estimator](https://github.com/uf-reef-avl/reef_estimator)
stack (UF REEF AVL, ROS 1 / catkin) to **ROS 2 Jazzy** with **Gazebo Harmonic**
simulation.

Status: starter phase. Upstream sources have been inspected and the sim-time
bridge is demonstrated. No packages have been ported yet. See
[docs/MIGRATION.md](docs/MIGRATION.md) for findings, pinned upstream commits,
dependencies, and next steps.

## Layout

| Path | Purpose |
|---|---|
| `src/` | ROS 2 packages (colcon source space); empty for now |
| `sim/` | Gazebo worlds and launch files for demos/tests |
| `scripts/` | Launch and check scripts; they set up their own environment |
| `docs/` | Migration notes |
| `reference/` | Upstream ROS 1 clones for reading only. Ignored by Git; `COLCON_IGNORE` keeps colcon out |

## Quick start

### Inside the container (VS Code attached terminal or `docker exec` shell)

The paths are container paths. The scripts set up their own ROS and display
environment and do not rely on `~/.bashrc`, so any container shell works.

```bash
cd /root/ros2_ws/reef_ros2
scripts/check_env.sh                     # ROS/Gazebo/overlay sanity checks
scripts/check_display.sh [--start]       # Xvfb :99 / noVNC status (--start runs /root/start_vnc.sh)
scripts/run_clock_demo.sh                # Gazebo GUI + /clock bridge on :99
scripts/check_clock_demo.sh              # automated: start, assert its own sim time advances, tear down
REEF_HEADLESS=1 scripts/check_clock_demo.sh
scripts/regress_clock_check.sh           # regression suite for the checker (~2 min)
```

### On the Mac host

The `/root/...` paths above do not exist on the Mac. Run them through the
existing container:

```bash
docker exec ros2_novnc_container /root/ros2_ws/reef_ros2/scripts/check_env.sh
docker exec -it ros2_novnc_container bash    # then use the container commands above
```

Open http://localhost:8080/vnc.html in a Mac browser to see the Gazebo GUI.
The project is also visible on the Mac through the `/root/ros2_ws` bind mount;
its Mac-side path depends on how the container was created.

### Environment knobs

| Variable | Default | Used by |
|---|---|---|
| `REEF_DISPLAY` | `:99` | all scripts |
| `REEF_HEADLESS` | `0` | run/check demo |
| `REEF_DISCOVERY_RANGE` | `LOCALHOST` | all scripts (sets `ROS_AUTOMATIC_DISCOVERY_RANGE`) |
| `ROS_DOMAIN_ID`, `GZ_PARTITION` | unset | passed through to `run_clock_demo.sh` |
| `REEF_CHECK_SECONDS` | `5` | check window |
| `REEF_STARTUP_TIMEOUT` | `60` | wait for first clock message |
| `REEF_TEST_ROS_DOMAIN_ID` | random 1–101 | `check_clock_demo.sh` (overrides `ROS_DOMAIN_ID`) |
| `REEF_TEST_GZ_PARTITION` | unique per run | `check_clock_demo.sh` (overrides `GZ_PARTITION`) |

`check_clock_demo.sh` isolates itself with a per-run Gazebo partition and ROS
topic, and fails if its own launch dies. Exit codes are 0 pass, 1 clock check
failed, 2 demo failed, 124 timeout, 130/143 interrupted. See
[docs/MIGRATION.md §5](docs/MIGRATION.md#5-sim-time-bridge-demonstration-v).

To recreate the reference clones, see the commit table in
[docs/MIGRATION.md](docs/MIGRATION.md#2-reference-sources-inspection-only-never-built).

## Building (once packages exist)

Inside the container, build from **this directory**, not from `/root/ros2_ws`. The parent workspace's
colcon run would also discover packages in `reef_ros2/src`:

```bash
cd /root/ros2_ws/reef_ros2
env -i HOME=$HOME PATH=/usr/bin:/bin bash --norc -c \
  'source /opt/ros/jazzy/setup.bash && colcon build --base-paths src'
```
