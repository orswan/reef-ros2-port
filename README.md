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

These commands can be run from any shell. The scripts do not rely on `~/.bashrc`.

```bash
cd /root/ros2_ws/reef_ros2
scripts/check_env.sh                     # ROS/Gazebo/overlay sanity checks
scripts/check_display.sh [--start]       # Xvfb :99 / noVNC status (--start runs /root/start_vnc.sh)
scripts/run_clock_demo.sh                # Gazebo GUI + /clock bridge (view at http://localhost:8080/vnc.html)
scripts/check_clock_demo.sh              # automated: start, assert sim time advances in ROS 2, tear down
REEF_HEADLESS=1 scripts/check_clock_demo.sh
```

Environment knobs: `REEF_DISPLAY` (default `:99`), `REEF_HEADLESS=1`,
`REEF_CHECK_SECONDS` (default 5), `REEF_DISCOVERY_RANGE` (default `LOCALHOST`),
and `ROS_DOMAIN_ID`.

To recreate the reference clones, see the commit table in
[docs/MIGRATION.md](docs/MIGRATION.md#2-reference-sources-inspection-only-never-built).

## Building (once packages exist)

Build from **this directory**, not from `/root/ros2_ws`. The parent workspace's
colcon run would also discover packages in `reef_ros2/src`:

```bash
cd /root/ros2_ws/reef_ros2
env -i HOME=$HOME PATH=/usr/bin:/bin bash --norc -c \
  'source /opt/ros/jazzy/setup.bash && colcon build --base-paths src'
```
