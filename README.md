# REEF ROS 2

Port of the [REEF Estimator](https://github.com/uf-reef-avl/reef_estimator)
stack (UF REEF AVL, ROS 1 / catkin) to **ROS 2 Jazzy** with **Gazebo Harmonic**
simulation.

Status: starter phase. Upstream sources have been inspected, the sim-time
bridge is demonstrated, and a reproducible dev container is defined. No
packages have been ported yet. See [docs/MIGRATION.md](docs/MIGRATION.md) for
findings, pinned upstream commits, dependencies, and next steps.

Every command below is labelled with where it runs:

- **Mac terminal**: macOS Terminal, in `~/ros2_ws/reef_ros2`.
- **Container terminal**: a shell inside the dev container, either a VS Code
  terminal or `docker compose exec dev bash`. Container paths start with `/root/`.

## Layout

| Path | Purpose |
|---|---|
| `src/` | ROS 2 packages (colcon source space): `reef_sim` (X3 scenario) |
| `sim/` | Gazebo worlds and launch files for demos/tests |
| `scripts/` | Launch and check scripts; they set up their own environment |
| `docs/` | Migration notes; `docs/setup/` has the original container recipe |
| `Dockerfile`, `compose.yaml`, `.devcontainer/`, `docker/` | Dev container definition |
| `reference/` | Upstream ROS 1 clones for reading only. Ignored by Git; `COLCON_IGNORE` keeps colcon out |
| `assets/`, `recordings/` | Downloaded Gazebo models and scenario recordings. Ignored by Git, kept on the Mac |

## Dev container

`compose.yaml` is the single definition of the dev container. The Mac
terminal (`docker compose …`) and VS Code (`.devcontainer/devcontainer.json`)
both use it, so they start the same container:

| | Dev container | Original container (unchanged) |
|---|---|---|
| Container | `reef_ros2_dev` | `ros2_novnc_container` |
| Browser desktop | http://127.0.0.1:8081/vnc.html | http://localhost:8080/vnc.html |
| Project mount | `~/ros2_ws/reef_ros2` → `/root/ros2_ws/reef_ros2` | `~/ros2_ws` → `/root/ros2_ws` |
| Desktop start | automatic, with the container | manual, `~/start_vnc.sh` |

Both can run at the same time. Nothing in this repository stops, modifies, or
reuses the original container or its port 8080.

### First-time build and startup

**Mac terminal:**

```bash
cd ~/ros2_ws/reef_ros2
docker compose build        # first build: ~990 packages (~3 GB installed) on a ~280 MB base; several minutes
docker compose up -d
docker compose ps           # wait until STATUS shows "(healthy)", about 20 s
```

"Healthy" means the image's health check (`reef-desktop status`) found Xvfb,
Fluxbox, x11vnc, and noVNC all answering.

### Verifying a new build

**Mac terminal**, after `docker compose up -d`:

```bash
docker compose exec dev scripts/validate_devcontainer.sh    # expect "== 0 failure(s)"
docker compose restart dev
docker compose exec dev reef-desktop wait 60                  # all four services OK again after restart
docker compose exec dev scripts/validate_devcontainer.sh     # expect "== 0 failure(s)" again
docker compose logs dev | grep -E "desktop ready|stale|ERROR" # one "desktop ready" per start
docker ps --filter name=ros2_novnc_container --format '{{.Names}} {{.Status}} {{.Ports}}'  # original: unchanged
```

Then open http://127.0.0.1:8081/vnc.html (see Browser access below).

### Opening the project in VS Code

1. Install the **Dev Containers** extension in VS Code on the Mac.
2. **File → Open Folder…** → `~/ros2_ws/reef_ros2`.
3. Run **Dev Containers: Reopen in Container** from the Command Palette.

VS Code builds (the first time) and starts `reef_ros2_dev` from the same
`compose.yaml`, or attaches to it if it is already running. The desktop is
started by the container itself, not by VS Code, so reopening or attaching
again never starts a second desktop. After attaching, VS Code runs
`scripts/check_display.sh --start`, which only waits for the desktop.
Terminals opened in VS Code are container terminals.

Closing VS Code leaves the container running (`shutdownAction: none`). Stop it
explicitly (see Daily use).

To check that VS Code and the terminal are using the same container
(**Mac terminal**):

```bash
docker ps --filter name=reef_ros2_dev --format '{{.Names}}  project={{.Label "com.docker.compose.project"}}  {{.Ports}}'
# expected: reef_ros2_dev  project=reef_ros2  127.0.0.1:8081->6080/tcp
```

### Browser access

Open **http://127.0.0.1:8081/vnc.html** in a browser on the Mac and click **Connect**.

What you should see:

- **Desktop only:** a black desktop with the Ubuntu logo and a grey
  **Fluxbox toolbar** along the bottom ("Workspace 1" and a clock). The toolbar
  shows the window manager is running.
- **During a GUI simulation** (for example `scripts/run_clock_demo.sh`): a
  *Gazebo Sim* window with a grey grid and a blue box, an entity tree listing
  `ground_plane`, `box`, and `sun`, and a real-time factor near 90–100 %.
  The window also appears in the Fluxbox toolbar.

The browser server can be up while Gazebo renders nothing, so "the page loads"
is not the same as "Gazebo renders". To check rendering without a browser,
`scripts/validate_devcontainer.sh` saves `log/checks/devcontainer_*/gazebo_gui.png`,
a capture of exactly what noVNC displays.

### Running the simulation checks

**Container terminal:**

```bash
scripts/validate_devcontainer.sh         # image, desktop, headless + GUI clock checks, negative case (~1 min)
scripts/validate_devcontainer.sh --full  # also the clock and X3 regression suites (~9 min more; run setup_assets.py first)
```

The same from the **Mac terminal**:

```bash
docker compose exec dev scripts/validate_devcontainer.sh
```

The individual checks (**container terminal**):

```bash
scripts/check_env.sh                     # ROS/Gazebo/overlay sanity checks
scripts/check_display.sh                 # X server, window manager, x11vnc, noVNC
scripts/run_clock_demo.sh                # Gazebo GUI + /clock bridge (watch it in the browser; Ctrl-C to stop)
scripts/check_clock_demo.sh              # automated GUI check: its own sim time must advance, then teardown
REEF_HEADLESS=1 scripts/check_clock_demo.sh
scripts/regress_clock_check.sh           # regression suite for the checker
scripts/test_desktop.sh                  # desktop supervisor tests on a spare display (:150)
reef-desktop status                      # desktop services; `reef-desktop logs` to follow their logs
```

Gazebo is started only by these scripts, never by the desktop services, so the
desktop is up before any simulation begins.

### X3 quadrotor scenario (data for the REEF port)

A bounded Gazebo flight (settle, ascend, forward, left, back, descend) that
records ground truth, IMU, and an idealized downward range, then checks and
plots the recording. The vehicle is flown by Gazebo's own velocity controller,
which uses **simulation truth**; this is a data source, not REEF control.
Interfaces, frames, conventions, and acceptance criteria are in
[docs/X3_SCENARIO.md](docs/X3_SCENARIO.md).

**Container terminal:**

```bash
scripts/setup_assets.py              # once: download + verify the pinned X3 model into assets/ (~22 MB)
scripts/run_x3_scenario.sh           # headless flight, recording, analysis (~80 s); prints the run directory
scripts/run_x3_scenario.sh --gui     # same, with Gazebo shown at http://127.0.0.1:8081/vnc.html
scripts/regress_x3_scenario.sh       # regression suite (~7 min)
ros2 run reef_sim analyze_x3_bag recordings/<run>   # re-analyze (after: source install/setup.bash)
```

The same from the **Mac terminal**: `docker compose exec dev scripts/setup_assets.py`,
then `docker compose exec dev scripts/run_x3_scenario.sh`.

Each run writes `recordings/x3_<time>_<id>/`: `manifest.yaml` (source revision,
model version and checksums, parameters, seeds, outcome), `bag/` (rosbag2,
MCAP), and `analysis/*.png`. On the Mac the same files are under
`~/ros2_ws/reef_ros2/recordings/`. Runs are offline: they fail if Gazebo
fetches anything from Fuel. For replay, see docs/X3_SCENARIO.md §8.

With `--gui`, the browser desktop shows a *Gazebo Sim* window. The world is
`x3_flight`, the entity tree lists `x3`, and a small quadrotor climbs to about
2 m and flies a square; scroll to zoom in.

Third-party asset: X3 UAV model by Open Robotics (Carlos Agüero, Cole
Biesemeyer), Gazebo Fuel, CC BY 4.0. `src/reef_sim/models/reef_x3` is derived
from it (see `src/reef_sim/assets/x3_uav_v4.json`).

### Daily use

**Mac terminal:**

```bash
cd ~/ros2_ws/reef_ros2
docker compose up -d          # start (the desktop starts with it)
docker compose exec dev bash  # open a container terminal (or use VS Code)
docker compose stop           # stop at the end of the day
```

`docker compose restart dev` restarts the container, and the desktop comes back
by itself. Stale X locks from an unclean stop are cleared automatically. Desktop
service logs are in `/var/log/reef-desktop/` (**container terminal**:
`reef-desktop logs`), and the supervisor's own messages are in
`docker compose logs dev` (**Mac terminal**).

### What persists across container replacement

`docker compose down` followed by `up`, or a rebuild after changing the
Dockerfile, replaces the container.

| Kept | Where |
|---|---|
| Everything under `~/ros2_ws/reef_ros2`: source, docs, `build/`, `install/`, `log/` (these three are ignored by Git) | Bind mount on the Mac |
| Project-owned Gazebo models and worlds (`src/reef_sim/models`, `src/reef_sim/worlds`, `sim/worlds`) | Bind mount, in Git |
| Pinned third-party Gazebo models (`assets/models/`, e.g. the X3 UAV, ~93 MB), installed by `scripts/setup_assets.py`, checksummed by the committed manifest | Bind mount, ignored by Git |
| Scenario recordings (`recordings/`) | Bind mount, ignored by Git |
| Ad-hoc Gazebo Fuel downloads and Gazebo GUI settings, `/root/.gz` (the X3 scenario does not use this cache) | Docker volume `reef_ros2_gz` |

| Lost | Why |
|---|---|
| Anything else under `/root` (`~/.ros` logs, shell history, `~/.fluxbox`) and packages installed by hand | Container filesystem |
| `/var/log/reef-desktop/` | Container filesystem |

`docker compose down --volumes` also deletes the `reef_ros2_gz` volume, so the
next Fuel models would be downloaded again.

### Returning to the original container

The original is independent and can run at the same time. **Mac terminal:**

```bash
docker compose stop                          # optional: stop the dev container
docker start ros2_novnc_container
docker exec -it ros2_novnc_container bash    # then, in that container terminal:  ~/start_vnc.sh
```

Browser: http://localhost:8080/vnc.html. The original setup is documented in
[docs/setup/GazeboMacDockerSetup.md](docs/setup/GazeboMacDockerSetup.md).
To remove the dev container entirely (**Mac terminal**):
`docker compose down --volumes && docker image rm reef_ros2_dev:jazzy`.

### Differences from the original container

| Area | Original `ros2_novnc_container` | Dev container | Why |
|---|---|---|---|
| Base image | `ros:jazzy` as of ~2026-09-09 (exact digest unknown) | `ros:jazzy@sha256:c3706ef0…` (amd64 `efbc8cb2…`, created 2026-09-16) | Pinned and reproducible; the original digest cannot be recovered from inside that container |
| Package versions | Installed 2026-09-29 | Current ROS/Ubuntu repo versions at build time, recorded in `/etc/reef-image-packages.txt` | The ROS apt repo keeps only current versions; compare with `docker/original-packages.txt` |
| Extra packages | none | `shellcheck`; `x11-utils`, `procps`, `util-linux`, `curl` listed explicitly | The scripts call them (the last four were already present in the original, via recommends or the base image) |
| Window manager | Fluxbox installed, **not running** | Fluxbox started after Xvfb answers; verified, and restarted if it dies | The original starts fluxbox before Xvfb is ready. That ordering failed 3 of 5 times in testing |
| Desktop start | `~/start_vnc.sh` by hand, each session | `reef-desktop run` as the container's main process | Survives restarts; no duplicates |
| noVNC port | 8080 in the container, published as `-p 8080:8080` (all host interfaces by default) | 6080 in the container, published on `127.0.0.1:8081` only | Keeps 8080 for the original; not reachable from the network |
| x11vnc | All interfaces in the container, `-bg` | `-localhost`, supervised in the foreground | Only websockify needs it |
| Environment | `~/.bashrc` exports, and sources `/root/ros2_ws/install` (an unrelated overlay) | Image `ENV DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3`. `/etc/bash.bashrc` sources `/opt/ros/jazzy` only | Documented and overlay-free |
| Mount | `~/ros2_ws` | `~/ros2_ws/reef_ros2` only | The project path is the same; the unrelated workspace is not visible |
| Gazebo cache | Container-local `~/.gz` | Volume `reef_ros2_gz` | Survives container replacement |
| PID 1 | `bash` | `tini` (`init: true`) | Reaps orphaned processes |

To compare package versions after a build (**Mac terminal**):

```bash
docker compose exec -T dev cat /etc/reef-image-packages.txt > /tmp/reef-new-packages.txt
diff <(grep -v '^#' docker/original-packages.txt) /tmp/reef-new-packages.txt | less
```

## Environment knobs

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
| `REEF_TEST_REGISTER_DELAY` | unset | test-only: pauses `check_clock_demo.sh` inside its startup window |
| `REEF_REQUIRE_WM` | `1` in the dev container, `0` in the original | `check_display.sh`: whether a missing window manager fails |
| `REEF_TEST_DESKTOP_DISPLAY`, `_VNC_PORT`, `_WEB_PORT` | `150`, `5950`, `6150` | `test_desktop.sh` spare display and ports |
| `REEF_X3_PARAMS` | `src/reef_sim/config/x3_scenario.yaml` | `run_x3_scenario.sh` parameters file |
| `REEF_X3_OUT` | `recordings/x3_<time>_<id>` | `run_x3_scenario.sh` run directory |
| `REEF_ASSETS_DIR` | `assets/models` | asset location (`setup_assets.py`, `run_x3_scenario.sh`) |
| `REEF_X3_ENABLE_RANGE` | `1` | test-only: `0` omits the range stream |

`check_clock_demo.sh` isolates itself with a per-run Gazebo partition and ROS
topic, and fails if its own launch dies. Exit codes are 0 pass, 1 clock check
failed, 2 demo failed, 124 timeout, 130/143 interrupted. See
[docs/MIGRATION.md §5](docs/MIGRATION.md#5-sim-time-bridge-demonstration-v).

To recreate the reference clones, see the commit table in
[docs/MIGRATION.md](docs/MIGRATION.md#2-reference-sources-inspection-only-never-built).

## Building

`scripts/run_x3_scenario.sh` builds `reef_sim` itself. To build by hand,
**container terminal**, from this directory (in the original container, not
from `/root/ros2_ws`, whose colcon run would also discover `reef_ros2/src`):

```bash
cd /root/ros2_ws/reef_ros2
env -i HOME=$HOME PATH=/usr/bin:/bin bash --norc -c \
  'source /opt/ros/jazzy/setup.bash && colcon build --base-paths src'
```

## Using the original container

The scripts also work in `ros2_novnc_container` (**Mac terminal**:
`docker exec -it ros2_novnc_container bash`, then use the container commands
above from `/root/ros2_ws/reef_ros2`). There, `check_display.sh --start` runs
`/root/start_vnc.sh`, and a missing Fluxbox is reported as a warning.
