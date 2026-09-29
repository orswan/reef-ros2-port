# Agent instructions: REEF ROS 2 migration

Goal: port the REEF Estimator stack from ROS 1 (catkin) to ROS 2 Jazzy, with
Gazebo Harmonic simulation. Read `docs/MIGRATION.md` before starting work. It
holds the verified findings, pinned upstream commits, and open decisions.

## Environment rules

- Two containers share this project, both Ubuntu 24.04, x86_64, ROS 2 Jazzy,
  Gazebo Harmonic 8.x, display `:99`:
  - `reef_ros2_dev`, the dev container (`compose.yaml`, `Dockerfile`). Its
    desktop is started by the container (`reef-desktop run`); browser at
    http://127.0.0.1:8081/vnc.html. `command -v reef-desktop` identifies it.
  - `ros2_novnc_container`, the original. Its desktop is started by hand with
    `/root/start_vnc.sh`; browser at http://localhost:8080/vnc.html.
    **Never stop, recreate, prune, or reconfigure it**, and do not edit its
    `/root/start_vnc.sh` or `~/.bashrc`.
- No agent in these containers has Docker access. Host-side steps
  (`docker compose …`) are Mac-terminal instructions for the user; never
  report them as run.
- **Never rely on the calling shell's environment.** Agent shells do not read
  `~/.bashrc` reliably, and when they do, they inherit the unrelated overlay
  `/root/ros2_ws/install` and may have the wrong `DISPLAY`. Run everything
  through `scripts/` (which re-exec under `env -i` via `scripts/env.sh`). For
  ad-hoc commands use:
  `env -i HOME=$HOME PATH=/usr/bin:/bin bash --norc -c 'source scripts/env.sh && reef_setup_env && <cmd>'`
- Build only from `/root/ros2_ws/reef_ros2` (`colcon build --base-paths src`).
  Never run `colcon build` in `/root/ros2_ws` for this project.
- Do not modify anything outside this directory. Do not start a second
  desktop: `scripts/check_display.sh --start` waits for the dev container's
  desktop, or runs `start_vnc.sh` in the original. Test desktop changes with
  `scripts/test_desktop.sh` (spare display `:150`).
- Do not install packages into the original container. Fetch tools such as
  shellcheck into a scratch directory, or add them to the `Dockerfile`.
- Run `shellcheck -x` on changed shell scripts (`.shellcheckrc` is in the repo
  root) and review each finding. Suppress only with a commented directive.

## Source rules

- `reference/` holds upstream ROS 1 clones **for reading only**. Do not edit
  them, do not build them, and do not remove `reference/COLCON_IGNORE`. They are
  ignored by Git.
- Do not build the original ROS 1 packages as ROS 2 packages. Port them into `src/` as
  new ament packages.
- `reef_estimator_2` is not a ROS 2 port; do not use it as a base.
- Preserve the estimator's filter math and parameter semantics when porting.
  Change the ROS plumbing, not the algorithm, unless a change is explicitly
  requested.
- No physical flight recordings exist. Validate against Gazebo ground truth.
- Do not commit build outputs, reference clones, or recordings (see `.gitignore`).

## Reporting

- In docs and reports, label findings **[V] verified** (observed here) or
  **[A] assumption**. Record commands and actual output for checks. State
  blocked or skipped checks explicitly.
- Checks: `scripts/check_env.sh`, `scripts/check_display.sh`,
  `scripts/check_clock_demo.sh` (add `REEF_HEADLESS=1` when no display is needed),
  and `scripts/regress_clock_check.sh` after changing any demo/check script.
- Label commands as **container** (`/root/...` paths) or **Mac host**
  (`docker exec ros2_novnc_container ...`, browser URLs).

## Test process rules

- Tests must own what they observe: use a per-run `GZ_PARTITION` and unique
  ROS topic names, and monitor the launched processes. A ROS domain ID or
  `LOCALHOST` discovery alone is not isolation.
- Register a child's pid or session before a signal can trigger cleanup
  (record signals during startup and act on them afterwards). Include an
  interruption test inside every startup window, not only once the test is
  running.
- Before signalling any process, check that the test started it (parent/session
  ownership). Ownership of one process does not extend to its session or
  process group. Signal a whole session only if the test created that session
  itself (its own `setsid`), and otherwise signal verified pids individually
  (see `scripts/test_lib.sh`). Never signal by broad `pgrep -f` patterns: they also match the
  shell whose command line contains the pattern. Never touch the Xvfb, x11vnc,
  or websockify services.
