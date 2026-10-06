# Agent instructions: REEF ROS 2 migration

Goal: port the REEF Estimator stack from ROS 1 (catkin) to ROS 2 Jazzy, with
Gazebo Harmonic simulation. Read `docs/STATUS.md` first: it holds the current
revision, evidence, open items, and next milestone. Then read
`docs/MIGRATION.md` (source audit, pinned upstream commits, open decisions),
`docs/ACCEPTANCE.md`, and `docs/INTERFACES.md`. `docs/handoff/` is
conversation history, not proof of the repository state. Where it conflicts
with this file, this file wins.

## Milestone workflow

- Fix acceptance criteria in `docs/ACCEPTANCE.md` before scoring an
  implementation; change thresholds only in a separate, explained commit.
- Expose checks through `scripts/reef_check.sh TARGET` and demos through
  `scripts/reef_demo.sh MODE`. Wrap existing scripts rather than duplicating
  them. Exit 0 PASS, 1 executed FAIL, 2 BLOCKED / NOT IMPLEMENTED / invalid;
  unavailable targets must say NOT IMPLEMENTED.
- At the end of a milestone, update `docs/STATUS.md` (revision, checks and exit
  codes, skipped checks, human checks still needed, next milestone) and add
  brief evidence in `docs/reviews/<milestone>.md`.

## Correction workflow (P09.5, docs/CORRECTIONS.md)

- Every correction has an ID (`C*` estimator, `KC*` controller where the
  number is the K-item it fixes, `NC*` chain-level behaviour with no legacy
  counterpart), one named boolean parameter, **default true**, whose false
  value reproduces the legacy path exactly.
- Before code: a register entry (defect, failure scenario, proposed fix,
  justified expected result, test, parity impact) and acceptance criteria in
  ACCEPTANCE.md. Before merging: a reproducing case that fails with the
  toggle off and passes with it on, a focused regression test separate from
  the legacy tests, and a before/after record.
- Never weaken or overwrite a legacy test or golden file to accommodate a
  correction. The legacy suites keep asserting the legacy path with the
  toggles off; they are the evidence that the port reproduces the originals.
- One correction per commit. A correction that changes tuning (for example a
  PID sign or anti-windup fix) states the effect on the simulation gains in
  the same commit, and gains still change only in explained configuration
  commits.
- Safety corrections may not depend on a test hook, on truth, or on anything
  unavailable on hardware.

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
- When experimenting with X clients or window managers, set `HOME` to a scratch
  directory. Fluxbox writes state to `~/.fluxbox`, and an earlier experiment
  modified the original container's copy (since restored).
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

## Estimator reference rules (`baseline/`, docs/BASELINE_DECISION.md)

- The reference harness compiles the pinned original sources unmodified.
  Never patch them. Any new adaptation goes in `baseline/README.md` and the
  decision document.
- Never use port output as its own reference. Golden files change only via
  `check_baseline.py --update-golden "<reason>"`, and only with an explained
  source or algorithm decision (the reason is stored in `golden/index.json`).
- Legacy defects (D1–D10) stay **in the reference**, which is historical
  evidence and never changes. What changes is the port.
- **P09.5 (USER, 2026-10-06): corrections are the default.** The faithful
  baseline is frozen at the tag `sim-baseline-v0.1.0`; on `main` the shipped
  default configuration is the safest corrected version, because the goal is
  physical flight. Legacy behaviour is reachable only by explicit toggles,
  and those are used by the parity runs. Each correction still needs its own
  USER decision, its own commit, a register entry in
  [docs/CORRECTIONS.md](docs/CORRECTIONS.md) and its own tests; a correction
  that is not in the register and approved is not implemented.
- The IMU vibration assumption stays (USER, 2026-09-30) until C6 is decided.
- Port tolerances are fixed in ACCEPTANCE.md §4; do not loosen them to make a
  port pass.

## Message, helper, and parameter rules (P03, docs/INTERFACES.md §3)

- `src/third_party/` holds unmodified upstream copies (pin in `UPSTREAM.json`,
  checked by `scripts/check_vendor.py`). Never edit them; re-vendor instead.
- `src/reef_msgs/test/data/legacy_helper_vectors.txt` comes from the pinned
  legacy code only (`baseline/helper_vectors.sh`); update it only with
  `--update "<reason>"`.
- Keep numerical code free of ROS (`reef_msgs_helpers`, `reef_estimator_core`);
  ROS types belong in the node and adapter layers.
- Invalid parameters are errors at startup, never silently replaced.

## Estimator port rules (P04, docs/INTERFACES.md §3)

- **With every correction toggled off, the port must stay bit-identical to
  the reference**: `baseline/tools/check_port.py` (via `reef_check.sh
  baseline`) after any change to `src/reef_estimator` or `reef_msgs`
  helpers. That parity is the historical evidence and may never be traded
  away; a correction that cannot be switched off is not acceptable.
  Physical plausibility is `reef_check.sh estimator`; fault behaviour is
  `reef_check.sh faults`.
- The default configuration (corrections on) is checked against the
  independent model, not against master. C1 (`correction_c1_clear_xy_flag`)
  is the pattern every later correction follows: one named parameter,
  default true, false reproducing the legacy path.
- C2–C6 and the later corrections stay unimplemented until each is approved
  and registered (docs/CORRECTIONS.md).
- Messages must carry the state at the original's publish point (before the
  takeoff check; R1 finding 1); `check_port.py` compares them with what the
  original published.
- Keep the estimator state on the executor thread. Callbacks that other
  threads may run (clock-jump handlers, time sources) only set flags.
- Simulation inputs to REEF that come from truth (the attitude and the
  simulated velocity observations) are labelled idealized in topics, reports,
  and plots. Never describe the velocity observations as RGB-D odometry. The IMU vibration overlay is a
  scenario assumption, not a sensor model; do not tune it to pass limits.
- Scripts that build C++ packages use the per-environment tree
  (`scripts/colcon_tree.py`); the shared `build/` belongs to manual builds.

## Controller port rules (P06, docs/CONTROL_CHAIN.md, INTERFACES.md §4)

- The tagged baseline (`sim-baseline-v0.1.0`) is a strictly faithful,
  bit-exact port of `12237b76` (USER, 2026-09-30). On `main`, P09.5 corrects
  K-items under the same rules as the estimator: corrections default on,
  each with a named toggle whose off state reproduces `12237b76` bit for
  bit, a register entry and its own tests. K-items that have no approved
  correction yet stay and stay asserted.
- After any change to `src/reef_control`, run `reef_check.sh control`
  (original in `baseline/control` vs port, core and node, independent model).
  Never patch the pinned sources; harness adaptations go in
  `baseline/README.md` (C1–C3).
- The low-level loop for simulation is a labelled stand-in (development
  tool, CONTROL_CHAIN.md §7), never described as ROSflight or hardware
  evidence. No hardware output exists; the dry-run sink refuses
  `hardware:=true` until P10.
- Closed loop (P07): after changing `src/reef_fc_standin`, the closed-loop
  world, launch, runner, analyzer, or controller configuration, run
  `reef_check.sh control` (nominal and causality runs). Controller gains for
  simulation change only in explained configuration commits
  (`reef_control_x3_sim.yaml`); never feed truth to the controller or the
  estimator beyond the documented idealized inputs.
- Test hooks (P07b) are labelled, default off, and enabled only by the
  scenario overlays in `src/reef_sim/config/closed_loop/`; never enable them
  in nominal runs or demos, and never add hooks to the controller or
  estimator cores.
- Safety behaviour that the legacy system lacks (freshness checks, output
  inhibition, non-finite containment) is **in scope from P09.5 phase 1** and
  belongs behind a registered correction, not in a test hook. Until the
  correction that covers a case is approved, that case's documented outcome
  stands, including the crashes in the dropout and stand-in-exit scenarios.
  The legacy outcome must stay reproducible with the toggles off.

## Simulation data rules (`src/reef_sim`, docs/X3_SCENARIO.md)

- Keep **truth** (`/x3/truth/...`) separate from **measurements** (`/x3/imu`,
  `/x3/range`). Anything derived from truth must say so in its topic
  documentation. Never put truth into a measurement field (for example IMU
  orientation).
- The stock Gazebo velocity controller flies on truth. Do not describe
  scenario results as REEF-in-the-loop or as hardware validation.
- Third-party models: pin them in `src/reef_sim/assets/*.json` (URL, version,
  license, SHA-256) and install them with `scripts/setup_assets.py` into
  `assets/` (ignored by Git). Runs must stay offline; `run_x3_scenario.sh`
  enforces this.
- Add noise in ROS with generators keyed by (seed, stamp), not with Gazebo
  sensor noise, which is not reproducible (see MIGRATION.md §10).
- Run the Gazebo server with `-s`. If a GUI is needed, start it as a separate
  `gz sim -g` viewer.
- After changing `reef_sim` or the X3 scripts, run `scripts/regress_x3_scenario.sh`.

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
