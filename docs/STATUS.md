# REEF ROS 2: project status

Updated 2026-09-30 (P05 complete on its branch). `main` = `b291ca3`, which
includes P00–P04, all merged by fast-forward at the user's request. P05
(combined estimator, faults, R1 packet) is on branch `p05-horizontal-estimator`. Section 3 reconciles
[docs/handoff/](handoff/) (conversation-derived history) with the repository
and the recorded evidence.

**Evidence labels:** **USER**: reported by the user. **REVIEW**: a pasted
independent Codex review of a named revision. **VERIFIED**: run in this
project by the implementer, with logs or manifests in the repository tree.
**PLANNED**: prompts or plans only.

## 1. Revisions and branches

| Branch | Head | Contents | Merged to `main`? |
|---|---|---|---|
| `main` | `b291ca3` | starter, checker fixes, dev container, P00–P04 | — |
| `feature/x3-sim-dataset`, `p00-status-and-wrappers` | merged | P01, P00 | yes (fast-forward, 2026-09-30) |
| `p02-baseline` | `04c9b19` | P02: reference harness, fixtures, independent check, baseline decision, `reef_check.sh baseline` | yes (fast-forward, 2026-09-30) |
| `p03-msgs-interfaces` | `65bd779` | P03: `reef_msgs`, vendored `rosflight_msgs`, parameter contract, interface contract | yes (fast-forward, 2026-09-30; USER: `interfaces` passed in the dev container) |
| `p04-vertical-estimator` | `b291ca3` | P04: vertical estimator | yes (fast-forward, 2026-09-30; USER: `estimator` passed in the dev container) |
| `p05-horizontal-estimator` | this work | P05: combined estimator, opt-in C1, horizontal fixtures, `baseline`/`estimator`/`faults` targets, R1 packet | no |

No Git remote, pull request, or tag exists; the repository is local. Upstream
references are pinned in [MIGRATION.md §2](MIGRATION.md): `reef_estimator`
`e4179f48c3f26e22bd1366b71ee1e117ce2f5f7f` (master) and the simulation-bundle
pin `95987b5118b624208910d9e51424300022e1f512`.

## 2. Environments

| | Original `ros2_novnc_container` | Dev container `reef_ros2_dev` |
|---|---|---|
| Status | preserved and unmodified, except a Fluxbox state file changed by an early test and restored (MIGRATION §9) | in use by the user |
| Definition | manual (`docs/setup/GazeboMacDockerSetup.md`) | `Dockerfile` (last changed `396ab55`), `compose.yaml` and `.devcontainer/` (`317ed67`) |
| Base image | `ros:jazzy` of about 2026-09-09 (digest unknown) | `ros:jazzy@sha256:c3706ef0…` (amd64 `efbc8cb2…`) |
| Browser desktop | http://localhost:8080/vnc.html | http://127.0.0.1:8081/vnc.html (→ 6080 inside) |
| Project mount | `~/ros2_ws` → `/root/ros2_ws` | `~/ros2_ws/reef_ros2` → `/root/ros2_ws/reef_ros2`; volume `reef_ros2_gz` → `/root/.gz` |
| Window manager | Fluxbox installed, not running (startup race) | Fluxbox, supervised by `reef-desktop` |
| Image identity | — | package-list sha256 `3c98605d7e7d…af73` (from manifests of the user's runs since 21:41 UTC). A pre-rebuild image `e93fee7d…` was used until the user ran `docker compose up -d` |

Values in the dev-container column come from the committed files and from
manifests written inside that container (VERIFIED). The implementer has no
Docker access and cannot inspect the running container directly. The built
image predates `3345a41`, so its `/usr/local/bin/reef-desktop` still has the
2 s noVNC probe timeout. That affects only the image health check; rebuild
when convenient (no functional need).

## 3. Handoff reconciliation

| Handoff item | Handoff status | Actual status | Evidence |
|---|---|---|---|
| Starter project `243180a` | REVIEW confirmed | same | REVIEW 1 |
| Checker false pass (foreign `/clock` rescued a failed owned launch) | open; fixes not pasted | **fixed** in `3d6e779` (per-run `GZ_PARTITION` and topic, publisher identity, liveness monitoring) | REVIEW 2 (of `3d6e779`/`9f1613a`): "Resolved", reproduced with a shared domain and partition. VERIFIED: `regress_clock_check.sh` cases 4b/4c/4e/4f at `9af00d2` |
| SIGTERM left the clock observer running | open | **fixed** in `3d6e779`; startup-window leak fixed in `81ae28a`; unsafe regression fallback fixed in `5b9c96e` | REVIEW 2 (observer), REVIEW 3 (startup fix confirmed; fallback defect found, then fixed). VERIFIED: cases 5, 6, 8–13; also through `reef_check.sh clock` (§5) |
| Estimator-diff understatement, truth-fed controller omission, discovery wording, Mac/container labels, `dynamics.h` location | open (docs) | fixed in `9f1613a` | REVIEW 2: all "Resolved" |
| Dockerfile / dev container | USER: complete; manual checks passed | same. The user also ran X3 checks in it | USER; manifests with image hash `3c98605d…` |
| Independent review of Dockerfile changes | prompt supplied, no result | **none performed** | — |
| X3 quadrotor, sensors, recordings (P01) | not established | **implemented** on `feature/x3-sim-dataset`. `regress_x3_scenario.sh` 13/13 in the dev container at `9af00d2`; user saw the drone fly in the browser | USER; VERIFIED (manifests `log/checks/regress_x3_20260929_223324/`) |
| P01 human check H3 (plots, manifest inspection) | — | **partly done**: user watched the GUI flight; plots and manifest not yet reported as reviewed | USER |
| REEF messages, estimator, controller, RGB-D, hardware | not established | not started | — |
| Replacement name and port | "must be read from files" | `reef_ros2_dev`, `127.0.0.1:8081` | `compose.yaml` |
| 95987b51 full SHA | to resolve | `95987b5118b624208910d9e51424300022e1f512` | MIGRATION §2 |

Where the handoff and AGENTS.md differ, AGENTS.md stays authoritative. One
convention differs on purpose: the older scripts use exit 2 for "owned
simulation failed". `reef_check.sh` and `reef_demo.sh` map that to 1 at the
wrapper boundary ([INTERFACES.md §1](INTERFACES.md#1-command-interface)).

## 4. Milestones

| Milestone | Status | Notes |
|---|---|---|
| P00 reconcile and wrappers | done, merged | |
| P01 quadrotor, sensors, recordings | done, merged | human check H3 (plots) still recommended |
| P02 baseline decision and reference tests | done, merged. USER: `reef_check.sh baseline` passed in the dev container. Corrections C1–C5 **deferred to R1** (USER, 2026-09-30); the initial port must match master exactly | `reef_check.sh baseline`; [BASELINE_DECISION.md](BASELINE_DECISION.md), [reviews/P02.md](reviews/P02.md) |
| P03 messages, helpers, ROS 2 interfaces | done, merged (USER: dev-container `interfaces` PASS) | [reviews/P03.md](reviews/P03.md) |
| P04 vertical estimator and ROS 2 wrapper | done, merged (USER: dev-container `estimator` PASS) | [reviews/P04.md](reviews/P04.md) |
| P05 combined estimator, faults | **done on `p05-horizontal-estimator`**; `faults` fails F11 (legacy D1, by design until R1) | `reef_check.sh baseline|estimator|faults`; [ACCEPTANCE.md §4d, §5](ACCEPTANCE.md), [reviews/P05.md](reviews/P05.md) |
| R1 independent review | **packet ready**: [reviews/R1_packet.md](reviews/R1_packet.md) | decisions C1–C6 |
| P06–P07 controller, REEF closed loop | not started | |
| P08 RGB-D | not started | |
| P09 simulation release | not started | |
| P10–P13 hardware | blocked: target hardware unknown | |

## 5. Checks run for P00 (original container, 2026-09-29, base `9af00d2`)

All VERIFIED; logs are under `log/checks/reef_check_*` (ignored by Git).
Details are in [reviews/P00.md](reviews/P00.md).

| Command | Exit | Result |
|---|---|---|
| `reef_check.sh help`, `reef_demo.sh help` | 0 | usage |
| `reef_check.sh` / `reef_demo.sh` with no argument, unknown target or mode, or invalid option | 2 | usage, error |
| `reef_check.sh baseline\|estimator\|faults\|control\|vision\|release` | 2 | NOT IMPLEMENTED |
| `reef_demo.sh estimator\|closed-loop\|vision` | 2 | NOT IMPLEMENTED |
| `reef_check.sh env` | 0 | PASS (check_env, check_display) |
| `reef_check.sh clock` | 0 | PASS: headless sim time 0.002 → 4.998 s; negative case exited 1 as required |
| `reef_check.sh clock --gui` | 0 | PASS (headless, GUI, negative) |
| `REEF_STARTUP_TIMEOUT=0.5 reef_check.sh clock` | 1 | FAIL (executed failure mapped to 1) |
| `REEF_DISPLAY=:197 reef_check.sh clock --gui` | 2 | BLOCKED, nothing judged |
| SIGTERM to `reef_check.sh clock` mid-run | 143 | 5 tagged processes → 0 |
| SIGINT to its process group mid-run | 130 | 5 tagged processes → 0 |
| `reef_check.sh clock --regress` | 0 | PASS, including 28/28 regression cases |
| `reef_check.sh sim-data` | 0 | PASS, 46 s scenario, sim t 47.51 s at end |
| `REEF_ASSETS_DIR=<empty> reef_check.sh sim-data` | 2 | BLOCKED |
| `reef_demo.sh stock` | 0 | completed, analysis passed |
| `reef_demo.sh replay <run> --rate 4` | 0 | an observer in the printed domain received `/x3/range` |
| `colcon build/test/test-result --base-paths src` | 0 | 7 unit tests passed. Negative control: a deliberately failing test gave exit 1 in both `test` and `test-result` |
| shellcheck 0.9.0 on all scripts | 0 | no findings |

Not run in P00: `reef_check.sh sim-data --regress` through the wrapper. The
underlying suite ran 13/13 in the dev container at `9af00d2` (USER). Nothing in
P00 was run inside `reef_ros2_dev` by the implementer.

## 5b. Checks run for P02 (original container, 2026-09-30)

| Command | Exit | Result |
|---|---|---|
| `check_baseline.py --update-lock --floor --update-golden "<initial characterization>"` | 0 | 37/37 (golden and lock written once from the unmodified originals) |
| `scripts/reef_check.sh baseline` | 0 | PASS, 36/36 including the negative mutation check, 523 s |
| `reef_check.sh baseline --gui`, `env --floor`, `clock --floor` | 2 | invalid option |
| shellcheck (scripts, baseline), hadolint (Dockerfile) | 0 | no findings |

## 5c. Checks run for P03 (original container, 2026-09-30)

| Command | Exit | Result |
|---|---|---|
| `scripts/reef_check.sh interfaces` | 0 | PASS 5/5: vendored pin, tampered-copy negative (exit 1 as required), legacy helper vectors reproduce, colcon build + test (89 test cases: `reef_msgs` 49, `reef_estimator` 33, `reef_sim` 7; 0 failures/errors/skips), P02 golden unchanged |
| `reef_check.sh interfaces --gui` | 2 | invalid option |
| mutation checks (reverted) | 1 | Eigen `toRotationMatrix().transpose()` and column-major import in `reef_msgs`; off-by-one channel range and a changed config value in `reef_estimator`; an injected failing test made `check_colcon.py` exit 1 |
| `rosdep check --from-paths src --ignore-src` | 0 | all system dependencies satisfied |
| shellcheck -x (scripts, baseline), hadolint (Dockerfile) | 0 | no findings |

USER, dev container, at `7963c84`: `reef_check.sh interfaces` **FAIL** (exit 1)
in the colcon step: `No rule to make target '/opt/ros/jazzy/lib/libfastcdr.so.2.2.7'`.
Cause [V]: the check reused the shared `build/reef_msgs`, which the implementer
had configured in the original container (fastcdr 2.2.7 there, not in the dev
image). Fix: `check_colcon.py` now builds in `build/colcon_check/<env>/`
(keyed by a hash of the installed package list), and the stale
`build/`/`install/` outputs of the three new packages were removed. After the
fix, in the original container: PASS 5/5 (tree `build/colcon_check/bfcf57e680be`),
with the shared tree deliberately left stale. The dev-container rerun is
pending (H5).

Not rerun: `reef_check.sh baseline`, because no file it uses changed since
`04c9b19` (only the new `baseline/helper_vectors.*` were added). Nothing in
P03 was run inside `reef_ros2_dev` by the implementer.

## 5d. Checks run for P04 (original container, 2026-09-30)

| Command | Exit | Result |
|---|---|---|
| `scripts/reef_check.sh estimator` (at `7edaeb6`) | 0 | PASS 6/6 + 1 N/A, 495 s: colcon build + tests (reef_msgs 49, reef_estimator 58, reef_sim 12 cases), vertical fidelity 40/40 streams bit-identical with wrapper equivalence and 16 named case groups, X3 + REEF vs truth (takeoff 7.38 s, altitude RMSE 6.7 mm, ż RMSE 0.037 m/s), offline and ROS replays, foreign-`/clock` negative; horizontal N/A (NOT IMPLEMENTED) |
| `reef_check.sh estimator --gui` | 2 | invalid option |
| `reef_demo.sh estimator --offline <run>` | 0 | deterministic replay and plots |
| `scripts/regress_x3_scenario.sh` (at `7edaeb6`) | 0 | 13/13 cases as expected |
| `scripts/reef_check.sh sim-data` | 0 | PASS; it first FAILED (exit 2) once the implementer's stale shared builds were removed: `reef_sim` now depends on `reef_estimator`, which only the shared tree lacked. Fixed in `7edaeb6` (scenario always builds in the per-environment tree) |
| mutation / discrimination checks | — | port vs simulation revision 2.08e9 × tolerance; the jump-reset test found a real threading defect (fixed: reset now applied on the executor thread) |
| shellcheck -x, hadolint | 0 | no findings |

Nothing in P04 was run inside `reef_ros2_dev` by the implementer. The first
dev-container run builds reef_msgs, rosflight_msgs, and reef_estimator in that
container's own tree (a few minutes).

## 5e. Checks run for P05 (original container, 2026-09-30)

| Command | Exit | Result |
|---|---|---|
| `scripts/reef_check.sh baseline` (at `17c0351`) | 0 | PASS: P02 reference + port parity on 50 streams (every value bit-identical), 805 s |
| `scripts/reef_check.sh estimator` (at `172dd6f`) | 0 | PASS 6/6: 130 test cases; simulation altitude 6.8 mm, ż 0.037, vx 0.0105, vy 0.0109 m/s RMSE; recorded-stream parity; replays. The first run at `17c0351` failed one launch-test case (discovery race in the test; fixed) |
| `scripts/reef_check.sh faults` (at `17c0351`) | 1 | FAIL 35/36: **F11 baseline** (D1 lock-out after a velocity dropout: 0.51 m/s RMS; C1 on: 0.015 m/s). Left failing for R1 |
| `scripts/reef_check.sh interfaces` | 0 | PASS |
| `scripts/regress_x3_scenario.sh` | 0 | 13/13 |
| shellcheck -x, mutation checks | — | clean; a transpose mutation fails the frame tests; C1 differs from the reference |

Nothing in P05 was run inside `reef_ros2_dev` by the implementer.

## 6. Open items and known limits

1. `sim/launch/clock_demo.launch.py` still uses Gazebo's combined GUI mode,
   which has the `wait_gui` startup race found in P01 (MIGRATION §10). It has
   passed every run so far, but should get the `-s` + `-g` split.
2. The clock regression suite has not yet run inside the dev container (H2).
3. The dev image's built-in `reef-desktop` predates the 5 s probe timeout;
   rebuild when convenient.
4. No independent review of the Dockerfile or of P01 has been performed.
5. (resolved) `p02-baseline` was merged on 2026-09-30.
6. `check_clock_demo.sh` and `scripts/sim_lib.sh` duplicate session logic
   (reviewed code left unchanged).
7. The reef_sim ROS nodes are covered by the scenario checks, not unit tests.
8. **License of `reef_msgs`** (USER): upstream declares `TODO` and has no
   license file; the bundle that pins it is MIT. Confirm before publishing.
9. The P04 history import of `reef_estimator` would carry a third-party PDF
   (`docs/Partial_Update.pdf`); decide how to handle it in P04.
10. The Dockerfile now declares the P03 build dependencies. They are expected
    in the current image, so a rebuild is not needed unless
    `reef_check.sh interfaces` reports a missing package there.
11. Hardware note for P10+: a MAVLink unused-channel value (`UINT16_MAX`)
    reads as "RC switch on" (INTERFACES.md §3.5).
12. **IMU vibration assumption: kept** (USER, 2026-09-30). The simulation
    runs keep 1.0 m/s² of per-axis vibration so the ported estimator stays
    bit-identical to master; the takeoff detector itself is correction
    candidate **C6** for R1 (BASELINE_DECISION.md §7).
13. (resolved in P05) Horizontal filter ported.
16. **F11 fails with the approved baseline** (D1 lock-out). R1 decision on
    C1 (opt-in implementation ready, default off).
17. ROS 2 params-file merge silently drops a type-changing override
    (INTERFACES §3.6); keep override files type-consistent.
18. No ROS 2 producers exist for mocap velocity or RGB-D velocity
    (INTERFACES §3.10); the simulated velocity observations are idealized
    and are not RGB-D odometry.
14. The `reef_estimator` history was imported with `docs/Partial_Update.pdf`
    removed (rewritten commits; the imported tip `dd21f7a6` equals
    `e4179f48` minus that file).
15. The attitude input in simulation is truth (idealized); no attitude
    estimator exists.

## 7. Human checks still needed

- **H0:** read this file; confirm the branch and commit (`git log -1`).
- **H1 (dev container):** `scripts/reef_check.sh env`. Expect PASS, with the
  environment line showing the dev image hash.
- **H2 (dev container):** `scripts/reef_check.sh clock --gui --regress`. Expect
  PASS with 28/28, and your browser desktop still working afterwards.
- **H4 (P02):** done. USER: `reef_check.sh baseline` passed in the dev
  container; C1–C5 deferred to R1.
- **H5 (P03):** done. USER: `reef_check.sh interfaces` passed in the dev
  container after the build-tree fix.
- **H6 (P04):** done. USER: `reef_check.sh estimator` passed in the dev
  container; vibration assumption kept; takeoff detector added as C6.
- **H7 (P05, dev container):** `scripts/reef_check.sh baseline` (expect
  PASS, about 15 min), `scripts/reef_check.sh estimator` (expect PASS), and
  `scripts/reef_check.sh faults` (expect **FAIL 35/36**, only F11 baseline).
  Then decide whether to merge `p05-horizontal-estimator` and start R1 with
  [reviews/R1_packet.md](reviews/R1_packet.md).
- **H3:** open the three plots and `manifest.yaml` of a recent
  `recordings/x3_*` run, and check them against
  [X3_SCENARIO.md](X3_SCENARIO.md). (`feature/x3-sim-dataset` is already
  merged.)

## 8. Next milestone

**R1**: independent review with [reviews/R1_packet.md](reviews/R1_packet.md):
confirm parity and the approved deviations, and decide C1–C6 (C1 has the
strongest evidence: F11). Then P06–P07 (controller and REEF-in-the-loop).
