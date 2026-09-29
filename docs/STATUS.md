# REEF ROS 2: project status

Updated 2026-09-29 (P00 reconciliation). Base revision `9af00d2` on branch
`feature/x3-sim-dataset`; the P00 changes are on branch
`p00-status-and-wrappers`, which starts from it. Reconciles
[docs/handoff/](handoff/) (conversation-derived history) with the repository
and the recorded evidence.

**Evidence labels:** **USER**: reported by the user. **REVIEW**: a pasted
independent Codex review of a named revision. **VERIFIED**: run in this
project by the implementer, with logs or manifests in the repository tree.
**PLANNED**: prompts or plans only.

## 1. Revisions and branches

| Branch | Head | Contents | Merged to `main`? |
|---|---|---|---|
| `main` | `4161a4d` | starter project, checker fixes, dev container | — |
| `feature/x3-sim-dataset` | `9af00d2` | P01: X3 scenario, sensors, recordings, analysis, and later fixes | **no** (awaiting user acceptance, H3) |
| `p00-status-and-wrappers` | this work | P00: status/acceptance/interface docs, `reef_check.sh`, `reef_demo.sh`, reef_sim unit tests | no |

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
| P00 reconcile and wrappers | **done** (this change) | human checks H0–H2 below |
| P01 quadrotor, sensors, recordings | **implemented; awaiting H3 acceptance and merge** | `reef_check.sh sim-data`, `reef_demo.sh stock` |
| P02 baseline decision and reference tests | not started (**next**) | `reef_check.sh baseline` → NOT IMPLEMENTED |
| P03–P05 messages, estimator, replay/faults | not started | |
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

## 6. Open items and known limits

1. `sim/launch/clock_demo.launch.py` still uses Gazebo's combined GUI mode,
   which has the `wait_gui` startup race found in P01 (MIGRATION §10). It has
   passed every run so far, but should get the `-s` + `-g` split.
2. The clock regression suite has not yet run inside the dev container (H2).
3. The dev image's built-in `reef-desktop` predates the 5 s probe timeout;
   rebuild when convenient.
4. No independent review of the Dockerfile or of P01 has been performed.
5. `feature/x3-sim-dataset` and this branch are unmerged.
6. `check_clock_demo.sh` and `scripts/sim_lib.sh` duplicate session logic
   (reviewed code left unchanged).
7. The only unit tests are for reef_sim geometry; the ROS nodes are covered
   by the scenario checks, not unit tests.

## 7. Human checks still needed

- **H0:** read this file; confirm the branch and commit (`git log -1`).
- **H1 (dev container):** `scripts/reef_check.sh env`. Expect PASS, with the
  environment line showing the dev image hash.
- **H2 (dev container):** `scripts/reef_check.sh clock --gui --regress`. Expect
  PASS with 28/28, and your browser desktop still working afterwards.
- **H3:** open the three plots and `manifest.yaml` of a recent
  `recordings/x3_*` run, and check them against
  [X3_SCENARIO.md](X3_SCENARIO.md). Then decide whether to merge
  `feature/x3-sim-dataset` (and this branch) into `main`.

## 8. Next milestone

**P02**: choose and document the estimator baseline (master `e4179f48` vs
simulation `95987b51`) with independent reference tests. Acceptance criteria
are drafted in [ACCEPTANCE.md §4](ACCEPTANCE.md#4-future-targets-criteria-to-be-fixed-before-implementation);
they must be finalized before any port output is scored.
