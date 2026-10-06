# REEF ROS 2: project status

Updated 2026-10-06 (P09 H11 PASS; manifest-label fix). `main` contains P00–P07, P07b,
the R1 fixes, and the R2 documentation corrections, all merged by
fast-forward at the user's request (P07: §5i, P07b: §5j, R2: §5k).
Section 3 reconciles
[docs/handoff/](handoff/) (conversation-derived history) with the repository
and the recorded evidence.

**Evidence labels:** **USER**: reported by the user. **REVIEW**: a pasted
independent Codex review of a named revision. **VERIFIED**: run in this
project by the implementer, with logs or manifests in the repository tree.
**PLANNED**: prompts or plans only.

## 1. Revisions and branches

| Branch | Head | Contents | Merged to `main`? |
|---|---|---|---|
| `main` | see `git log -1 main` | starter, checker fixes, dev container, P00–P07, P07b, R1 fixes, R2 documentation corrections | — |
| `feature/x3-sim-dataset`, `p00-status-and-wrappers` | merged | P01, P00 | yes (fast-forward, 2026-09-30) |
| `p02-baseline` | `04c9b19` | P02: reference harness, fixtures, independent check, baseline decision, `reef_check.sh baseline` | yes (fast-forward, 2026-09-30) |
| `p03-msgs-interfaces` | `65bd779` | P03: `reef_msgs`, vendored `rosflight_msgs`, parameter contract, interface contract | yes (fast-forward, 2026-09-30; USER: `interfaces` passed in the dev container) |
| `p04-vertical-estimator` | `b291ca3` | P04: vertical estimator | yes (fast-forward, 2026-09-30; USER: `estimator` passed in the dev container) |
| `p05-horizontal-estimator` | `49f7073` | P05: combined estimator, opt-in C1, horizontal fixtures, `baseline`/`estimator`/`faults` targets, R1 packet | yes (fast-forward, 2026-09-30; USER: checks behaved as expected in the dev container) |
| `r1-fixes` | `1ee999b` | R1 decisions: finding 1 (publish point), C1 default, published-message parity, doc fixes | yes (fast-forward, 2026-09-30; USER: `baseline`, `faults` passed in the dev container) |
| `p06-controller` | `0f2aa63` | P06: control-chain spec, `reef_control` port (history imported), reference harness, fixtures, model, `control` target, dry-run sink | yes (fast-forward, 2026-10-01; USER: `control` 106/106 and `interfaces` passed in the dev container) |
| `p07b-faults-position` | merged | P07b: closed-loop fault scenarios (test hooks), position mode with idealized mocap, K9/K10, `faults` target extension, R2 packet, R2 doc corrections | yes (fast-forward, 2026-10-01, after R2) |
| `p07-closed-loop` | merged | P07: criteria, `reef_fc_standin`, closed-loop world/launch/runner/analyzer, recorded runs, `control` closed-loop steps, `reef_demo.sh closed-loop` | yes (fast-forward, 2026-10-01; USER: headless `control` passed in the dev container) |

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
| REEF messages, estimator, controller, RGB-D, hardware | not established | **done** in simulation: messages and helpers (P03), estimator (P04, P05), controller and closed loop (P06, P07, P07b), RGB-D (P08); all merged. Hardware **not started**: P10 is blocked until the target hardware is known | §4; reviews/P03–P08 |
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
| P05 combined estimator, faults | done, merged (`49f7073`). F11 failed until R1 (legacy D1); passes with C1, the default since R1 | `reef_check.sh baseline|estimator|faults`; [ACCEPTANCE.md §4d, §5](ACCEPTANCE.md), [reviews/P05.md](reviews/P05.md) |
| R1 independent review | **SELF-REVIEW at `49f7073` (§5f)**, not independent confirmation. USER decisions (2026-09-30): fix finding 1; approve C1 (default on); defer C2–C6; keep the vibration assumption; fix the stale docs (finding 5). Applied on `r1-fixes` (§5g) | [reviews/R1.md](reviews/R1.md) |
| P06 controller port and command interface | done, merged (`0f2aa63`): faithful bit-exact port of `reef_control` `12237b76` (USER), dry-run sink, stand-in design. USER: K1–K13 kept for the baseline; `is_flying` unconnected and the hardware throttle check deferred to P10 approved | `reef_check.sh control`; [CONTROL_CHAIN.md](CONTROL_CHAIN.md), [ACCEPTANCE.md §5 control](ACCEPTANCE.md), [INTERFACES.md §4](INTERFACES.md), [reviews/P06.md](reviews/P06.md) |
| P07 REEF closed loop (stand-in low-level loop) | done, merged: REEF estimator + controller fly the X3 through the stand-in (development tool) on idealized inputs; nominal run 22/22, causality +0.30 m | `reef_check.sh control`, `reef_demo.sh closed-loop`; [ACCEPTANCE §5 control P07](ACCEPTANCE.md), [CONTROL_CHAIN §7](CONTROL_CHAIN.md), [reviews/P07.md](reviews/P07.md) |
| P07b closed-loop faults and position mode | done, merged: 11 scenarios (estimate dropout short/long, estimator reset, controller restart, stale setpoint, range and velocity loss, pause, stand-in exit, position square with K9, `face_target` K10); crashes documented where the legacy system has no protection (USER) | `reef_check.sh faults`; [ACCEPTANCE §5 P07b](ACCEPTANCE.md), [reviews/P07b.md](reviews/P07b.md) |
| R2 independent review | **PASS (SELF-REVIEW)**, USER 2026-10-01: an independent Claude session reviewed `p07b-faults-position` (code `bb47cf0`); minor documentation corrections applied (§5k) | [reviews/R2.md](reviews/R2.md), [reviews/R2_packet.md](reviews/R2_packet.md) |
| P08 RGB-D | **done, merged** (USER 2026-10-02: H12 dev-container `vision` PASS; cleared to merge): `rgbd_to_velocity` ported bit-exact (53/53); camera interface; replacement OpenCV odometry; open loop 20/20; faults 24/24; closed loop on vision 17/17 (staleness p99 14 ms after the `/clock` fixes, USER option 2); capability matrix; `reef_demo.sh vision` | `reef_check.sh vision` PASS (§5l); [VISION.md](VISION.md), [reviews/P08.md](reviews/P08.md) |
| P09 simulation release | **every gate PASS on `p09-release`, H11 included** (gates at `132c27c`, H11 at `09b9a5e`, manifest fix `e743179`; §5n): release vision profile PASS; fresh-clone offline reproduction PASS; `faults` PASS; CI 6/6; code quality (0 warnings, ASan/UBSan clean). Licence: public open source (MIT, Hunter Swan; ports MIT UF REEF AVL) | `reef_check.sh release`; [RELEASE_SIMULATION.md](RELEASE_SIMULATION.md), [reviews/P09.md](reviews/P09.md) |
| P10–P13 hardware | blocked: target hardware unknown | |

## 5. Checks run for P00 (original container, 2026-09-29, base `9af00d2`)

All VERIFIED; logs are under `log/checks/reef_check_*` (ignored by Git).
Details are in [reviews/P00.md](reviews/P00.md).

| Command | Exit | Result |
|---|---|---|
| `reef_check.sh help`, `reef_demo.sh help` | 0 | usage |
| `reef_check.sh` / `reef_demo.sh` with no argument, unknown target or mode, or invalid option | 2 | usage, error |
| `reef_check.sh baseline\|estimator\|faults\|control\|vision\|release` | 2 | NOT IMPLEMENTED at P00 (all implemented since: §4) |
| `reef_demo.sh estimator\|closed-loop\|vision` | 2 | NOT IMPLEMENTED at P00 (all implemented since: §4) |
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
with the shared tree deliberately left stale. The dev-container rerun was
pending at the time (H5; since done, §7).

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

USER, dev container, at `8bac5a5`: `reef_check.sh estimator` **FAIL**. Its
log (`log/checks/reef_check_estimator_20260930_152027`, shared mount) showed
a single failed assertion: estimate age p99 22 ms (limit 20 ms). [V] Stage
breakdown of that run: Python adapter IMU path p99 20 ms, max 50 ms;
estimator stage p99 4 ms. Fix (`3edbd95`): the IMU path moved to a C++ node
(`reef_x3_adapter/x3_imu_adapter`), and the estimator reads `/x3/range`
directly. The limit was not changed. Rerun here after the fix: `estimator`
PASS 6/6 (age p99 8 ms, adapter stage p99 4 ms), `faults` 35/36 (F11 only),
`interfaces` PASS, `regress_x3_scenario.sh` 13/13. The dev-container rerun
was pending at the time (H7; since done, §7).

Nothing in P05 was run inside `reef_ros2_dev` by the implementer.

## 5f. R1 review: Claude self-review (dev container, 2026-09-30, `49f7073`)

**Label: SELF-REVIEW.** This was a fresh Claude session (Opus 5.5, `claude-opus-5-5`) using the R1
prompt. It shares the implementer's model and may share its blind spots, so it is
not the independent R1 confirmation that
[handoff/REEF_TESTING_AND_REVIEW.md](handoff/REEF_TESTING_AND_REVIEW.md) asks for.
It does not authorize flight. The reviewer edited no source; the only change is this STATUS entry.

| Check (run by the reviewer) | Exit | Result |
|---|---|---|
| originals in `c80f824`/`d8e3096` vs `reference/` at `e4179f48`/`7fb63ff9` (sha256) | 0 | identical (12 files) |
| `baseline/tools/source_diff.sh <scratch>` | 0 | math files: include lines only |
| reference rebuilt (`REF_TAG=r1 build_reference.sh master`), port rebuilt; h05 ref vs port | 0 | 6720 events, all 79 shared columns byte-identical; D1 visible (estimate pinned near 0.196 m/s while truth reaches 0.40) |
| `test_horizontal_core` | 0 | 7/7; expected values from Eigen AngleAxis and f = C(a − g), not from the code |
| F1, F2 (`check_faults.f1/f2`, fixtures) | 0 | 5/5 |
| fresh `run_x3_scenario.sh --estimator`, then F11/F12/determinism | 1 | **F11 baseline FAIL** 0.506 m/s RMS (0 of 2629 later observations accepted); F11 C1 PASS 0.015; F12, determinism PASS |

Findings (not fixed):
1. **Publish order differs from master** (`sensor_manager.cpp:242-249`).
   The node publishes after `checkTakeoffState`, but master published before
   it (BASELINE_DECISION §4.5 step 8). At a landing step, `xyz_estimate`
   carries z = z_x0 and ż = 0 instead of the pre-landing estimate (h08,
   t = 11.922 s: master ≈ −0.236 m, +0.34 m/s). At takeoff and landing, the debug P and σ
   carry P0_flying/P0. The parity checks cannot see this, because harness and
   port both record the state after the callback.
2. D1 makes the filter about 3× overconfident even without dropouts (F1: P_vx
   7.3e−4 vs 2.3e−3 with C1). F11 shows a permanent lock-out.
3. No case covers a NaN, zero, or negative velocity-message covariance or
   observation, which master's gate accepts (NaN D²), or a NaN attitude (D8).
4. Covariance parameters are checked only for symmetry and diagonal ≥ 0, not
   for positive semidefiniteness (`matrix_operation.h:138-155`).
5. Stale docs: this file's §1 (`main` is `49f7073` and contains P05), and
   INTERFACES §1 (`faults` "not implemented", `estimator` "vertical only") and
   §3.7 ("no health topic").
6. The vibration assumption is applied for the whole run, including on the
   ground. In simulation the takeoff detector therefore reduces to the
   range ≥ 0.25 m test. Gazebo's default gravity (9.8) against REEF's 9.81 is
   the likely source of the ≈ 0.01 m/s² z bias (D3).

USER decisions on these findings are recorded in §5g and
[reviews/R1.md](reviews/R1.md). Reviewer's recommendations: C1 approve; C6 revise before P07 (or
make the vibration depend on thrust); C4 decide before P07; C2, C3, and C5
defer (small effect on the declared simulation inputs); keep the vibration
assumption, labelled, until C6 is decided. Still needed: a genuinely
independent R1 (a different model or a human), and a decision on finding 1.

## 5g. R1 fixes (original container, 2026-09-30, branch `r1-fixes`)

USER decisions after the §5f self-review: fix finding 1; approve C1 as the
default; defer C2–C6; keep the 1.0 m/s² vibration assumption; fix the stale
docs of finding 5. Evidence: [reviews/R1.md](reviews/R1.md). [V] All checks
below ran at `527df13` (code and criteria committed; docs pending).

| Command | Exit | Result |
|---|---|---|
| colcon test `reef_estimator` | 0 | 75/75, including `EstimateIsTakenBeforeTheTakeoffCheck` |
| `baseline/tools/check_port.py` | 0 | PASS 253/253, 1005 s: parity 50/50 (C1 off), wrapper 50/50, **published messages 50/50 bit-identical to the original's** (harness A6), **c1 50/50** (independent model = original with C1 off; port default = model with C1) |
| `scripts/reef_check.sh faults` | 0 | **PASS 36/36**, 379 s. F11 default (C1 on): 0 fusions during the dropout, P_vx 1.2e−4 → 2.5, error after 0.015 m/s RMS (limit 0.10). F11 legacy (C1 off, characterization): 1249 re-fusions, locked out, 0.515 m/s |
| `scripts/reef_check.sh interfaces` | 0 | PASS, 120 s |
| `scripts/reef_check.sh estimator` | 0 | PASS 6/6, 442 s: altitude RMSE 6.7 mm, ż 0.037, vx 0.0108, vy 0.0111 m/s (now with C1 on) |
| `scripts/reef_check.sh baseline` | 0 | PASS, 1654 s: P02 reference 36/36, port parity 253/253 |

Skipped: `regress_x3_scenario.sh` (no `reef_sim` or X3 script changed);
shellcheck (no shell script changed). Nothing was run in `reef_ros2_dev`
by the implementer (H8).

## 5h. Checks run for P06 (original container, 2026-10-01, branch `p06-controller`)

| Command | Exit | Result |
|---|---|---|
| `scripts/reef_check.sh control` (at `12b093f` plus the uncommitted `reef_check.sh` target, `check_colcon.py` package set, docs, and a comment-only restore in `controller.cpp`; all committed right after) | 0 | **PASS**, 503 s. colcon build + test of all packages (reef_control 20 cases); own X3 + REEF run and offline replay; `check_control.py` **106/106**: reference build from checksummed pinned sources; fixture lock; 21 streams (20 fixtures + 11,595-event stream from the run) with port core == original and node == original on every one of 142 columns (node: all but the command stamp, which equals the estimate stamp); independent model 0 difference on every stream; determinism; K13 and missing-parameter cases; `Gains.cfg` tables; K1–K12; negative control (D-term sign flipped: model and port comparison both fail). Closed loop: N/A (P07) |
| colcon test `reef_control`, `reef_msgs` | 0 | 20 and 55 cases (launch test: exit codes 1/2/0, live commands, sink trace with offboard timeout) |
| `shellcheck -x` on `reef_check.sh`, `build_control_reference.sh`, `fetch_sources.sh`, `source_diff.sh` | 0 | no findings |

Found while testing (my expectations corrected, code unchanged): a 100 ns
stamp step runs a control step, because `1e-9 * 100` is
1.0000000000000001e−7 in double (> 1e−7); the original does the same.
Not run: `interfaces`, `estimator`, `baseline`, `faults` (the estimator is
unchanged; `reef_msgs` gained two messages and `get_yaw`, covered by the
colcon step above). Nothing was run in `reef_ros2_dev` by the implementer (H9).

## 5i. Checks run for P07 (original container, 2026-10-01, branch `p07-closed-loop`)

| Command | Exit | Result |
|---|---|---|
| `scripts/reef_check.sh control` (at `85c1108`, after the end-of-run fix; uncommitted: docs only) | 0 | **PASS**, 725 s: P06 106/106; nominal run with return, landing, and disarm: every judged P07 check including the end state (disarmed, motors 0, at rest on the ground); estimate age p99 18–20 ms (at the limit; item 24); causality drop 0.301 m |
| `scripts/reef_check.sh control` (at `14ebb2b`; uncommitted: docs only) | 0 | **PASS**, 587 s: colcon build + test (reef_control 20, reef_fc_standin 5 cases); P06 fidelity 106/106; closed-loop nominal run: every P07 criterion (takeoff 4.1 s after arming, tilt ≤ 0.09 rad, altitude hold RMSE ≤ 0.016 m, velocity RMSE ≤ 0.009 m/s, yaw rate 0.262 rad/s, no saturation, estimate age p99 12 ms); causality run: +0.30 m range bias lowers the truth hover by 0.301 m |
| same, at `9fa655e` | 1 | the closed-loop steps passed, but the estimate-stream run also applied the P05 analysis, whose estimate-age limit failed narrowly (p99 20.0 ms), so no stream file existed and `check_control.py` crashed. Fixed in `14ebb2b` (stream run without the P05 analysis; missing stream = FAIL). No limit changed |
| recorded runs `recordings/p07_*` | — | see [reviews/P07.md](reviews/P07.md): shipped gains 20/22 (K2 windup in the first hover); X3 gains A 21/22 (data-path test bug), B 21/22 (latency p99 24 ms), C 22/22; causality 0.300 m |
| motor-mapping experiment (stand-in alone, owned session) | — | roll, pitch, yaw-rate signs as designed |
| end-of-run fly-away (USER, GUI demo) and the stand-in kill test (owned session) | — | cause verified: ROS nodes stop 5 s+ before Gazebo, motor model holds the last speeds; after the fix the stand-in zeroes the motors on shutdown (vehicle falls and rests) and the scenario lands and disarms first (`p07_cl_land_a`: back to 0.018 m from the start, touchdown 0.75 m/s) |
| `scripts/regress_x3_scenario.sh` | 0 | 13/13 |
| `scripts/regress_clock_check.sh` (after the `env.sh` change) | 0 | all cases PASS |
| colcon test `reef_fc_standin`; pytest `test_closed_loop_world.py` | 0 | 5 + 2 cases |
| `shellcheck -x` on `run_x3_scenario.sh`, `reef_check.sh`, `reef_demo.sh`, `env.sh` | 0 | no findings |

Not run: `reef_demo.sh closed-loop --gui` (it wraps the same run with the
GUI viewer; H10), `reef_check.sh estimator|faults|baseline|interfaces`
(estimator unchanged; the package set is covered by the colcon step).
Nothing was run in `reef_ros2_dev` by the implementer (H10).

## 5j. Checks run for P07b (original container, 2026-10-01, branch `p07b-faults-position`)

| Command | Exit | Result |
|---|---|---|
| `scripts/reef_check.sh faults` (at `bb47cf0`; uncommitted: docs only) | 0 | **PASS**, 1667 s: F1–F12 36/36; all 11 closed-loop scenarios PASS (every judged check and characterization; see reviews/P07b.md) |
| scenario runs `recordings/p07b_*` | — | first pass: 3 PASS, 8 FAIL from evaluation bugs (fixed, `203a59e`) and three wrong characterizations (corrected with USER approval, `384bec6`); after re-scoring and one rerun (`pause_resume`): all PASS |
| `scripts/reef_check.sh control` (at `bb47cf0`; uncommitted: docs only) | 0 | PASS, 660 s: P06 106/106; nominal P07 criteria (estimate age p99 12 ms); causality drop 0.300 m |
| `scripts/regress_x3_scenario.sh`, `scripts/regress_clock_check.sh` (after the P07b changes to the launch, scripts, and `env.sh`) | 0, 0 | all cases PASS |
| pytest `test_adapter.py` (mocap pose) | 0 | 9 cases |
| `shellcheck -x` on `run_x3_scenario.sh`, `reef_check.sh`, `env.sh` | 0 | no findings |

Not run: `reef_check.sh estimator`, `baseline`, `interfaces` (the IMU
adapter changed only by a test hook that is off by default; the estimator
is unchanged); GUI demos (informational, USER); nothing in `reef_ros2_dev`
by the implementer (the P07b faults check, §7).

## 5k. R2 review (SELF-REVIEW) and documentation corrections (2026-10-01)

USER: the independent R2 review (a Claude session, labelled SELF-REVIEW)
is complete with result **PASS**; it asked for minor documentation
corrections only. Applied: stale lines in STATUS (`main` contents, branch
and milestone rows) and INTERFACES §4 (the controller is in the X3 loop since
P07); the latency margin stated precisely (ACCEPTANCE P07 note, item 24);
the firmware `MIN_THROTTLE` / `MOTOR_IDLE_THR` semantics that the stand-in
does not model (CONTROL_CHAIN §7, INTERFACES §4.3). [V] The reviewer's
`reef_check.sh control` run (`log/checks/reef_check_control_20261001_062923`)
passed: P06 106/106, nominal closed loop, causality. Findings record:
[reviews/R2.md](reviews/R2.md). No code changed.

## 5l. Checks run for P08 so far (original container, 2026-10-01, branch `p08-rgbd`)

Container commands, headless, idle machine. VERIFIED.

| Check | Exit | Result |
|---|---|---|
| `scripts/reef_check.sh vision` (log `log/checks/reef_check_vision_20261001_193029`, working tree on `f34e043`) | 0 | PASS: colcon (175 test cases; `reef_rgbd_odometry` 4 gtest, `reef_sim` 24 pytest); `rgbd_to_velocity` 53/53; vision assets; **vision flight 20/20** (camera interface, replacement odometry open loop, REEF on vision, weak texture, depth loss); **faults run 24/24** (delay 200 ms, drop 1 s; hooks verified acting); closed loop on vision N/A |
| `scripts/run_x3_scenario.sh --vision` (`recordings/p08_vision_open1`, odometry rev. 0) | 1 | 17/20: weak-texture REEF recovery, depth-loss resume (+1.14 s), depth-loss REEF recovery FAIL. Led to odometry rev. 1 (VISION.md §6 revision note) |
| same, rev. 1 (`recordings/p08_vision_open2`; `recordings/p08_vision_faults2` with `REEF_X3_VISION_FAULTS=1`) | 0, 0 | 20/20, 24/24 (VISION.md §8) |
| `shellcheck -x` on `run_x3_scenario.sh`, `reef_check.sh`, `env.sh` | 0 | no findings |
| `scripts/regress_x3_scenario.sh`, `scripts/regress_clock_check.sh` | 0, 0 | all cases PASS (interruption, ownership, replay, display services untouched) |
| `scripts/reef_check.sh vision` with the closed-loop step (log `log/checks/reef_check_vision_20261001_212841`, `c8b6a57` + working tree) | **1** | FAIL: everything as above PASS (20/20, 24/24); **closed loop on vision 16/17: staleness p99 24 ms (limit 20)**. Five closed-loop runs: p99 32, 30, 20, 20, 24 ms (VISION.md §8.1) |
| `scripts/regress_x3_scenario.sh`, `scripts/regress_clock_check.sh` (after the closed-loop and demo changes) | 0, 0 | all cases PASS |
| `scripts/reef_check.sh control` (`log/checks/reef_check_control_20261001_215352`), `scripts/reef_check.sh faults` (`log/checks/reef_check_faults_20261001_220500`), after the closed-loop launch and analyzer changes | 0, 0 | PASS: P06 parity, P07 nominal and causality; F1–F12 and all 11 P07b scenarios (unchanged behaviour of the P07/P07b paths) |
| USER-approved fixes (2026-10-02): orphaned `reef_estimator_node` pid 1081415 (an F9 case from P05 development, namespace `/faults_f9`) terminated with SIGTERM after checking it; `check_faults.py` F9 now runs each case in its own session and stops that session on timeout (`ros2 run` orphaned the node); `closed_loop_runner` reads sim time from truth stamps instead of `/clock`; `camera_check` stops following sim time after its check | — | timeout path tested: exit 124, no leftover process |
| `scripts/reef_check.sh vision` (`log/checks/reef_check_vision_20261001_235509`, `df61b3a` + these fixes) | **0** | **PASS**: colcon, 53/53, assets, vision flight 20/20, faults run 24/24, **closed loop on vision 17/17, staleness p99 14 ms** (0.03 % over 20 ms) |
| `scripts/reef_check.sh control` (`log/checks/reef_check_control_20261002_000956`), `faults` (`log/checks/reef_check_faults_20261002_002206`), after the runner and F9 changes | 0, 0 | PASS: P06 parity, P07 nominal and causality; F1–F12 (F9 3/3) and all 11 P07b scenarios |
| `scripts/regress_x3_scenario.sh`, `scripts/regress_clock_check.sh` (after these changes) | 0, 0 | all cases PASS |

Key numbers (reef_check run): vision velocity RMSE x 0.0004, y 0.0013 m/s;
REEF on vision x 0.028, y 0.033 m/s; loss shown −0.90 s (weak texture) and
−0.21 s (depth); resumes −6.90 s and +0.72 s; REEF recovered ≤ 0.042 m/s.
USER (2026-10-01): both odometry behaviours outside the judged items are
kept as documented characterizations V1 (near-zero first sample after a
recovery) and V2 (intermittent wrong pose just before a depth loss), with no
suppression logic (VISION.md §8).

## 5m. Capability matrix (P08)

The full matrix is in [INTERFACES.md §5](INTERFACES.md).
- **Supported in simulation:** IMU, RGB-D velocity (Gazebo camera →
  replacement odometry → ported `rgbd_to_velocity`), controller velocity
  mode, scenario setpoints.
- **Supported with idealized input:** attitude, range, mocap velocity,
  controller position mode and `face_target`.
- **Port tested only:** mocap z, RC override switch.
- **Unsupported/Deferred:** VRPN mocap and joystick teleop (USER); the Astra
  driver and `demo_rgbd` (replaced); `setpoint_generator` and `dubins_path`;
  ROSflight firmware and hardware output (P10).

## 5n. H11 release reproduction and the manifest fix (2026-10-05/06)

**H11** (USER, Mac, fresh clone and `--no-cache` image of `09b9a5e`, core
profile, offline): `scripts/reef_check.sh release` **exit 0**,
`release_summary.json` verdict PASS, 0 uncommitted paths, all 8 steps PASS
(`log/checks/reef_check_release_20261005_211208` in the H11 clone;
`log/h11_release_summary.json` and `log/h11_demo_report.txt` here). Closed-loop nominal 26/26, estimate age p99 10 ms;
causality 19/19, p99 10 ms. `reef_demo.sh closed-loop` 26/26, p99 14 ms.
Image package list `8a574d1a79f7…`, identical to the 2026-10-02 image.
REPORTED: the run took 6 h 15 min, all of the excess in `baseline`
(20032 s vs 1429 s on 2026-10-02) while `control` at the end was normal
(614 s); a loaded host overnight, which the deterministic parity checks do
not depend on. The documented ~65 min is not a promise on a busy Mac.

**Attempt 1** (2026-10-02, `7b42ed9`) failed only the closed-loop nominal run
(`sim time stalled at 59.090 s`). Its run directory has now been inspected
(`log/h11_attempt1_nominal_result.json`; full logs kept outside Git):
- [V] every topic ran at its nominal sim-time rate to the end (`/clock`
  498/s, `/x3/imu` 249.6/s, `/x3/truth/odom` 99.6/s), with phases at ~real
  time: no degradation, an abrupt stop.
- [V] the runner was not scheduled for ~136 s: the last log line and the
  stall error are 156 s apart in wall time, against a 20 s timeout.
- [V] the bag ends at sim 59.436 s while the runner gave up at 59.090 s, so
  unprocessed samples were queued — the precondition `stall.py` needs.
- [V] Gazebo needed SIGTERM escalation after SIGINT while the other nine
  processes exited cleanly: starved, not crashed.
- [A] cause: host load froze the container's VM. Sim time never advanced
  past 59.436 s afterwards, so it is not proven that this run would have
  completed; the fix (`23376fd`) turns a certain false failure into a
  re-check, and a genuinely wedged simulation still fails 20 s later.

**Manifest fix** (`e743179`, found by the USER while reading `manifest.yaml`
for H11 step 4): `model.controller` claimed the stock truth-fed controller in
closed-loop runs. Checks after the fix, original container:

| Check | Exit | Result |
|---|---|---|
| `scripts/regress_x3_scenario.sh` (full, GUI case included) | 0 | 13/13 PASS (`log/checks/regress_x3_20261006_041439`, repeated with the exit code recorded) |
| `scripts/reef_check.sh control` | 0 | 5/5 assertions; closed loop 26/26 and 19/19 (`log/checks/reef_check_control_20261006_043047`); both closed-loop manifests name the REEF controller and the stand-in |
| `reef_sim` pytest | 0 | 31/31 (28 before; `test/test_manifest.py` adds 3) |
| `scripts/reef_check.sh faults` (USER decision: historical evidence on the baseline commit) | 0 | PASS, 27 min, on `5af39af`, 0 uncommitted paths: F1–F12 36/36; the 11 closed-loop scenarios 87 judged (81 acceptance + 6 characterizations) + 3 REPORTED; 163 PASS lines, 0 FAIL (`log/checks/reef_check_faults_20261006_044758`) |

Not re-run for the fix: the release gate itself. The delta from the H11
commit is one manifest string and one test file, and the gate's own runs
produce the corrected label.

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
8. **License of `reef_msgs`: resolved** (USER 2026-10-02): the bundle's MIT
   license covers it. The repository itself has no license file
   (`src/reef_msgs/LICENSE_NOTE.md`, NOTICE.md).
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
16. (resolved at R1) F11 failed with master's D1 lock-out; C1 is approved
    and on by default. `correction_c1_clear_xy_flag: false` restores master.
19. R1 findings 3, 4 and 6 (NaN or non-positive velocity covariance, PSD
    checks on covariance parameters, vibration on the ground) are open; they
    belong with C2–C6 decisions. A genuinely independent R1 (a different
    model or a human) is still recommended before flight work.
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
20. **License of `reef_control`: resolved** (USER 2026-10-02): MIT, added
    upstream in `43cdee8` (2020), applies to the ported `12237b76`
    (`src/reef_control/LICENSE_NOTE.md`).
21. Controller legacy behaviour K1–K13 (CONTROL_CHAIN.md §5) is kept by
    USER decision, notably K1 (D term anti-damping), K2 (no effective
    anti-windup), K5 (no output inhibition), K6 (NaN latch), K9 (no heading
    wrap). Candidates for a later, separately approved correction list.
22. [A] ROSflight 2.x firmware semantics of `Command.u[3]` in mode 2 are
    unverified; the hardware command contract is P10.
23. The controller's `is_flying` input is unconnected, as in the legacy
    launch files (the estimator publishes `is_flying_reef`); integrators run
    whenever armed. **Approved by the USER (2026-10-01).**
24. **Timing margins on this host are thin.** P07 estimate age at the motor
    command (sim time, 2 ms resolution), p99 over 19 headless runs: 12–24 ms
    (median 12; 15 runs 12–16 ms; one at 20 ms, passing; one at 24 ms,
    failing); GUI runs 20 and 24 ms (limit 20 ms, ACCEPTANCE P07 note). The P05 estimate age (stock run,
    `reef_check.sh estimator` criterion) was p99 8 ms at P05 and 20.0 ms
    (FAIL, narrowly) in one run on 2026-10-01. No limit was changed. Options
    if it recurs: the stand-in steps on command arrival as well as on the
    gyro (it now waits up to 4 ms for the next gyro sample); a C++ IMU
    noise node (the Python node is on the latency path). **USER
    (2026-10-01): not pursued; official acceptance is headless on an idle
    machine, GUI runs are informational.**
27. P07b findings (characterized, kept by USER decision: no new
    failsafes): no estimate or setpoint freshness check (a 6 s stale
    setpoint keeps the vehicle moving); any command gap > 100 ms drops the
    vehicle (a controller restart takes 3.7 s); without velocity
    observations the controller follows REEF's drifting estimate (2.7 m in
    10 s); without range, altitude error 0.27 m in 10 s; `face_target` makes
    the heading wander inside the dead zone (K10). Position mode uses an
    IDEALIZED (truth) mocap pose.
28. **R2 follow-ups (code): deferred to P09.5** (USER, 2026-10-06; the
    baseline is frozen as verified). The items are: compare `node_id` in the P06 parity;
    legacy helper vectors for `get_yaw`; assert K11's "differentiators never
    reset"; report the K10 heading error at 6 s again; report the three
    always-pass characterization items as REPORTED instead of counting them
    as checks; make "fault injected" require the hook's own subscription
    (not the recorder's); throw on int32 duration overflow. R2 verdict: P10
    must re-verify firmware `u[3]`, `MIN_THROTTLE`/RC override, failsafe
    (0.3), the nonlinear throttle map with `wI`, the real latency budget,
    K4 with wall-clock stamps, and gains on the vehicle; R3 is required
    before any flight.
25. The closed-loop controller gains are `reef_control_x3_sim.yaml` (the
    shipped quad gains with `dI` = 0, explained in that file and in
    reviews/P07.md); with the shipped gains the first hover fails the
    altitude-hold criterion because of K2.
26. The stand-in uses truth attitude and rates, linear thrust, and no
    attitude integrators; it is not ROSflight (CONTROL_CHAIN.md §7).
29. `src/reef_sim/worlds/x3_closed_loop_vision.sdf` is **not well-formed
    XML**: its generated header comment contains `--headless-rendering`, and
    `--` is illegal inside an XML comment, so `ElementTree.parse` fails at
    line 4. Gazebo's TinyXML2 accepts it, which is why no check has failed.
    The text comes from `scripts/make_vision_assets.py`; fixing it
    regenerates both vision worlds and changes their SHA-256s, so it is
    deferred past the tag (`src/reef_sim/test/test_manifest.py` strips
    comments before parsing rather than depending on the defect).
    **Deferred to P09.5** (USER, 2026-10-06): fixing it before the tag would
    invalidate the SHA-256 values the release check recorded.
30. (resolved, `e743179`) `manifest.yaml` named the stock truth-fed
    controller in closed-loop runs.

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
- **H7 (P05):** done. USER: the checks behaved as expected; merged.
- **H8 (R1 fixes):** done. USER: `baseline` and `faults` passed in the dev
  container.
- **H9 (P06):** done. USER: `control` 106/106 and `interfaces` passed in the
  dev container; merged.
- **H10 (P07):** done. USER: headless `reef_check.sh control` passed on an
  idle machine in the dev container; the GUI demo returns, lands, and
  disarms. A GUI run under other load failed only the latency check (p99
  24 ms; analysed: CPU contention). USER: GUI runs are informational,
  headless runs are official; no latency rework. Merged.
- **P07b faults check (dev container; formerly listed here as H11, renamed
  because H11 is the P09 release reproduction in the testing guide):**
  `scripts/reef_check.sh faults` (expect PASS: F1–F12 36/36 and 11
  closed-loop scenarios, about 30 min, headless, idle machine). Not yet run
  in the dev container (R2 relied on the implementer's run). It is not part
  of either release profile; recommended once before the tag.
- **H11 (P09, Mac + fresh image):** **done, PASS** (USER, 2026-10-05/06,
  `09b9a5e`): `reef_check.sh release` exit 0, verdict PASS, 0 uncommitted
  paths, closed loop 26/26 and 19/19. Details, the 6 h 15 min duration and
  the attempt-1 analysis are in §5n. The first attempt (2026-10-02,
  `7b42ed9`, transcript `log/h11_release_output.txt`) failed only the
  closed-loop nominal run (`sim time stalled at 59.090 s`); the fix is
  `reef_sim/stall.py` (`23376fd`), which confirms a stall only after draining
  pending input, and the archived run directory
  (`log/h11_attempt1_nominal_result.json`) shows the queued samples the
  drain needs.
- **H11b (P09, Mac, about 5 min):** demo re-check of the manifest-label fix
  `e743179` in the H11 container: `git pull --ff-only` in the H11 clone, then
  `scripts/reef_demo.sh closed-loop`, and confirm `manifest.yaml` names the
  REEF controller and the stand-in. The release gate is not re-run (§5n).
- **H3:** open the three plots and `manifest.yaml` of a recent
  `recordings/x3_*` run, and check them against
  [X3_SCENARIO.md](X3_SCENARIO.md). (`feature/x3-sim-dataset` is already
  merged.)
- **H12 (P08, dev container):** done. USER (2026-10-02): `scripts/reef_check.sh
  vision` passed in the dev container. Watch `scripts/reef_demo.sh vision
  --closed-loop --gui` (informational): the weak-texture segment can end with
  the vehicle drifting into the wall (VISION.md §8.1).

## 8. Next milestone

**P09**: every gate passed on `p09-release`, **H11 included** (gates at
`132c27c`, H11 at `09b9a5e`; §5n, RELEASE_SIMULATION.md §4). One item is
outstanding before the tag: a short demo re-check of the manifest-label fix
`e743179` in the H11 container (`reef_demo.sh closed-loop`, about 5 min; the
6 h 15 min gate stays valid at `09b9a5e`). Then: merge into `main` and apply
the annotated tag `sim-baseline-v0.1.0`.

USER decisions for the tag (2026-10-06), all in the direction of freezing the
baseline exactly as verified: `reef_check.sh faults` is run once before the
tag as historical evidence on the baseline commit (§5n); the seven R2 code
follow-ups (item 28) and the invalid XML in the generated vision world
(item 29) are both deferred to P09.5, so that no SHA-256 recorded by the
release check is invalidated.

**After P09 (USER, 2026-10-02):**

1. Finish P09 as planned: strict bit-exact parity, every legacy behaviour
   kept. The approvals to retain legacy behaviour applied to this baseline
   only.
2. Tag the checkpoint `sim-baseline-v0.1.0` (annotated), with no permanent
   legacy branch. The release notes state the exact legacy configuration
   (RELEASE_SIMULATION.md §1b).
3. **P09.5 Modernization** on `main`: fix the documented legacy defects
   systematically (controller K1–K13; also the estimator's D-items and the
   `rgbd_to_velocity` Q-items where they fall under a phase).
   - **Phase 1, safety:** invalid and NaN input handling, covariance
     validation, controller output inhibition, stale estimate and setpoint
     handling.
   - **Phase 2, control logic:** D-term anti-damping (K1), ineffective
     anti-windup (K2), heading wrapping (K9), differentiator and reset
     behaviour (K3, K4, K6, K11).
   - **Also in P09.5 scope** (USER, 2026-10-06): the seven R2 code
     follow-ups (item 28) and the invalid XML in the generated vision
     worlds (item 29), both deferred from P09 to keep the tagged baseline
     exactly as verified.
4. **Testing methodology for corrections.** Never overwrite the legacy golden
   outputs or reference tests; they stay as historical evidence that the
   original behaviour is reproduced. Each correction gets:
   - a reproducing case;
   - a justified expected result;
   - a focused regression test, separate from the legacy tests;
   - a before/after comparison.

   AGENTS.md (which still says K1–K13 and the D-items stay) is to be updated
   for P09.5 when it starts.

**P08** done and merged (USER 2026-10-02). Accepted by USER:
- the remaining `/clock` overhead of `imu_noise` and `range_sensor`, with no
  rewrite of P07 code;
- the `rgbd_to_velocity` license stays flagged internally. **Superseded**
  (USER 2026-10-02): it is MIT, the upstream repository's own license, and
  the project is intended for public open-source distribution (NOTICE.md).

Next: **P09** (simulation release). Plan and criteria are to be presented for
USER approval before any P09 code.
