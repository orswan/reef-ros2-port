# REEF ROS 2: project status

Updated 2026-10-01 (P06 complete on its branch). `main` = `1ee999b`:
P00–P05 and the R1 fixes, all merged by fast-forward at the user's request.
P06 (controller port, dry-run sink, stand-in design) is on branch
`p06-controller` (§5h). Section 3 reconciles
[docs/handoff/](handoff/) (conversation-derived history) with the repository
and the recorded evidence.

**Evidence labels:** **USER**: reported by the user. **REVIEW**: a pasted
independent Codex review of a named revision. **VERIFIED**: run in this
project by the implementer, with logs or manifests in the repository tree.
**PLANNED**: prompts or plans only.

## 1. Revisions and branches

| Branch | Head | Contents | Merged to `main`? |
|---|---|---|---|
| `main` | see `git log -1 main` | starter, checker fixes, dev container, P00–P05, R1 fixes | — |
| `feature/x3-sim-dataset`, `p00-status-and-wrappers` | merged | P01, P00 | yes (fast-forward, 2026-09-30) |
| `p02-baseline` | `04c9b19` | P02: reference harness, fixtures, independent check, baseline decision, `reef_check.sh baseline` | yes (fast-forward, 2026-09-30) |
| `p03-msgs-interfaces` | `65bd779` | P03: `reef_msgs`, vendored `rosflight_msgs`, parameter contract, interface contract | yes (fast-forward, 2026-09-30; USER: `interfaces` passed in the dev container) |
| `p04-vertical-estimator` | `b291ca3` | P04: vertical estimator | yes (fast-forward, 2026-09-30; USER: `estimator` passed in the dev container) |
| `p05-horizontal-estimator` | `49f7073` | P05: combined estimator, opt-in C1, horizontal fixtures, `baseline`/`estimator`/`faults` targets, R1 packet | yes (fast-forward, 2026-09-30; USER: checks behaved as expected in the dev container) |
| `r1-fixes` | `1ee999b` | R1 decisions: finding 1 (publish point), C1 default, published-message parity, doc fixes | yes (fast-forward, 2026-09-30; USER: `baseline`, `faults` passed in the dev container) |
| `p06-controller` | this work | P06: control-chain spec, `reef_control` port (history imported), reference harness, fixtures, model, `control` target, dry-run sink | no |

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
| P05 combined estimator, faults | done, merged (`49f7073`). F11 failed until R1 (legacy D1); passes with C1, the default since R1 | `reef_check.sh baseline|estimator|faults`; [ACCEPTANCE.md §4d, §5](ACCEPTANCE.md), [reviews/P05.md](reviews/P05.md) |
| R1 independent review | **SELF-REVIEW at `49f7073` (§5f)**, not independent confirmation. USER decisions (2026-09-30): fix finding 1; approve C1 (default on); defer C2–C6; keep the vibration assumption; fix the stale docs (finding 5). Applied on `r1-fixes` (§5g) | [reviews/R1.md](reviews/R1.md) |
| P06 controller port and command interface | **done on `p06-controller`**: faithful bit-exact port of `reef_control` `12237b76` (USER), dry-run sink, stand-in design | `reef_check.sh control`; [CONTROL_CHAIN.md](CONTROL_CHAIN.md), [ACCEPTANCE.md §5 control](ACCEPTANCE.md), [INTERFACES.md §4](INTERFACES.md), [reviews/P06.md](reviews/P06.md) |
| P07 REEF closed loop (stand-in low-level loop) | not started; design in CONTROL_CHAIN.md §7 | |
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

USER, dev container, at `8bac5a5`: `reef_check.sh estimator` **FAIL**. Its
log (`log/checks/reef_check_estimator_20260930_152027`, shared mount) showed
a single failed assertion: estimate age p99 22 ms (limit 20 ms). [V] Stage
breakdown of that run: Python adapter IMU path p99 20 ms, max 50 ms;
estimator stage p99 4 ms. Fix (`3edbd95`): the IMU path moved to a C++ node
(`reef_x3_adapter/x3_imu_adapter`), and the estimator reads `/x3/range`
directly. The limit was not changed. Rerun here after the fix: `estimator`
PASS 6/6 (age p99 8 ms, adapter stage p99 4 ms), `faults` 35/36 (F11 only),
`interfaces` PASS, `regress_x3_scenario.sh` 13/13. The dev-container rerun
is pending (H7).

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
20. **License of `reef_control`** (USER): MIT was added upstream in
    `43cdee8` (2020), after the ported `12237b76`; applicability assumed
    [A] (`src/reef_control/LICENSE_NOTE.md`). Confirm before publishing.
21. Controller legacy behaviour K1–K13 (CONTROL_CHAIN.md §5) is kept by
    USER decision, notably K1 (D term anti-damping), K2 (no effective
    anti-windup), K5 (no output inhibition), K6 (NaN latch), K9 (no heading
    wrap). Candidates for a later, separately approved correction list.
22. [A] ROSflight 2.x firmware semantics of `Command.u[3]` in mode 2 are
    unverified; the hardware command contract is P10.
23. The controller's `is_flying` input is unconnected, as in the legacy
    launch files (the estimator publishes `is_flying_reef`); integrators run
    whenever armed.

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
- **H9 (P06, dev container):** `scripts/reef_check.sh control` (expect PASS,
  106/106, about 9 min; the first run builds `reef_control`) and
  `scripts/reef_check.sh interfaces` (expect PASS; the package set now
  includes `reef_control`). Then decide whether to merge `p06-controller`.
- **H3:** open the three plots and `manifest.yaml` of a recent
  `recordings/x3_*` run, and check them against
  [X3_SCENARIO.md](X3_SCENARIO.md). (`feature/x3-sim-dataset` is already
  merged.)

## 8. Next milestone

**P07**: REEF-driven closed loop in Gazebo Harmonic with the labelled
stand-in low-level loop (CONTROL_CHAIN.md §7): fix the P07 `control`
criteria first, then the stand-in, the X3 world without the stock velocity
controller, altitude then velocity and yaw scenarios, causality checks,
`reef_demo.sh closed-loop`.
