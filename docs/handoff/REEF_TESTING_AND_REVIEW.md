# REEF ROS 2 — human testing and economical review

Prepared 2026-09-29. Read alongside [project context](REEF_PROJECT_CONTEXT.md) and [implementation prompts](REEF_COMPLETION_PROMPTS.md).

This guide tells a human what evidence to look for as Claude implements the port. It does not assert that the listed features or commands already exist. P00 introduces the wrapper commands; later milestones implement their targets. A missing command means the corresponding milestone is incomplete.

## A practical workflow

1. Give Claude the context file, the session launch instruction, and the next incomplete milestone prompt. Avoid asking for a new overall plan every session.
2. Let Claude implement and run the relevant automated checks. Read its status report and inspect the named revision, failures, and artifacts.
3. Perform the matching human checks below. Most involve one command, the browser view, and a few plots. You do not need to inspect every source line.
4. At R1 and R2, obtain a focused independent review if available. R3 is conditional on a specific hardware integration. A colleague or a repository-capable Copilot session can substitute for Codex; their actual access and expertise determine what they can verify.
5. Send concrete failures back using the repair prompt. Repeat the failed check and relevant regressions, then continue. Do not rerun every expensive simulation for a documentation correction.

The current Docker setup has already passed your reported Mac-terminal and VS Code checks. Repeat that manual exercise when configuration changes or for the clean-install release check, rather than at every milestone.

## Where to type commands

**Mac Terminal** is for Docker and opening/attaching the project. Use the actual container name, mounts, and browser URL documented by the new configuration. The original environment used port 8080; a proposed replacement port is not evidence of its actual setting.

**The VS Code terminal attached to the development container** is for the commands below. Start in the project root:

~~~bash
cd /root/ros2_ws/reef_ros2
git status --short
git rev-parse HEAD
scripts/reef_check.sh help
scripts/reef_demo.sh help
~~~

If the new configuration deliberately changed the project path, use its documented path. Do not run container-only `/root/...` commands in a plain Mac shell.

The wrapper's intended result conventions are:

| Result | Meaning | Human action |
|---|---|---|
| PASS, exit 0 | The named assertions executed and passed | Inspect the artifacts and continue |
| FAIL, exit 1 | A check executed and found a failure | Save its output and ask for a repair |
| BLOCKED / NOT IMPLEMENTED, exit 2 | A prerequisite or implementation is missing, or invocation is invalid | Resolve the stated reason; do not count it as a pass |
| SKIPPED / no tests / stale report | No fresh evidence for that feature | Treat it as untested |

After a command, `echo $?` displays its exit code if you run it immediately. Avoid piping a check to another command unless the documented launcher preserves the original exit status. Successful launch, a green editor icon, or a process appearing in a list does not establish successful behavior.

For an explicit source build, follow the project's environment instructions and use a fresh shell loading the Jazzy base rather than an unrelated workspace. The expected build/test pattern after packages exist is:

~~~bash
cd /root/ros2_ws/reef_ros2
source /opt/ros/jazzy/setup.bash
colcon list --base-paths src
CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build --symlink-install --executor sequential --base-paths src
source install/setup.bash
colcon test --base-paths src --executor sequential --return-code-on-test-failure
colcon test-result --verbose
~~~

Run each line only after the preceding operation succeeds. An empty package list was expected at the original starter milestone; it cannot demonstrate a completed port. Use the memory-conscious build above if Docker has limited resources. Do not manually rebuild before every demo unless source/configuration changed. `colcon test` should be paired with its failure-return option and result inspection; see the [test command](https://colcon.readthedocs.io/en/released/reference/verb/test.html) and [result command](https://colcon.readthedocs.io/en/released/reference/verb/test-result.html).

## Agree on acceptance criteria before looking at results

Have Claude put the criteria in `docs/ACCEPTANCE.md` before scoring a milestone. You or an experienced controls colleague should review criteria that depend on the intended use.

Distinguish three questions:

- **Migration fidelity:** Does the ROS 2 implementation reproduce the selected original algorithm for the same ordered inputs and initialization? Compare states, covariances, gates and flags against an independently derived reference with justified numerical tolerances.
- **Estimator quality:** Does it estimate the simulated physical state accurately under stated sensor errors and operating conditions? Measure bias, RMSE, peak error, time alignment, initialization time and covariance behavior. Agreement with old code alone does not answer this.
- **Control quality:** Does the actual REEF feedback loop track bounded commands, handle saturation and stale estimates, and recover or terminate according to the defined simulation behavior? Measure tracking error, overshoot, settling, command limits and health transitions.

Every criterion should name the scenario, seed, units/frame, time basis, warm-up interval, scoring window and limit. Include timestamp age and processing delay, distinguishing wall time from simulation time. Slower-than-real-time simulation can still be valid; an arbitrary exact clock frequency is not a quality threshold. Conversely, acceptable simulation behavior does not prove the real computer can meet flight timing requirements.

There is no universal sensible position-error threshold or set of flight gains for the unspecified future vehicle. Ask Claude to propose justified *simulation* thresholds and explain their assumptions. Do not lower a failed threshold silently. A legitimate threshold correction needs a documented reason and a separately visible change.

## Human milestone checks

### H0 — reconcile what is actually present (P00)

Open `docs/STATUS.md`, inspect `git status --short`, and record the current commit. The status should distinguish the existing Docker/clock demonstration from the future estimator, controller and hardware work. Check that repairs to the original false-pass and SIGTERM cleanup findings have evidence, or remain explicitly open.

Verify that the new container uses the intended project directory and that your files survive reopening it. Do not delete the original container or its data to carry out these checks. If a report refers to an older commit, have Claude identify whether later changes affect its conclusion.

### H1 — environment and browser display (P00; repeat when environment changes)

Run `scripts/reef_check.sh env`. Confirm it reports the intended Jazzy/Gazebo versions, project environment and actual display. Run the documented GUI demonstration and open the configured noVNC URL. You should see Gazebo and be able to interact with its view. A log saying a GUI process started is insufficient evidence of browser display.

After stopping the demonstration, your browser desktop and attached VS Code terminal should remain usable. If you have already performed this on the unchanged Docker configuration, record that result rather than repeating it immediately.

### H2 — clock ownership, failure detection and cleanup (P00)

Run `scripts/reef_check.sh clock` and inspect its report. It should show advancing ROS simulation time from the test-owned simulation, bounded waits, and proper cleanup.

Ask Claude to demonstrate the regression cases from the initial review through its automated harness: another simulation running while the owned launch fails; an invalid display for a GUI-required launch; no simulator; and interruption while an observer is running. A deliberately rejected launch must fail internally, and the harness must assert that rejection. Another simulation's clock must not rescue it.

The report should identify both ROS-domain and Gazebo-partition isolation and show no owned observer/server/bridge leftovers. You need not manually kill processes: inspect the harness evidence and confirm your unrelated VNC/demo services survived. Do not use broad `pkill` commands as a cleanup test.

### H3 — vehicle motion and useful simulated measurements (P01)

Run `scripts/reef_demo.sh stock --gui`, then `scripts/reef_check.sh sim-data`. Use the documented record/analysis options if these are separate commands.

Watch the bounded scenario: hover, ascent/descent, translation and stop should match the recorded commands. Open the plots and bag manifest. Confirm nonempty IMU, range, clock and separate truth streams. Stationary IMU values should match the documented gravity/specific-force convention; do not assume a stationary accelerometer must report zero. Range should match sensor geometry and change plausibly with height and tilt.

Confirm sensor frames, timestamps and configured rates are recorded, and truth is visibly labelled. This stage uses the stock truth-fed controller to generate data. It does not demonstrate REEF estimation or REEF feedback control.

### H4 — algorithm baseline and interface contract (P02–P03)

Run `scripts/reef_check.sh baseline`. Read the baseline decision and a short comparison report. It must resolve a full source SHA and document all meaningful master/simulation differences, not just rejection switches.

The reference outputs should come from the original mathematics or independently calculated cases, with provenance. Outputs newly generated by the port and then compared with themselves provide no independent evidence. Inspect at least one stationary and one nontrivial motion case, including timing/gating or partial updates where relevant.

Read `docs/INTERFACES.md`: each measurement/state/command should have units, frames, time basis and freshness semantics. The document should identify attitude and yaw sources. A disabled RC switch in simulation must be an explicit setting; it should not silently erase future hardware functionality.

### H5 — vertical estimator (P03–P04)

Run the documented vertical scenario under `scripts/reef_demo.sh estimator --gui` and `scripts/reef_check.sh estimator` with the implemented vertical selection. The help output must give the exact selection syntax; do not guess topic or package names.

Inspect height, vertical velocity and relevant bias plots against separate truth. Confirm a stationary vehicle settles plausibly, ascent and descent have the expected signs, and tilt/range geometry is handled as specified. Inspect numerical reference comparisons and error metrics, including initialization transients. Check for finite values and appropriate covariance behavior.

An attractive plot without units, aligned times, known frames or an acceptance limit is not a completed validation. If the legacy algorithm has a real defect, expect a documented compatibility result and a separately tested correction.

### H6 — horizontal/fused estimator (P05)

Run the full estimator scenario and check. Inspect X/Y position and velocity, height, attitude inputs and error plots. Exercise motion in each horizontal direction plus a yawed or tilted case. Verify the expected frame transformation with the documented coordinate diagram and an easily recognized trajectory.

Read the sensor-source declaration. If horizontal motion depends on idealized pose/velocity or mocap-like inputs at this stage, the report must say so. Do not accept an unimplemented state filled from ground truth as a REEF estimate.

Prepare the R1 packet after H6–H7 pass. The reviewer should spend effort on source fidelity, timing, gravity, frames and independent expectations rather than prose style.

### H7 — replay, time changes and sensor faults (P05)

Run `scripts/reef_check.sh faults` and the documented `scripts/reef_demo.sh replay` invocation with an existing recording. Confirm replay uses recorded inputs and simulated time, with a manifest identifying the bag and configuration.

Inspect the result table for missing/stale inputs, duplicates, out-of-order timestamps, dropouts, rejected measurements, pause/resume and simulation reset. The specified behavior may be rejection, controlled reset or an explicit unhealthy state; silent use of invalid data is not acceptable. Check that missing required streams and broken initialization cannot produce a successful result.

Two runs with the same data should agree within the specified deterministic/tolerance contract. Do not assume unordered ROS callback arrival is inherently deterministic. Avoid simultaneous live and replay clocks in the same test scope. Fault injection here is for simulation/test processes, not an in-flight vehicle.

### H8 — controller math and actual feedback path (P06–P07)

Run `scripts/reef_check.sh control`; before a complete loop exists it should clearly report which dry-run tests are implemented and which integration tests are unavailable.

Read the controller's input/output contract and plot several bounded dry-run cases. Check command signs, units, limits, integrator behavior, required attitude/yaw sources and stale-estimate handling. Determine which component owns position/velocity control, attitude/rate control and motor actuation.

Inspect the automated simulation-only causality test: changing or removing the REEF estimate should affect the controller as specified. The stock Gazebo outer velocity controller must not continue stabilizing the vehicle from true world state underneath an alleged REEF outer loop. Ground truth may be used for scoring and physics; every other use in feedback must be declared and justified.

### H9 — REEF closed-loop simulation (P07)

Run `scripts/reef_demo.sh closed-loop --gui` and the complete control check. Watch bounded takeoff/hover, height changes, horizontal motion and stop/landing where implemented. Open reference-versus-estimated-versus-true-state plots and actuator/health plots.

Confirm the executed control path, configuration, low-level controller/firmware version and limits match the report. Only one system should integrate vehicle physics, and each control layer should have a declared owner. If an interim low-level surrogate is used, label the result accordingly; it is not evidence of ROSflight firmware integration.

Inspect dropout, saturation and restart behavior from the automated simulation tests. Acceptance means the predefined tracking and failure-response criteria passed, not merely that the drone stayed visible. Run R2 at this boundary. Preserve the accepted scenario/configuration for later regressions.

### H10 — RGB-D and any other declared sensor pipelines (P08)

Run `scripts/reef_demo.sh vision --gui` and `scripts/reef_check.sh vision` if rendered RGB-D is part of the intended release. Inspect actual image/depth samples, camera calibration, frame transforms, sensor timestamps and the generated odometry/velocity fed to REEF.

Compare a textured scene with reduced visual information, image/depth dropouts and the documented depth limits. Inspect tracking/estimation health and recovery. Ensure a failed visual pipeline does not silently substitute truth-derived velocity. Synthetic noisy truth is a useful earlier test input but does not validate image processing.

Record wall processing rate, simulation rate and sensor age on the Intel Mac. If software rendering cannot sustain the chosen workload, lower resolution/rate explicitly or test on suitable compute; do not count an unrun vision pipeline as complete. Revisit R2 with a targeted review if this integration changes closed-loop or failure behavior. Apply comparable checks to any declared mocap, teleoperation or additional input path.

### H11 — reproducible simulation release (P09)

Run `scripts/reef_check.sh release`. Review the capability matrix and the exact revision/configuration it covers.

Follow the new-user instructions from a separate clone and a freshly built development image when practical. Do not destroy the working environment. Test initial dependency/asset retrieval, then a documented offline simulation run after setup. A container filesystem copied from the development machine is not equivalent to reproducing the Dockerfile. Confirm nested meshes/textures and calibration files are available.

Run one representative demo and its scorer using only the documented steps. Confirm a second person could find logs, interpret failure and reproduce the result. Verify source/dependency versions, licensing notices, requirements, limitations and recording manifests are included. A simulation release can be complete while hardware work is explicitly deferred.

### H12 — actual hardware definition and review (P10)

With the REEF team, fill in the real vehicle/flight-controller board, firmware/version, companion computer/OS/architecture, sensors, transport, frame conventions, RC/mode behavior and command semantics. Use current documentation for that exact system. A ROS message definition compiling is not evidence that firmware accepts its values in the intended mode.

Review the hardware contract, installation/rollback instructions, watchdog design and lab commissioning plan. Use R3 before enabling real control. If these details or the responsible local operator are unavailable, mark hardware integration BLOCKED and continue using the accepted simulation release.

### H13 — propeller-free bench and shadow estimation (P11)

The local lab operator follows the team's bench procedure and confirms propellers are removed before any relevant powered actuator work. Keep initial software checks read-only. The operator, not a remote AI, authorizes and performs physical setup and any separately approved command test.

Confirm device identification, message decoding, axes/signs/units, clock behavior, sensor freshness, reconnect behavior and CPU/memory/processing delay on the actual computer. Check RC/mode behavior and watchdogs using the approved bench procedure. Preserve timestamps and raw input recordings.

Run REEF alongside the existing system without taking control. Compare results to independent available measurements and replay the recordings through the regression suite. Agreement with a second estimator alone is not ground-truth accuracy. Resolve unexplained timing/frame differences before flight; a fast desktop simulation does not establish onboard timing performance.

### H14 — supervised vehicle commissioning (P12)

Use a written, vehicle-specific plan approved and run by qualified local personnel. It must define the operating envelope, boundaries, qualified pilot/observer roles, control handover, abort criteria, recovery procedure and logging. Exact procedures depend on the vehicle, firmware and facility; generic gains or a generic zero-thrust/disarm response are not substitutes.

Progress through the team's approved sequence: existing validated control, estimator observation, limited REEF authority, vertical control, horizontal/yaw tasks, and visual feedback if in scope. Advance only after the previous stage has reviewed evidence. Change one substantive variable at a time and record the configuration.

Do not repeat simulation fault injection by killing nodes, removing sensors, or cutting throttle during flight unless it is part of a separately approved specialist test. A person at the vehicle retains authority to stop testing. Remote AI review cannot approve physical flight or attest that a flight occurred.

### H15 — final handoff and support boundary (P13)

Have another person follow the documented installation and representative supported tests. Review the final capability matrix: simulation-only, bench-tested, flight-tested, untested and unsupported must remain distinct.

Confirm the released source/image/dependency/configuration versions match the evidence. Include actual supported hardware/firmware combinations, known limits, procedures for reporting a failure, and a tested rollback route. A flight result for one vehicle/configuration does not certify another. Tag/publish only the intended reviewed version under the project's publication policy.

## Alternatives to paying for frequent Codex checks

| Method | Best use | Limits and cost |
|---|---|---|
| Reproducible automated tests and local reports | Repeatable math, interfaces, regressions, isolation, failures and cleanup | No model credits; still requires sound expectations and local compute |
| Independently calculated cases and original-source comparisons | Catching errors shared by implementation and self-written tests | Someone must verify the reference assumptions; old-code agreement is not physical correctness |
| Compiler warnings, shellcheck and focused static analysis | Shell cleanup errors, suspicious C++ behavior and code-quality issues | Useful evidence but does not validate estimation or control mathematics |
| Debug builds with appropriate sanitizers | Runtime memory/undefined-behavior defects in owned C++ code | Additional compute; instrumentation does not prove numerical correctness |
| A fresh Claude session using R1/R2/R3 | Challenging a finished implementation with clean context | Uses Claude allowance and can share model blind spots; demand reproduced evidence |
| Copilot with repository access | Diff-focused review and, in suitable agent environments, running targeted tests | Depends on your plan and available tools; do not assume it is free or can execute commands |
| Experienced REEF/controls/robotics colleague | Baseline choice, frame conventions, observability, feedback and hardware behavior | Availability may be limited; a short source/configuration/evidence packet saves their time |
| CI on selected changes and release candidates | Checking that the repository works away from an agent's shell | Runner time, GPU/display constraints and hosted quotas apply; not all Gazebo checks belong on every push |

Start with automated checks and manual plot inspection. Spend independent review effort at R1 (estimator fidelity) and R2 (actual feedback); add R3 when real hardware interfaces are defined. If no independent AI budget is available, use the same packets with a fresh Claude session and a human reviewer. That is a workable process, but call it self-review when appropriate rather than claiming independent confirmation.

GitHub documents both pull-request code review and editor-based review, including selections and uncommitted changes. Choose a mechanism that actually includes the work you want checked: if the milestone is already committed, an empty uncommitted diff will not review it. Use a pull request or an explicit base-to-head comparison without undoing commits. See [Copilot code review](https://docs.github.com/en/copilot/how-tos/use-copilot-agents/request-a-code-review/use-code-review).

For a PR-only reviewer, paste the relevant R prompt into the review instructions and attach the short evidence packet. It can assess the diff and reported results, but should not claim it independently ran tests. For a repository-capable chat/agent, use the full R prompt and ask it to state which commands it actually executed. A review comment is a finding to investigate, not proof; require reproduction or a clear source-based argument before changing code.

For owned C++ code, [UndefinedBehaviorSanitizer](https://clang.llvm.org/docs/UndefinedBehaviorSanitizer.html) is one available runtime check. Configure diagnostics and test failure behavior deliberately; the absence of a crash alone is not a sanitizer pass. Keep expensive/instrumented runs focused on concrete risks and release gates.

The [Gazebo multicopter controller documentation](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1MulticopterVelocityControl.html) and actual version-matched source matter when establishing feedback ownership. For eventual hardware, begin with [ROSflight's hardware installation documentation](https://docs.rosflight.org/latest/user-guide/installation/installation-hardware/) and then verify the exact selected board/firmware; do not treat a legacy paper's component list as a current compatibility guarantee.

## A small evidence record to reuse

Keep summaries in Git and large bags/logs outside Git, with paths and hashes sufficient to recover the inputs. A test record can be as small as this:

~~~text
Milestone/check:
Date and human tester:
Source commit and uncommitted changes:
Host, container image/build identity, ROS/Gazebo versions:
Hardware/firmware (if applicable):
Scenario/input manifest, seed, config hashes:
Acceptance criteria revision:
Commands and exit codes:
Assertions actually run:
Measured errors, timing and health transitions:
Plot/log/recording locations:
Browser or physical observations:
PASS / FAIL / BLOCKED / NOT RUN, with reason:
Known limits and next action:
~~~

When reporting a failure, include the smallest reproducing command, exact error, relevant log excerpt and revision. Keep expected behavior separate from observed behavior. This usually saves more AI time than sending a long terminal history without an identifiable failure.
