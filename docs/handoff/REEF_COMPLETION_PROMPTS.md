# REEF ROS 2 — remaining implementation and review prompts

Prepared 2026-09-29. Companion files: [context](REEF_PROJECT_CONTEXT.md) and [testing/review](REEF_TESTING_AND_REVIEW.md).

This is a completion playbook, not a claim that code or tests described below already exist. It covers the remaining simulation port and the conditional physical-drone work. Unknown hardware choices and future defects cannot be predetermined; the hardware intake and repair prompts handle those branches.

## How to use this guide without spending credits on repeated planning

1. Put all three Markdown files in the project, for example under docs/handoff/. Keep existing AGENTS.md authoritative; reconcile conflicts explicitly.
2. Give Claude the launch instruction below and ONE milestone prompt at a time. Claude should reuse completed work, implement, run focused checks, commit, and update persistent status.
3. Use the reusable repair prompt for concrete failures. Do not commission a new broad review for every fix.
4. Use independent reviews R1 and R2 at the two simulation boundaries. R3 is for actual hardware integration later. Codex, Copilot with suitable repository access, a fresh Claude session, or an experienced human can perform them; see the testing guide for differences.
5. At each milestone, the user performs the matching human check. A PASS from an unimplemented, skipped, stale, or unrelated test is not completion.
6. Stop at a declared simulation release if hardware is unavailable. Continue P10–P13 when the target vehicle and lab are available.

| Prompt | Deliverable | Human check | Independent review |
|---|---|---|---|
| P00 | Reconciled state and stable check commands | H0–H2 | Only if old blockers remain unresolved |
| P01 | Quadrotor, simple sensors, recordings | H3 | Automated/self-review |
| P02 | Chosen baseline and independent reference comparisons | H4 | Included in R1 |
| P03 | Messages/helpers and ROS interfaces | H4–H5 | Included in R1 |
| P04 | Vertical estimator | H5 | Included in R1 |
| P05 | Horizontal/fused estimator and replay | H6–H7 | R1 |
| P06 | REEF controller and command-interface contract | H8 | Included in R2 |
| P07 | Actual REEF closed-loop simulation | H8–H9 | R2 |
| P08 | RGB-D sensing/processing path | H10 | Targeted review if it changes feedback/failure behavior |
| P09 | Simulation release, CI, installation and documentation | H11 | Automation + human reproduction |
| P10 | Specific hardware integration contract | H12 | R3 before control is enabled |
| P11 | Propeller-free bench and shadow estimation | H13 | Human/lab engineering review |
| P12 | Supervised commissioning and flight evidence | H14 | Qualified human authority |
| P13 | Final supported release and handoff | H15 | Human reproduction + targeted checks |

## Session launch instruction — give once per new Claude session

~~~text
Work on the REEF ROS 2 migration in /root/ros2_ws/reef_ros2. Read the project's AGENTS.md and current STATUS/MIGRATION documents, plus the three handoff files if present.

Use the context handoff as conversation history, not proof of the current repository. Inspect Git state and the actual execution environment. Preserve user changes and the original ros2_novnc_container. Use the working replacement environment and its documented name/ports.

Implement only the milestone I provide. Make routine implementation choices yourself. Use a focused branch and small commits. Pin dependency versions, preserve source history/licensing, and keep middleware changes separate from algorithm changes.

Run meaningful checks, inspect their results, and fix failures within scope. Do not weaken assertions or silently update golden data to make the implementation pass. Report real defects separately from environment blockers.

Do not invoke additional paid agents automatically. Prepare review packets for the sparse checkpoints. Do not send messages, publish releases, or operate physical hardware on my behalf.

If a needed host operation is unavailable inside the container, finish the files and available checks first, then give exact Mac-terminal commands and expected outputs. Do not add privileged Docker access merely for convenience.

At completion update docs/STATUS.md with revision, scope, actual checks and exit statuses, failed/skipped checks, data/config versions, human checks still needed, and the next milestone. Write brief docs/reviews/<milestone>.md evidence where useful. Commit reviewable changes; report the commit hash. Preserve large raw logs/recordings outside Git and track their manifests.
~~~

## P00 — reconcile progress and make testing accessible

~~~text
Reconcile the actual project state with the handoff. The user reported that the rebuilt Docker environment passed manual Mac-terminal and VS Code checks; do not rebuild or repeat those checks without a concrete reason.

Identify current source/container revisions, original checker-fix commits, any review results, and actual replacement-container settings. Confirm whether the known false-pass and SIGTERM observer defects were repaired. If still open, fix and regression-test them before trusting the checker. Retain test-owned isolation in both ROS and Gazebo, owned-process monitoring, and cleanup that never targets unrelated processes.

Create or update docs/STATUS.md, docs/ACCEPTANCE.md, and docs/INTERFACES.md. Keep uncertainty explicit.

Provide a small, documented command interface, wrapping existing checks rather than rewriting them:
- scripts/reef_check.sh help
- scripts/reef_check.sh TARGET, for env, clock, sim-data, baseline, estimator, faults, control, vision, release.
- scripts/reef_demo.sh help, with stock, estimator, closed-loop, vision, and replay modes implemented as their milestones become available.

All demo modes default to simulation. A future or unavailable target must say NOT IMPLEMENTED and exit nonzero, never silently succeed. Use exit 0 for an executed successful check, 1 for an executed failed check, and 2 for unavailable/invalid invocation. Document any unavoidable existing conventions at the wrapper boundary.

Each check should report source/config/input revisions, assertions run, elapsed wall and simulation time when relevant, and artifact paths. A negative-test harness can pass only after checking the expected nonzero result of the rejected case. Avoid success inferred from a background process merely starting.

Create concise acceptance criteria before evaluating each later implementation. For mathematical equivalence use justified numeric tolerances; for simulation use explicit errors, timing limits, and scenarios. Never infer flight readiness from synthetic tests.

Document how to run builds from the project root with --base-paths src and how to inspect failed colcon results. Keep this a small usability layer, not a new test framework.
~~~

## P01 — repeatable multicopter and simulated measurements

~~~text
Implement the first incomplete simulated-quadrotor milestone. Reuse any accepted work already present.

Use the audited, version-matched Harmonic example and X3 unless inspection establishes an incompatibility. Record model sources, versions, checksums, dependencies and license information. Provide a pinned setup/download step and persistent cache. Test an offline run after setup; all nested meshes/textures must be available. Do not commit a large incidental Fuel cache into ordinary Git.

Use the stock controller solely to generate motion, documenting its internal ground-truth feedback. Provide a bounded scenario: settling, hover, ascent/descent, horizontal motion, and stop. Commands, frames, limits, durations and noise seeds must be configurable and recorded.

Add an appropriate IMU and a simple downward range source. Geometry-derived range is acceptable initially if labelled idealized, with sensor pose, beam, tilt, limits and flat-ground assumptions explicit. Do not confuse vertical height with slant range. Keep ground-truth pose/velocity separate from measurements. Label any ideal attitude input; it is not an implemented attitude estimator.

Expose suitable standard ROS 2 messages. Document units, frames, acceleration/specific-force and gravity conventions, timestamps, configured rates and future REEF transformations. Do not hide missing data by substituting truth without a visible mode label.

Implement reef_demo.sh stock [--gui] and reef_check.sh sim-data. Record relevant streams with rosbag2 and an input/config/source manifest outside Git. Provide small plots and a documented analysis command for truth motion, range geometry and stationary/moving IMU behavior. Confirm the bag contains data, not only metadata.

Use simulation time, bounded wall-clock timeouts, established isolation and owned-process cleanup. Check that the vehicle actually moves, required streams arrive, unavailable streams fail the test, and interruption leaves VNC/unrelated processes healthy. Measure real-time factor without requiring exactly 1.

Finish with one-command launch/record/analysis instructions and evidence. Do not port REEF or add rendered cameras yet.
~~~

## P02 — algorithm baseline and trustworthy reference tests

~~~text
Choose and document the estimator algorithm baseline before changing its mathematics.

Compare pinned master e4179f48c3f26e22bd1366b71ee1e117ce2f5f7f with the simulation revision beginning 95987b51 (resolve its full SHA). Trace all differences already identified in the handoff to source lines: rejection, initialization, timing, F/bias coupling, Q/R/P, gravity initialization, and new-measurement flag handling. Examine any other behavior-changing differences.

Write docs/BASELINE_DECISION.md: selected revision, reasons, equations/state ordering/frames, initialization, propagation/update schedule, time-step and noise semantics, partial-update behavior, rejection rules, defaults, and explicitly approved deviations. Master is a candidate, not an unquestionable specification.

Create deterministic, timestamped event fixtures and a minimal reference harness using the pinned original mathematics with only necessary mechanical adaptations. It need not recreate the whole ROS 1 flight system. Record adaptation patches and original source provenance so the reference remains inspectable.

Compare against independent analytic cases/invariants too: stationary motion, constant acceleration with known convention, bias response, range geometry, covariance symmetry/finite values and justified semidefinite tolerances. Test full/partial updates and rejected measurements.

If a legacy bug is established, characterize it with a regression, preserve original outputs, and implement any chosen correction in a separately documented change. Do not use port outputs as their own golden oracle. Golden updates require an explained source/algorithm decision.

Set numerical equivalence tolerances before testing the port, allowing justified floating-point differences. Keep filter correctness and fidelity-to-old-code as separate questions.

Implement reef_check.sh baseline. Produce a baseline decision and evidence that a reviewer can assess without reading the entire repository. If evidence is insufficient to settle a choice, state the remaining uncertainty and keep both references; do not silently blend them.
~~~

## P03 — messages, helper code, and ROS 2 interfaces

~~~text
Port the minimal dependencies required by the selected estimator baseline.

Audit reef_msgs as both message definitions and C++ helper code, including matrix/parameter parsing. Preserve source attribution/history and port only the required pieces first. Use appropriate ament/rosidl build configuration, explicit exports and dependencies, and validate parameter dimensions/types.

Verify the estimator's RCRaw usage against a pinned upstream ROS 2 rosflight_msgs. Prefer the compatible upstream message package and build only its necessary dependencies. Check fields, channel indexing/units, defaults and switch semantics; do not invent a look-alike package. Keep simulation RC switching explicitly disabled and missing-RC behavior documented. Old launch files that start rosflight_io are a separate integration path.

Define the node interface contract: inputs/outputs, message fields, units, frame_id meanings, covariance layout, timestamps, QoS, parameters, health/reset behavior, and measurement selection. Do not claim that altitude/velocity estimates are a complete global pose. If standard messages are used, clearly represent unavailable components rather than presenting fabricated states as measured.

Keep numerical state and propagation/update functions testable independently of ROS. Minimize restructuring until comparison tests protect behavior. Select an execution/callback strategy that makes state mutation and input ordering explicit.

Add small message/helper and parameter-validation tests. Build/test in the reproducible environment with real packages selected. Update declared dependencies; do not treat an empty colcon build as validation.

Keep flight-control firmware and hardware command publication out of this task.
~~~

## P04 — vertical estimator

~~~text
Implement the selected REEF vertical estimator and its ROS 2 wrapper using the accepted baseline and interfaces.

Preserve state order, bias/gravity signs, propagation/update timing, noise discretization, initial covariance, partial updates, range compensation and rejection semantics except for separately documented baseline corrections.

Feed exactly matched event streams into reference and port; compare states and covariances. Test stationary, ascent/descent, tilt/range geometry, parameter errors, missing/range-invalid data, repeated measurements and timestamp anomalies.

Use measurement timestamps and the documented clock policy. Do not replace a fixed-step model with variable dt, or change its process-noise scaling, merely as a ROS API migration.

Run the port alongside the stock-controlled simulated drone, with separate truth scoring and visible idealized-sensor labels. Produce altitude/vertical-velocity errors and covariance plots after a defined initialization interval.

Extend reef_check.sh estimator to include named vertical cases and show that horizontal coverage is still unavailable. Implement the available part of reef_demo.sh estimator and a documented offline replay path without competing /clock publishers.

Handle launch/reset/shutdown deterministically. Do not close the flight-control loop in this milestone. Update evidence, interface documentation and acceptance results.
~~~

## P05 — horizontal estimation, fusion, and recorded-data validation

~~~text
Complete the horizontal and combined REEF estimator port.

Preserve the chosen velocity/attitude-bias/accelerometer-bias states, body-level frame transformations, partial updates, gating, measurement flags, and covariance semantics. Verify frame conversions with independent vectors/rotations, including nonzero yaw and tilted attitude; relabelling coordinates is not a transformation.

Prevent unintended reuse or double fusion of observations while respecting the approved baseline. Keep any legacy-behavior correction separate and regression-tested. Document whether optional mocap and RGB-D measurement inputs have compatible ROS 2 interfaces.

Initially use clearly labelled simulated velocity observations as needed; do not claim these exercise RGB-D odometry. Keep the external attitude assumption visible.

Compare original and port outputs on identical ordered events and on recorded simulated streams. Include rejected/out-of-order/duplicate measurements, start-up, resets, sensor dropouts and parameter failures. Evaluate physical plausibility separately from numerical parity.

Complete reef_check.sh estimator, baseline, and the currently relevant faults cases. Produce per-axis errors, finite/covariance checks, initialization behavior and timing statistics with configured thresholds. Quantify assumptions behind any statistical consistency metric.

Review QoS compatibility and callback ordering; use the simplest adequate executor configuration. Verify replay cannot consume an unrelated live clock or sensor stream.

Prepare R1's compact review packet: baseline decision, source diff/approved deviations, interface table, fixture provenance, comparison results, and exact reproduce commands. Make no controller or firmware changes.
~~~

## R1 — sparse independent review: estimator fidelity and meaning

Use Codex or another capable reviewer with the packet and necessary source access.

~~~text
Independently review the completed REEF ROS 2 estimator boundary. Read AGENTS.md, BASELINE_DECISION, INTERFACES and the R1 packet. Identify the exact revision. Do not edit.

Concentrate on equation/state/covariance parity, independently grounded reference fixtures, approved legacy corrections, gravity and frame signs, dt/Q semantics, partial updates, gating, observation reuse, parameter validation, time and callback ordering. Verify that ideal inputs and unestimated state components are labelled honestly.

Run a small discriminating subset: one nontrivial reference-versus-port comparison, an independent frame/gravity case, and a rejection/duplicate/time-fault case. Inspect expected values before trusting green tests. Do not rerun all unrelated environment checks.

If your interface cannot execute code, review statically and list the exact tests the human/implementer must run; do not report execution.

Return actionable findings with evidence, blocked checks, and an estimator-readiness assessment limited to the declared simulation inputs. This review does not authorize flight.
~~~

## P06 — controller port and command-interface design

~~~text
Port the REEF controller required for altitude and horizontal-velocity control, using the accepted estimator outputs. Inspect reef_control and needed teleoperation/guidance packages and preserve provenance.

First specify the control chain: requested velocity/altitude/yaw behavior → REEF estimates → REEF outer controller → intended low-level attitude/rate/thrust interface → actuator model. Determine actual commands and units from source; do not guess or replace the output with a stock velocity command that bypasses REEF feedback.

Document ownership of each loop, coordinate frames, thrust normalization, saturation, rates, mode transitions, setpoint freshness, integrator initialization/reset and anti-windup. Distinguish deliberate robustness changes from faithful controller porting.

Add pure-function or controlled-harness tests with independent expected signs, limits and transitions. Verify response to stale/invalid estimates, missing commands and restarts. Define startup output inhibition until required state is valid.

Choose a version-compatible low-level simulation integration, prioritizing the intended ROSflight architecture. Audit existing ROSflight ROS 2/SIL components and the actual Harmonic gap. Document the required adapter/plugin work and any firmware assumptions.

A simplified low-level surrogate may be a labelled development fixture, but it does not complete the intended ROSflight control chain. Do not claim closed-loop stack completion until P07 proves the chosen chain.

Provide a dry-run command sink and traces that can be inspected without sending commands to physical hardware. Extend interface and acceptance documents and keep hardware mode disabled by default.
~~~

## P07 — actual REEF-driven closed-loop simulation

~~~text
Integrate and validate the full selected REEF feedback chain in Gazebo Harmonic.

Implement the required ROSflight SIL/low-level-controller and Harmonic adapters/plugins identified in P06. Treat this as substantive integration work if no compatible plugin exists; do not claim ros_gz alone solves it.

Choose one authoritative physics engine. If Harmonic integrates the vehicle, do not also integrate duplicate ROSflight standalone dynamics. There must be one owner of each command/actuator layer, explicit motor ordering/units, compatible sensor timestamps and documented firmware/message revisions.

Disable the stock Gazebo truth-based outer controller in REEF-controlled runs. Trace REEF estimates into the REEF controller and through the intended attitude/thrust/rate interface to motor forces. Truth is retained for physics/scoring, not secretly substituted as outer-loop estimator feedback.

Add simulation-only causality checks: a bounded perturbation or interruption of the selected estimate must affect the intended controller path and trigger the documented response. Log selected sources/modes and demonstrate that another active controller is not masking failure.

Start with altitude control, then horizontal velocity and yaw requests. Tune simulation parameters in separate versioned configuration changes; never conceal estimator divergence by disabling rejection or feeding truth. Compare against predeclared tracking, overshoot, settling, saturation, latency and finite-state limits.

Implement reef_demo.sh closed-loop [--gui] and reef_check.sh control. Extend faults tests for stale estimates/setpoints, relevant low-level link loss, reset, pause/resume and process restart. Use measured/defined simulation and steady-clock semantics appropriately; avoid large propagation jumps.

Provide stock-controlled versus REEF-controlled manifests, recordings and plots. Keep all failure injection disconnected from real hardware. Package the architecture, causality evidence, firmware/plugin diffs and regression results for R2.

If the intended low-level integration cannot be completed with available components, deliver the implemented subset and precise blocker; do not relabel a surrogate or stock-controller demonstration as completed REEF control.
~~~

## R2 — sparse independent review: real feedback and failure behavior

~~~text
Review the REEF closed-loop simulation boundary at its exact revision. Use AGENTS.md, the interface/control-chain documents and R2 packet. Do not edit or touch real hardware.

Trace a command and an estimate end-to-end. Confirm the stock truth-feedback outer controller is inactive, REEF estimates are actually consumed, one physics integration and one controller per layer are active, and low-level command modes/units/motor ordering are correct.

Inspect startup inhibition, limits, mode transitions, integrator handling, stale-data policies and clock/reset behavior. Run a representative tracking case and a discriminating estimate-loss/perturbation case that would expose a bypassed estimator. Verify independent truth scoring and actual failure detection.

Review adapter/firmware changes and their supported versions. Treat any idealized inner loop as an explicit validation limitation. Check whether the evidence establishes the intended ROSflight path.

Report evidence-based defects and untested assumptions. Identify the exact supported simulation configuration and whether P08/P09 can proceed. A static-only review must say it did not execute tests.
~~~

## P08 — RGB-D processing and remaining declared sensor modes

~~~text
Implement the sensor-processing path needed to reproduce the intended vision-enabled REEF capabilities, using the validated core and closed-loop simulation.

Audit the original RGB-D odometry, delta-pose/velocity conversion, calibration and optional mocap/teleop interfaces. Port required components or explicitly justify a replacement behind the same documented measurement contract. An algorithm replacement needs its own noise/frame/timing assessment; do not call it a mechanical port.

Add rendered RGB and depth streams with correct camera information, optical frames, extrinsics, timestamps, depth units and invalid-pixel conventions. Produce velocity from those streams through the selected processing algorithm. Truth-derived noisy velocity is not acceptance evidence for this stage.

Run a textured scene with useful motion, followed by weak texture/depth loss/delayed or missing frames. Verify health and recovery behavior without silently switching to truth. Keep measurement gating and rate/covariance assumptions explicit.

Implement reef_demo.sh vision and reef_check.sh vision. Compare vision-derived velocity and REEF estimates with independent truth in matching frames; verify closed-loop response to the declared measurement failures.

On the Intel Mac software renderer, reduce resolution/rate and real-time factor as documented test profiles when necessary. Report both simulation-time correctness and wall-time performance. Do not equate slow simulation success with adequate onboard flight performance.

For any advertised mocap/teleoperation mode, implement/test its interface or mark it unsupported in the capability matrix. No hardware camera or mocap server is required for a simulated interface test, but driver compatibility remains unverified until hardware testing.

If rendering/perception performance blocks this environment, provide measured bottlenecks and a reproducible alternative runner profile; do not require the old laptop or cloud spending without the user's choice. Keep the limited-sensor release distinct from the completed vision-enabled target.
~~~

## P09 — simulation release and low-cost continuous checks

~~~text
Prepare a reproducible simulation release for the actually completed capabilities.

Run focused automated tests and a fresh-checkout/image reproduction independent of hidden overlays, ignored fixtures and old caches. Fetch pinned resources through declared setup steps. Verify a subsequent offline simulation where claimed.

Create a compact capability matrix: estimator inputs, controller modes, low-level backend/firmware version, vision/mocap/teleop support, limitations and evidence. Separate a core idealized-sensor release from a vision-enabled release; incomplete advertised features must be marked incomplete.

Add local validation scripts and a modest CI workflow: deterministic math/unit/interface tests and short headless integration checks by default, with expensive rendering/long simulation suites explicitly invoked at milestones. Configure nonzero failures and publish useful reports. Do not turn on paid AI review for every push.

Use relevant compiler warnings, shell/Python checks and a debug sanitizer profile for owned C++ code. Review suppressions. Do not require unrelated stylistic rewrites.

Implement reef_check.sh release for the declared profile. Include exact build/test commands, data/config manifests, licences/attribution, example launch/replay/plot commands and troubleshooting. Complete the human procedures in the testing guide against actual command names.

Create docs/RELEASE_SIMULATION.md identifying tested versions, passed/failed/skipped gates and the exact release commit. Prepare a local tag only after tests and documented human acceptance; do not publish externally unless the user asks.

Preserve a short review packet so an unfamiliar engineer can reproduce the result. Then mark physical-drone support pending and stop hardware-dependent implementation until P10 supplies real target details.
~~~

## P10 — hardware intake and specific command contract

~~~text
Prepare physical integration for the actual vehicle. First determine what is known; do not guess a flight-controller board, firmware revision, companion CPU/OS, command mode, sensor model, link or RC behavior.

Create docs/HARDWARE_INTAKE.md and request one compact set of missing facts from the user/lab: board and firmware revision, supported ROSflight link/messages, airframe/motor configuration, companion architecture and power/link arrangement, sensors/drivers/calibrations, RC/manual-takeover arrangement, lab test procedures and responsible operators.

While those details are unavailable, complete the intake form, interface tests with mocks, deployment checklist and config schema. Mark hardware support blocked by those specifics. Do not flash, arm, calibrate actuators or send physical control commands.

With a confirmed target, verify board/firmware/host-message compatibility and build for the companion architecture. Do not assume an Intel Mac image runs efficiently or correctly on an ARM companion. Preserve the flight controller's low-level firmware responsibilities; ROS 2 host support is a separate concern.

Implement the hardware transport adapter and target-specific configuration with explicit units, frames, command modes, limits, validity checks and freshness watchdogs. Provide read-only observation/dry-run modes as defaults, explicit source selection and no route for simulated topics to drive hardware accidentally.

Document the accepted loss-of-command, loss-of-estimate, RC takeover and restart responses for this exact firmware/platform with the lab. There is no universal safe “zero thrust” or disarm-in-air fallback.

Prepare deployment/rollback instructions and automated interface-contract tests. Package the target spec, compatibility evidence, command mapping and failure policies for R3 before control is enabled.
~~~

## R3 — sparse independent review: hardware interface

~~~text
Review the proposed physical-drone interface at its exact revision and target hardware/firmware configuration. Use the intake, interface, deployment, failure-policy and R3 evidence documents. Do not operate hardware.

Audit command modes, units, axes, motor/channel mappings where applicable, thrust normalization, limits, validity, freshness, time sources, mode transitions, startup inhibition, RC takeover ownership, firmware compatibility and rollback.

Verify isolation of simulated inputs from hardware commands and ensure no stock or firmware controller duplicates the intended REEF outer loop. Inspect mock tests for invalid/stale data, process restart and transport failure; execute only non-actuating tests if capable.

Identify what remains a lab measurement or operational acceptance question. Report concrete defects and constraints. This is a code/interface review, not flight authorization; an experienced lab operator must accept the platform-specific test procedure.
~~~

## P11 — bench and shadow-data support

~~~text
Prepare and support the lab's propeller-free bench and shadow-estimation checks for the confirmed target, following the accepted hardware procedure.

Write a human-run checklist for timestamps, stationary IMU/gravity direction, mounting/calibration, range behavior, firmware status, dry-run controller outputs and freshness detection. Use read-only/dry-run modes; no autonomous arming or actuator exercise.

Provide a data-collection manifest and analysis commands for comparing real inputs/estimates with independent reference measurements where available. Flag missing reference truth instead of asserting accuracy.

If the lab later collects flight data under an already validated controller/pilot, analyze REEF in shadow mode without control authority. Measure bias, dropouts, delay, covariance behavior and CPU/timing margins on the actual companion.

Transfer simulated configuration only as an initial documented hypothesis. Keep hardware calibration/noise/gain changes versioned, justified and separate from code changes. Replay real recordings through the estimator and add small regression fixtures when permitted.

Update acceptance evidence and propose corrections from actual data. Record who performed physical checks, exact vehicle/config/firmware revisions, and unresolved issues. Do not advance to REEF-controlled flight solely because code tests pass.
~~~

## P12 — supervised commissioning and physical flight evidence

~~~text
Prepare the final commissioning package for qualified lab operators. Actual arming, flashing, actuator calibration, flight, takeover and abort decisions remain human operations under the lab's accepted platform-specific procedure.

Use the accepted hardware interface, bench/shadow results and vehicle limits. Document a progressive test sequence: independent manual-takeover/contingency verification in the appropriate ground setup; limited REEF altitude-control trial if supported; then bounded horizontal-velocity/yaw trials; then the approved vision or other sensor mode.

For every trial specify entry conditions, active controller/source configuration, permitted envelope, logging, measurable success criteria, abort conditions and the named human roles. Do not supply generic flight gains or a universal in-air emergency command.

Prepare logging and analysis tools before the flight. After humans supply results, analyze tracking, estimator errors where reference truth exists, control saturation, latency, CPU margins, dropouts and mode transitions. A successful hover alone is not full acceptance.

Use separate configuration commits for tuning. Repeat only the affected trial plus necessary regression checks after a change. Keep unexplained behavior as a failed gate rather than widening tolerances until it passes.

Produce a commissioning report for the exact tested vehicle/firmware/sensors/parameters. If no qualified operator or hardware evidence is available, deliver the test package and explicitly retain physical validation as pending.
~~~

## P13 — final handoff and supported release

~~~text
Finalize the port for the capabilities actually demonstrated.

Reconcile source/dependency/image/config versions and the capability matrix. Preserve original authorship and licences. Verify from a fresh checkout that another person can build, run the declared Gazebo scenarios, replay data and reproduce the reported metrics.

Consolidate architecture, interface, baseline-decision, supported-platform, calibration, launch, failure-mode, troubleshooting and rollback documentation. Include known deviations from the original REEF implementation and evidence for each.

Record all unimplemented or unverified features. Use separate simulation and hardware validation statements/tags. Physical support applies only to the tested configuration and envelope; do not generalize to all drones or boards.

Complete a release checklist with human acceptance records, immutable commit/config references and example data manifests. Prepare a reviewable PR or release description locally; publish or merge externally only when the user requests it.

Update the context handoff and STATUS with final revision, commands, remaining issues and maintenance ownership. Keep a minimal regression suite and explain when larger tests should run after future changes.
~~~

## Reusable repair prompt

~~~text
Address the concrete findings below in the REEF project. Read current instructions and Git state. Reproduce each finding or explain why the evidence does not apply to this revision.

Make the smallest maintainable fix, add a discriminating regression where useful, and rerun the affected tests plus necessary adjacent checks. Do not alter acceptance thresholds or golden results without a separately explained specification/algorithm change.

Preserve unrelated work, the original container, and physical-hardware isolation. Do not expand into future milestones.

Report each finding as fixed, not reproducible with evidence, or blocked. Include commits, actual test results, and any new limitation. Prepare only changed areas for a focused re-review if a material risk remains.

Findings:
[Paste the actual findings here.]
~~~

## Reusable resume prompt

~~~text
Resume the REEF migration. Read AGENTS.md, docs/STATUS.md, the latest milestone evidence and the handoff. Inspect current Git state and running environment.

Identify the first incomplete milestone and its missing acceptance evidence. Do not infer progress from old plans or redo accepted work. Continue that bounded milestone using the completion guide.

If only host access or hardware facts block it, finish all available code/docs/checks, give the exact remaining human action, and identify another independent useful task if one exists. Do not invent hardware settings or declare a blocked gate passed.
~~~

## Review packet format

Keep each packet compact: exact base/head revisions; capability claim; changed files and reason; interface/baseline decisions; test commands, outputs and input hashes; failed/skipped tests; one or two specific uncertainties. Link the complete artifacts rather than pasting the entire chat into every review.

Prompts R1–R3 work in a repository-capable chat/agent. A pull-request review UI may accept only a diff and review instructions; in that case put the focused criteria and evidence links in the PR description. Runtime verification still needs a suitable environment and someone to execute it.
