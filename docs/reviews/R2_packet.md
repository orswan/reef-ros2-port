# R2 review packet: REEF closed-loop simulation boundary (P06, P07, P07b)

Prepared 2026-10-01. **Review revision: branch `p07b-faults-position`**, code
at `bb47cf0` (later commits on the branch change documentation only: `git
log bb47cf0..p07b-faults-position --stat`). Base of the reviewed work:
`1ee999b` (the end of R1). P06 and P07 are merged in `main`; P07b is not
yet merged (waiting for this review). The reviewer works in a
repository-capable session in either project container, from
`/root/ros2_ws/reef_ros2`, following [AGENTS.md](../../AGENTS.md) and the R2
prompt in [handoff/REEF_COMPLETION_PROMPTS.md](../handoff/REEF_COMPLETION_PROMPTS.md).
Do not edit sources; do not touch hardware (none is connected or supported).
**USER: this review will be run by an independent Claude session and must be
labelled SELF-REVIEW** (same model family as the implementer).

## 1. Capability claim (what is claimed, and what is not)

| Claimed | Not claimed |
|---|---|
| The ROS 2 `reef_control` is a **bit-exact port** of `reef_control` `12237b76` (USER: strictly faithful; legacy flaws K1–K13 kept) | any correction of K1–K13 |
| In Gazebo Harmonic, the REEF estimator and the REEF controller **close the loop** on the X3 through a **stand-in low-level loop** (`reef_fc_standin`, development tool), with the stock truth-fed controller removed | the intended **ROSflight** low-level path (firmware SIL, MAVLink, PWM/ESC); the stand-in uses **truth attitude and rates**, linear thrust, no attitude integrators |
| Tracking, stability, causality, return/land/disarm, and the documented legacy behaviour under faults (estimate loss and restart, stale setpoint, sensor loss, pause, stand-in death) and in position mode, in **simulation with idealized REEF inputs** | flight, hardware, or ROSflight evidence; realistic sensors (attitude is truth; range idealized; velocity observations are truth + noise, not RGB-D) |
| Official acceptance = **headless** runs on an idle machine (USER) | latency criteria under host load or with the software-rendered GUI (informational only) |

## 2. Specification and decisions

| Item | Where |
|---|---|
| Control chain from source: loops, owners, frames, units, modes, integrators, firmware mux, K1–K13 | [CONTROL_CHAIN.md](../CONTROL_CHAIN.md) §1–6 |
| Stand-in as built (mux, attitude law, allocation, thrust mapping, shutdown) | CONTROL_CHAIN §7 |
| Controller interface (topics, `Command` mapping `u[0..3]`, parameters, startup, sink) | [INTERFACES.md](../INTERFACES.md) §4; closed-loop topics §2 |
| Criteria (fixed before code; amendments in separate, explained commits) | [ACCEPTANCE.md](../ACCEPTANCE.md) §5 `control` P06, P07, P07b |
| USER decisions | faithful port, K1–K13 kept; `is_flying` unconnected; hardware throttle semantics deferred to P10; stand-in instead of SIL; X3 gains `dI` = 0 (explained config commit); headless official; no new failsafes (crashes documented); R2 = SELF-REVIEW |

## 3. Changed files (reviewed range `1ee999b..bb47cf0`)

| Area | Files | Reason |
|---|---|---|
| Controller port | `src/reef_control/` (history imported by `git subtree`, pure renames, then plumbing edits), `src/reef_msgs` (`DesiredState`, `DesiredVector`, `get_yaw`) | P06 |
| Controller reference | `baseline/control/` (harness, stand-in headers, provenance, fixtures + lock, independent model, `check_control.py`) | P06 |
| Stand-in | `src/reef_fc_standin/` | P07 |
| Closed-loop simulation | `src/reef_sim/worlds/x3_closed_loop.sdf` (generated from `x3_flight.sdf` minus `MulticopterVelocityControl`), `config/bridge_closed_loop.yaml`, `config/x3_closed_loop.yaml`, `config/closed_loop/*.yaml`, `launch/x3_closed_loop.launch.py`, `reef_sim/closed_loop_runner.py`, `analyze_closed_loop.py`, `closed_loop_scenarios.py`; hooks in `range_sensor.py`, `reef_adapter.py`, `src/reef_x3_adapter` | P07, P07b |
| Wrappers | `scripts/run_x3_scenario.sh --closed-loop`, `reef_check.sh control` and `faults`, `reef_demo.sh closed-loop`, `env.sh`, `check_colcon.py` | P07, P07b |

Source diff of the controller against the original:
`baseline/tools/source_diff.sh <dir>` (reef_control files added in P06).

## 4. End-to-end trace (to verify)

`/x3/imu` (sim, + seeded noise and the vibration assumption) →
`x3_imu_adapter` (FLU→FRD, **truth attitude**) → `/x3/reef/imu/data` →
REEF estimator → `/x3/reef/xyz_estimate` → `reef_control_pid`
(`/x3/reef`) → `/x3/reef/command` (`rosflight_msgs/Command`, mode 2,
`u` = roll, pitch [rad], yaw rate [rad/s], throttle [0, 1]; stamp = estimate
stamp) → `reef_fc_standin` (firmware mux: ignore bits, 100 ms timeout,
armed; angle-P/rate-D attitude loop on truth; linear thrust; allocation) →
`/x3/fc/motor_speed` (rad/s) → bridge → `/X3/gazebo/command/motor_speed`
→ 4 × `MulticopterMotorModel` (motor order = `actuator_number` 0–3; mapping
tested in `test_closed_loop_world.py` and verified in Gazebo).
The runner records the controller's subscriptions and their publishers from
the ROS graph in every run (`scenario_result.json`, `graph`).

## 5. Evidence (implementer runs, original container, headless)

| Check | Result | Where |
|---|---|---|
| `reef_check.sh control` | PASS: P06 fidelity 106/106 (21 streams, every one of 142 columns bit-identical core and node; independent model 0 difference; negative control); closed-loop nominal all P07 criteria; causality +0.30 m | STATUS §5i |
| `reef_check.sh faults` (at `bb47cf0`) | PASS, 1667 s: F1–F12 36/36; all 11 closed-loop scenarios PASS | STATUS §5j |
| `reef_check.sh control` (at `bb47cf0`), `regress_x3_scenario.sh`, `regress_clock_check.sh` | PASS (P06 106/106; nominal P07; causality 0.300 m; regressions all cases) | STATUS §5j |
| USER, dev container | `control` 106/106 + `interfaces` (P06); headless `control` (P07) passed; GUI demo returns, lands, disarms | STATUS §7 H9, H10 |
| Recorded runs | `recordings/p07_*`, `recordings/p07b_*` (manifests, bags, `analysis_closed_loop/`) | [reviews/P07.md](P07.md), [reviews/P07b.md](P07b.md) |

Nominal closed loop (X3 gains, 3 runs): takeoff 4.1 s after arming, tilt
≤ 0.09 rad, altitude hold RMSE ≤ 0.016 m, velocity RMSE ≤ 0.009 m/s, yaw
rate 0.262 for 0.3 rad/s (P-only stand-in yaw loop), no saturation;
returns to 0.03 m of the start, lands, disarms. Shipped quad gains: first
hover fails the altitude-hold limit through K2 windup.

Fault and position scenarios: every judged check and characterization PASS;
crashes where the legacy system has no protection (long estimate loss,
controller restart: 3.7 s ≫ 100 ms timeout, stand-in exit). Details and
numbers: reviews/P07b.md.

## 6. Commands for the reviewer (container; headless; idle machine)

```bash
scripts/reef_check.sh control      # about 12-14 min
scripts/reef_check.sh faults       # about 35-40 min
REEF_X3_CL_SCENARIO=dropout_long scripts/run_x3_scenario.sh --closed-loop   # the discriminating estimate-loss case
REEF_X3_RANGE_BIAS=0.30 REEF_X3_NOMINAL=recordings/<nominal run> scripts/run_x3_scenario.sh --closed-loop  # causality
baseline/tools/source_diff.sh /tmp/r2_diff
```
Each run writes `recordings/<run>/analysis_closed_loop/report.txt` and
`closed_loop.png`.

## 7. Failed, skipped, and known-limited items

*Correction after R2 (2026-10-01): the latency item below understated the
problem. A headless run (`p07_cl_nominal_b`) also failed at 24 ms; the
precise distribution is in ACCEPTANCE §5 (P07 note) and STATUS item 24.*

- Latency margin: estimate age at the motor command p99 12–24 ms (limit
  20 ms) in headless runs; GUI runs on a loaded host fail it (informational
  by USER decision; no rework). STATUS §6 item 24.
- Not run by the implementer in the dev container (USER ran `control`).
- Not covered: RC override, failsafe (none exists in the stand-in), landing
  quality (reported only), wind, sensor noise models beyond the scenario,
  ROSflight firmware semantics of `Command.u[3]` (P10).
- Three P07b characterizations were corrected after the first runs (USER
  approved; `384bec6`); several evaluation bugs fixed (`203a59e`); listed
  in reviews/P07b.md. Review whether the corrections are justified by the
  recorded data.

## 8. Specific uncertainties for the reviewer

1. **Is the estimator really in the loop, and is truth confined to the
   declared idealized inputs?** Evidence: the data-path check from the ROS
   graph, the range-bias causality run (+0.30 m bias → 0.30 m lower), and
   `dropout_long` (estimates stop → timeout → fall). Look for any other path
   from `/x3/truth/*` or `/x3/fc/truth_odom` to the controller.
2. **Does the stand-in hide controller problems?** It flies on truth
   attitude with an 8 rad/s attitude loop, linear thrust, and exact
   allocation; the REEF controller's outer loops may look better than on a
   real firmware/ESC chain. Judge whether P08/P09 can proceed on this basis
   and what P10 must re-verify.
