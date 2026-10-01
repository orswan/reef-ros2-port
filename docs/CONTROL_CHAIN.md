# REEF control chain (P06)

Status: specification from source, written 2026-09-30 before any P06 port
code. Evidence labels: **[V]** read or run here, **[A]** assumption.
USER decisions (2026-09-30): the controller port is **strictly faithful and
bit-exact** to the legacy math (no robustness changes, no algorithm
improvements); the low-level loop in simulation is a **simplified stand-in**
(a development tool), not a port of ROSflight SIL.

## 1. Sources and pins

| Component | Pin | Role | Read here |
|---|---|---|---|
| `uf-reef-avl/reef_control` | **`12237b76`** (hardware bundle, on `master`; MIT license added later in `43cdee8`, 2020) | outer controller (the port target) | [V] all of `src/`, `include/`, `cfg/Gains.cfg`, `params/`, `launch/` |
| same, sim bundle | `fa4ffa39` (parent-line ancestor of `12237b76`) | differs only by requiring an unused `hover_throttle` parameter and logging it, plus one launch file | [V] `git diff fa4ffa39 12237b76` |
| `uf-reef-avl/reef_teleop` | `d13b0523` | joystick → `desired_state` | [V] `src/Teleop.cpp` |
| `uf-reef-avl/setpoint_generator` | `aa28ee42` (sim bundle) | waypoints → `desired_state` (velocity mode) | [V] `scripts/get_setpoints.py` (partly) |
| `reef_msgs` | `7fb63ff9` | `DesiredState`, `DesiredVector`, `get_yaw` | [V] |
| `rosflight/rosflight` | `44e5f37e` (2019-10-21) | `rosflight_io`, legacy `rosflight_msgs/Command` (`x, y, z, F`, `uint8 ignore`) | [V] messages |
| `rosflight/firmware` | `b77c3854` (submodule of `44e5f37e`) | command mux, attitude controller, mixer | [V] `command_manager.cpp`, `controller.cpp` (partly), `mixer.h`, `param.cpp` (partly) |

Why `12237b76` and not the sim pin: the estimator baseline is the hardware
bundle's master (BASELINE_DECISION.md); the controller math is identical in
both pins [V], and the sim pin's extra required parameter is unused.

## 2. The chain

```
setpoint source ─ desired_state (reef_msgs/DesiredState) ─┐
  (reef_teleop / setpoint_generator / dubins_path)        │
REEF estimator ─ xyz_estimate (z, ż, ẋ, ẏ) ───────────────┤  one controller step per estimate
mocap ─ pose_stamped (x, y, yaw) ─────────────────────────┤
rosflight_io ─ status (armed); `is_flying` (no publisher) ─┤
                                                          ▼
                        reef_control (outer loops, this port)
                           │ command (rosflight_msgs/Command:
                           │   mode ROLL_PITCH_YAWRATE_THROTTLE,
                           │   x roll [rad], y pitch [rad], z yaw rate [rad/s], F throttle [0,1])
                           ▼
          rosflight_io → MAVLink OFFBOARD_CONTROL → ROSflight firmware
            command mux (RC override, 100 ms offboard timeout, failsafe, arming)
            attitude controller (angle PID roll/pitch, rate PID yaw; firmware's own attitude estimate)
            mixer (quad X) → PWM → ESC/motors
```

The controller never sees attitude estimates from REEF (REEF does not
estimate attitude, INTERFACES §3.1). Roll and pitch are closed by the
firmware on its own estimator.

## 3. Loop ownership, frames, units

| Loop | Owner | Law (legacy) | Input → output, units |
|---|---|---|---|
| altitude | `reef_control` `d_` SimplePID | PID, limits **[−0.7, `max_d`]** (−0.7 hard-coded) | desired `pose.z`, estimate `z_plus.z` (NED down, m) → `velocity.z` (NED, m/s) |
| climb rate | `reef_control` `w_` | PID, limits ±`max_w` | `velocity.z`, `z_plus.z_dot` (NED, m/s) → `acceleration.z` (dimensionless); **throttle F = clamp(−acceleration.z, 0, 1)**. No hover feed-forward, no mass, gravity or tilt compensation: the integrator carries the hover throttle |
| position | `reef_control` lookup table (not a PID) | sigmoid `v = max_vel / (1 + kp·exp(−(d − center_point)/alpha))`, 0 inside `deadzone`; heading `θ = atan2(e_y, e_x) − ψ` | desired `pose.x/y` and mocap `pose_stamped` (mocap NED, m) → `velocity.x/y` (body-level, m/s) |
| heading | `reef_control` `yaw_` | PID, limits ±2.0 (hard-coded), **no angle wrapping** | desired `pose.yaw`, mocap yaw (rad) → `velocity.yaw` (rad/s); only in position mode |
| horizontal velocity | `reef_control` `u_`, `v_` | PID, limits ±`max_u`, ±`max_v` | `velocity.x/y`, `xy_plus.x_dot/y_dot` (body-level: x forward, y right, m/s) → `acceleration.x/y`, used **directly as angles**: roll = `acceleration.y`, pitch = −`acceleration.x` (rad), then clamped ±`max_roll`, ±`max_pitch` |
| yaw rate | `reef_control` | clamp ±`max_yaw_rate` | `velocity.yaw` → command z (rad/s) |
| attitude and yaw rate | ROSflight firmware | angle PID roll/pitch, rate PID yaw, on the firmware's own attitude estimate [V `controller.cpp`] | angles, yaw rate → torques |
| mixing | firmware | quad X mixer [V `mixer.h`]: F, x, y, z mix rows | F, torques → motor PWM |
| arming, failsafe, RC override, offboard timeout | firmware | offboard channels inactive after `OFFBOARD_TIMEOUT` = 100 ms [V `param.cpp`]; ignored channels and inactive ones fall back to RC; failsafe throttle 0.3 | — |
| mode selection | setpoint source | `DesiredState` flags (§4) | — |
| takeoff detection | REEF estimator (`is_flying_reef`) | **not connected**: the controller subscribes to `is_flying`, and no pinned launch file remaps it [V] | — |

Sign check (independent of the code): in NED/FRD, a positive roll (right
side down) accelerates to the right (+y), and a negative pitch (nose down)
accelerates forward (+x); so roll = +a_y and pitch = −a_x are the
physically correct signs [V by construction; checked in the unit tests].
Throttle: `w_` error = ż_c − ż; climbing faster than commanded (ż more
negative than ż_c) gives a positive error, a positive `acceleration.z`, and a
smaller F: correct sign.

Rates: one controller step per `xyz_estimate` (REEF publishes one per IMU
message; 250 Hz in the X3 scenario). `dt` is the difference of the estimate
header stamps with ROS 1 `Duration` arithmetic (§5 K4). The firmware loop
rate is not modelled [A: about 1 kHz on hardware].

## 4. Modes (per step, from the latest `DesiredState`)

Evaluation order in one step [V `PID.cpp`, `controller.cpp`]:

1. If not `initialized_`: clear all five integrators (not the differentiators).
2. Altitude loop and climb-rate loop (always, in every mode).
3. If `position_valid`: optionally `pose.yaw = θ_prev + ψ` (`face_target`,
   using θ from the **previous** step); yaw PID → `velocity.yaw`; lookup
   table → `velocity.x/y`; set `velocity_valid = true` in the stored
   message.
4. If `velocity_valid`: `u_`, `v_` → `acceleration.x/y`.
5. Publish the (modified) desired state on `controller_state`.
6. Command: `F = clamp(−acceleration.z, 0, 1)`, mode 2, and
   - neither `attitude_valid` nor `altitude_only`: ignore 0; x, y, z = the
     clamped roll, pitch, yaw rate;
   - `altitude_only`: ignore 0x07 (x, y, z ignored → RC in the firmware);
     x, y, z keep their **previous** values;
   - `attitude_valid` (and not `altitude_only`): ignore 0; x = `attitude.x`,
     y = `attitude.y`, z = **`attitude.yaw`** sent in the yaw-*rate* field;
     not clamped.

The stored desired state is modified in place and persists until the next
`desired_state` message. With no flags set, `acceleration.x/y` and
`velocity.yaw` come straight from the message (feed-forward pass-through).
Before the first `desired_state`, all fields are zero: the vehicle is held
at z = 0 with zero attitude.

`initialized_`: the `status` callback sets `armed_` and `initialized_ =
armed`; the `is_flying` callback sets `initialized_ = is_flying && armed`.
The last callback wins. With the pinned launch files, `is_flying` has no
publisher, so the integrators run whenever the vehicle is armed.

## 5. Legacy behaviour kept by the port (K-list)

These are reproduced exactly, asserted as characterizations, and are
candidates for a later, separately approved correction list (none approved).

| ID | Behaviour | Source |
|---|---|---|
| K1 | D term has the **wrong sign**: u = kp·e + kd·ẋ with ẋ the filtered derivative of the *state* (not the error), so D adds anti-damping | `simple_pid.cpp` |
| K2 | Anti-windup is **effectively inactive**: the test `|i| > |u − p + d|` equals `|i| > |i + 2d|`, never true when d = 0 (`wD = dD = 0` in the shipped quad/Y6 gains); the back-calculation also uses `+d` | `simple_pid.cpp` |
| K3 | First-sample derivative kick: `last_state_` starts at 0 | `simple_pid.cpp` |
| K4 | First step uses `dt = stamp − 0` (about 1.7e9 s on wall-clock stamps): integrator and differentiator see a huge dt (integrators are cleared again on the next step while not initialized). Backward or equal stamps (dt ≤ 1e−7; in double, a 99 ns step gives 9.9e−8 and is skipped, a 100 ns step gives 1.0000000000000001e−7 and runs) skip the step but still move the reference stamp | `controller.cpp` |
| K5 | No startup output inhibition in the controller: commands are published from the first estimate on, armed or not; inhibition is the firmware's (arming, offboard timeout) | `controller.cpp` |
| K6 | A non-finite error returns 0 but stores the non-finite state; with kd > 0 the differentiator stays NaN permanently and the output becomes NaN; `std::min/max` pass NaN through, so F can be NaN | `simple_pid.cpp`, `controller.cpp` |
| K7 | `altitude_only` sends stale x, y, z (ignored by the firmware) | `controller.cpp` |
| K8 | `attitude.yaw` is sent as a yaw *rate*; attitude-mode angles are not clamped | `controller.cpp` |
| K9 | Heading PID without wrapping: an error across ±π commands the long way round | `PID.cpp` |
| K10 | `face_target` uses θ from the previous step; θ is an **uninitialized** member before the first lookup | `PID.h`, `PID.cpp` |
| K11 | Integrators are not reset on mode changes or gain changes; differentiators are never reset | `PID.cpp`, `simple_pid.cpp` |
| K12 | `face_target` and `fly_fixed_wing` are read from the *global* namespace, so the values inside `reef_control_pid:` in `quad_pid.yaml`/`y6_pid.yaml` are ignored (defaults false); `kiwi_pid.yaml` sets them globally | `PID.cpp`, `params/` |
| K13 | Out-of-range gains are silently clamped to `Gains.cfg` ranges by dynamic_reconfigure; the config's `max_yaw_rate` (clamped to 0.5) is unused, the controller reads its own `max_yaw_rate` parameter | dynamic_reconfigure, `cfg/Gains.cfg` |

## 6. Port plan (faithful)

- Package `reef_control` (ament_cmake), history imported with
  `git subtree` (as for `reef_estimator`), sources moved by pure renames,
  then edited for ROS 2 plumbing only.
- ROS-free core: `SimplePID` (math unchanged) and the `Controller` /
  `PIDController` step with the original statement order, working on plain
  structs that keep the message field types (command fields `float32`,
  stamps as `sec`/`nanosec` with ROS 1 `Duration` arithmetic for `dt`).
- rclcpp node `reef_control_node`: the same topics (`desired_state`,
  `status`, `is_flying`, `xyz_estimate`, `pose_stamped` in; `command`,
  `controller_state` out). `rc_raw` was subscribed with an empty callback;
  it is not subscribed (no effect).
- Parameters: gains and limits as ROS 2 parameters with the `Gains.cfg`
  defaults. **Plumbing deviation (AGENTS rule):** values outside the
  `Gains.cfg` ranges are startup errors (and rejected at runtime) instead of
  being clamped (K13); in-range values behave identically. Runtime gain
  changes (dynamic_reconfigure) map to `on_set_parameters`, applying
  `setGains`/`setMinMax` exactly as `gainsCallback` (no integrator reset,
  K11). `max_roll`, `max_pitch`, `max_yaw_rate` are required (ROS_ASSERT in
  the original). K10: the port initializes θ to 0; the reference harness
  constructs the original object in zeroed storage so both agree (adaptation
  documented in `baseline/README.md`).
- Command message: the ROS 2 type `rosflight_msgs/Command` (vendored 2.0)
  with `u[0..3] = x, y, z, F`, mode 2 and ignore bits 1/2/4 (identical
  values to the legacy constants `MODE_ROLL_PITCH_YAWRATE_THROTTLE`,
  `IGNORE_X/Y/Z`). [A] **Whether ROSflight 2.x firmware reads `u[3]` as
  throttle in this mode is not verified** (its firmware source is not
  vendored); the hardware command contract is P10. The header stamp is set
  to the triggering estimate's stamp (the original left it zero; metadata
  only, for traces).
- `reef_msgs` gains `DesiredState` and `DesiredVector` (unchanged fields
  from `7fb63ff9`, `std_msgs/Header`).
- Dry-run command sink (`reef_control_sink`): records every command with a
  firmware-semantics interpretation (ignore bits, 100 ms offboard timeout,
  armed flag) to CSV; it has no hardware output, and a `hardware` parameter
  is refused (NOT IMPLEMENTED, P10).

## 7. Low-level simulation: the stand-in (built in P07)

USER decision: a simplified stand-in, not ROSflight SIL.

Audit [V]: ROS 2 `rosflight_sim` (`reference/rosflight_ros_pkgs`,
`5ef20134`) builds a standalone simulator and a Gazebo *Classic*
visualizer; no Gazebo Harmonic integration is present (the modern
`rosflight_viz_gazebo` directory is absent, and CMake skips it). Porting SIL
firmware plus a Harmonic plugin would be substantial work; it is out of scope.

**`reef_fc_standin` (package `src/reef_fc_standin`): DEVELOPMENT TOOL, not
ROSflight, not flight-representative.** As built [V]:

| Aspect | As built |
|---|---|
| Input | `/x3/reef/command` (`rosflight_msgs/Command`, mode 2; other modes give neutral values), `/x3/fc/arm` (`std_msgs/Bool`, from the scenario) |
| Command mux | `reef_control::FirmwareMux` (firmware `b77c3854` semantics): channels inactive 100 ms after the last command; ignored or inactive channels take the neutral "RC" value (roll 0, pitch 0, yaw rate 0, **throttle 0**: the vehicle descends or falls, there is no RC and no failsafe); disarmed → motors stopped |
| Attitude loop | roll/pitch: τ = I·(k_p (θ_c − θ) − k_d ω) (angle P, D on the body rate, the firmware's structure); yaw: τ_z = I_zz k_r (r_c − r). k_p = 64 s⁻², k_d = 12.8 s⁻¹ (8 rad/s, damping 0.8), k_r = 1 s⁻¹, chosen before the first run (`config/x3_standin.yaml`). No integrators |
| Attitude source | **truth**: attitude from a 250 Hz truth odometry (`/x3/fc/truth_odom`), body rates from Gazebo's noise-free gyro; one step per gyro sample (250 Hz, sim time) |
| Thrust | **linear**: T = F · T_max, T_max = 4 k ω_max² = 21.9 N (hover F ≈ 0.68). Assumption: a thrust-linearized motor; ROSflight maps F to PWM, and real thrust is not linear in PWM |
| Allocation | exact inverse of the X3's rotor geometry (FRD): T = Σf, τ_x = Σ −y f, τ_y = Σ x f, τ_z = Σ dir·m·f; each f clamped to [0, k ω_max²]; ω = √(f/k). The design's quad-X mixer rows were replaced by the exact geometry (the X3's arms are not symmetric: 0.20 and 0.22 m) |
| Actuators | `actuator_msgs/Actuators` on `/x3/fc/motor_speed` → ros_gz_bridge → `gz.msgs.Actuators` on `/X3/gazebo/command/motor_speed` (the four `MulticopterMotorModel` systems). Motor mapping verified in Gazebo before closing the loop (roll, pitch, yaw signs) |
| Physics | one owner: Gazebo. `worlds/x3_closed_loop.sdf` = `x3_flight.sdf` without `MulticopterVelocityControl` (test `test_closed_loop_world.py`) |
| Outputs | `/x3/reef/status` (armed; read by reef_control), `/x3/fc/debug` (per step: mux selection, attitude, torques, motor speeds, saturation, estimate age, timeouts), `/x3/fc/label` |
| Shutdown | Gazebo's motor model keeps the last commanded speeds (no timeout) and the Gazebo server outlives the ROS nodes by 5 s or more at the end of a launch [V]. The stand-in therefore commands zero motor speeds in a pre-shutdown callback (SIGINT, SIGTERM). Limits: the bridge must still be running; a SIGKILLed stand-in sends nothing. The closed-loop scenario lands and disarms before it ends, so normally the motors are already stopped |
| Labels | node log, label topic, run manifest, analysis report and plots say "stand-in low-level loop (development tool)" |

What the stand-in does not give: firmware estimator behaviour, RC override,
failsafe, PWM/ESC dynamics, MAVLink latency, firmware parameter semantics,
integrators in the attitude loop.

Closed-loop runs: `scripts/run_x3_scenario.sh --closed-loop`
(`reef_demo.sh closed-loop`, `reef_check.sh control`); results in
[reviews/P07.md](reviews/P07.md).
