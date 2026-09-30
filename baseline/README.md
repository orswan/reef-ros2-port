# REEF estimator reference harness (P02)

Runs the **unmodified**, pinned original REEF estimator code on deterministic
event fixtures, so the ROS 2 port can be compared with the original
mathematics rather than with itself. The baseline decision, specification,
and tolerances are in [docs/BASELINE_DECISION.md](../docs/BASELINE_DECISION.md).

```bash
scripts/reef_check.sh baseline          # container terminal; the whole check
```

| Path | Role |
|---|---|
| `provenance.json` | pinned commits and SHA-256 of every original file compiled |
| `fetch_sources.sh` | clones the pinned upstream repositories into `reference/` if missing |
| `build_reference.sh` | extracts the files from the pinned Git objects, verifies them, and builds `build/baseline/<variant>/reef_ref` |
| `harness/reef_ref_main.cpp` | drives `SensorManager` → `XYZEstimator` with an event file and writes the full state after every event |
| `harness/shim/` | minimal ROS 1 stand-in headers (only what the sources use) |
| `harness/access_prelude.h` | adaptation A1 |
| `tools/fixtures.py` | deterministic fixtures and parameter sets |
| `tools/independent.py` | step-wise re-derivation of the equations (numpy), independent of the C++ |
| `tools/check_baseline.py` | runs everything and writes `build/baseline/report/` |
| `fixtures.lock.json` | SHA-256 of every generated fixture |
| `golden/` | decimated outputs of the unmodified originals (both variants, both parameter sets) |

## Adaptations (the complete list)

No original file is patched. The build uses the original directory layout, so
even the relative include `../../reef_msgs/include/reef_msgs/dynamics.h`
resolves.

- **A1:** `access_prelude.h` is force-included. It first includes every
  standard, Boost, and Eigen header, then defines `private`/`protected` as
  `public`, so only the REEF class definitions are affected, identically in
  every translation unit. Access control only.
- **A2:** stand-in headers for roscpp and the message types, with ROS 1
  semantics where the estimator depends on them:
  - `Time::toSec() = sec + 1e-9 nsec`;
  - `param` defaults;
  - integer-valued `double` parameters;
  - `ROS_ASSERT` enabled, as in a catkin default build;
  - `sensor_msgs/Range` fields are `float`, as in ROS 1.
- **A3:** the harness sets the uninitialized diagnostic member
  `Mahalanobis_D_hat_square` to NaN before the first event. The estimator
  writes it before reading it.
- **A4:** an event reaches its callback only if the ROS 1 node would have
  subscribed to its topic (`enable_*` parameters, `enable_mocap_switch`).
- **A5:** events are delivered in fixture order, one at a time. ROS 1 queue
  depths and threading are not modelled.
- **A6 (R1, 2026-09-30):** the stand-in `Publisher::publish` also passes
  `reef_msgs::XYZEstimate` and `XYZDebugEstimate` messages to
  `record_published` (argument-dependent lookup; other types are ignored).
  With `REF_PUBLISHED=<path>`, the harness writes the fields of what the
  original published, one row per estimate. The main output CSV and the
  golden files are unchanged. Needed because the original publishes before
  its takeoff check, so the published values can differ from the state
  recorded after the callback (R1 finding 1).

Numbers in event files are parsed with `strtod`, so `inf`, `-inf`, and `nan`
are delivered as such. (`istream >> float` would silently give 0; the
independent check caught this during development.)

## Licensing

`reef_estimator` is MIT-licensed (`LICENSE` at `e4179f48`; the file is absent
at `95987b51`). `reef_msgs` has no license file at `7fb63ff9`. The original
sources are **not** copied into this repository. They are extracted from the
pinned upstream Git objects at build time, into `build/` (ignored by Git).

## Controller reference harness (P06)

`baseline/control/build_control_reference.sh` builds
`build/baseline/control/control_ref` from `reef_control` `12237b76`
(`controller.cpp`, `PID.cpp`, `simple_pid.cpp` and their headers) and
`reef_msgs` `7fb63ff9` (`dynamics.h`, for `get_yaw`), extracted from the
pinned Git objects and checked against `baseline/control/provenance.json`.
The event format is described at the top of `control_ref_main.cpp`; the
output columns are listed in `control/control_columns.txt` and are shared
with the port's replay tool. Adaptations (the complete list):

- **C1:** `control/access_prelude.h` is force-included, as A1: library
  headers first, then the access keywords are redefined so the harness can
  read the controller's private and protected state. It also defines
  `BOOST_BIND_GLOBAL_PLACEHOLDERS` (roscpp exposed `_1`, `_2` globally).
- **C2:** stand-ins in `control/shim` (searched before `harness/shim`):
  roscpp with ROS 1 `Time − Time` (`Duration` normalized as roscpp_core,
  `toSec() = sec + 1e−9·nsec`), private `~` parameters, roscpp `getParam`
  type rules (doubles accept integers, booleans only booleans), subscribers
  that store callbacks; dynamic_reconfigure's `Server` (defaults, then the
  node's parameters, then clamping to the cfg ranges; the callback runs at
  `setCallback` and at each runtime change) and the generated
  `GainsConfig` (fields, defaults, and ranges of `cfg/Gains.cfg`); the
  legacy message layouts (`rosflight_msgs/Command` with `x, y, z, F`,
  `Status`, `reef_msgs/DesiredState`, `nav_msgs/Odometry`); a
  `geometry_msgs/Quaternion` that default-constructs to zeros, as in ROS 1.
- **C3:** the controller object is constructed in zeroed storage (and the
  harness is built with `-fno-lifetime-dse`), so `PIDController::theta`,
  which the original reads before writing when `face_target` is set, is 0.
  The port initializes it to 0 (CONTROL_CHAIN.md K10).

`ROS_ASSERT` failures (a missing required parameter) exit with status 3.
The ROS 2 `rc_raw` subscription of the original had an empty callback and
is not driven.
