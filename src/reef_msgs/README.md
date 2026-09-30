#REEF_MSGS

This package is to support various projects at the REEF including REEF Estimator, position_to_velocity etc.

## ROS 2 port (P03)

Ported from UF REEF `reef_msgs` `7fb63ff93269040316b71d346dbc32919da1f63d`
(2019-09-11, Prashant Ganesh), the revision pinned by `reef_estimator_bundle`.
The original history was imported with `git subtree`, so `git log` and
`git blame` on this directory reach the original commits. Only what
`reef_estimator` master `e4179f48` uses is ported; the rest stays in the
history until a later milestone needs it.

### Messages

| Ported | Change from ROS 1 |
|---|---|
| `XYZEstimate`, `XYEstimate`, `ZEstimate`, `XYZDebugEstimate`, `XYDebugEstimate` | `Header` → `std_msgs/Header` (no `seq`); unit/frame comments added |
| `ZDebugEstimate` | as above, and `P` → `p` |
| `DeltaToVel` | as above, and `S_upper_bound` → `s_upper_bound`, `S_lower_bound` → `s_lower_bound` |

ROS 2 requires lower-case field names, which forces the three renames. Field
order, types, and fixed array sizes are unchanged (`test/test_messages.py`).
Not ported: `DesiredState`, `DesiredVector` (controller), `SyncEstimateError`,
`SyncVerifyEstimates` (analysis tools).

### Helpers

| Library | Contents | Depends on |
|---|---|---|
| `reef_msgs_helpers` | `dynamics.h`: `quaternion_to_rotation`, `roll_pitch_yaw_from_rotation321`, `skew` (bodies unchanged). `matrix_operation.h`: `vectorToMatrix`, `vectorToDiagMatrix`, `matrixToArray`, `importMatrixFromVector`, plus validation (`covarianceError`, `rangeError`) | Eigen only |
| `reef_msgs_parameters` | `parameters.hpp`: `importMatrixFromParameter`, `getNumberArrayParameter` (replace ROS 1 `importMatrixFromParamServer`) | rclcpp |
| header only | `ros_conversions.hpp`: `quaternion_to_rotation(geometry_msgs::msg::Quaternion)` | geometry_msgs |

Note on conventions: `quaternion_to_rotation(q)` returns the transpose of the
standard rotation matrix of `q`. With `q` the orientation of the body in NED,
the result maps NED vectors into the body frame (C_NED→body).

Matrix parameters keep the legacy meaning of the legal forms: rows·cols
values fill the matrix row-major (checked first), and n values on an n×n
matrix fill the diagonal. Behaviour that differs on purpose, all for input the
original accepted silently (BASELINE_DECISION.md D10):

| Input | Original | Port |
|---|---|---|
| parameter missing | zero matrix, warning | error |
| wrong number of values | matrix left uninitialized, error log | error, matrix unchanged |
| n values on a non-square n×m matrix | out-of-bounds write | error |
| empty list, NaN/inf | accepted | error |
| integer list (`[0, 0, 0]`) | accepted (roscpp converts) | accepted, converted exactly |

Not ported: `matrixToVector` (no return statement, undefined behaviour),
`verifyDimensions`, `loadTransform`, and the unused `dynamics` functions.

### Tests

- `test_helpers_legacy`, `test_ros_conversions`: bit-identical to vectors
  recorded from the pinned legacy code (`test/data/legacy_helper_vectors.txt`,
  written by `baseline/helper_vectors.sh`, never from port output).
- `test_matrix_operation`, `test_parameters`: import rules, rejection cases,
  ROS 2 parameter types, a params file.
- `test_messages`: generated fields against the legacy definitions.
- `test_ros_independence`: the helpers compile and run with no ROS on the
  include path (with a negative control).

### License

The original package declares `<license>TODO</license>` and has no license
file. The bundle that pins it (`reef_estimator_bundle`) is MIT-licensed
(© 2020 University of Florida REEF Autonomous Vehicles Lab). Whether that
covers this submodule is **not confirmed**; `package.xml` keeps `TODO` until
the owner confirms.
