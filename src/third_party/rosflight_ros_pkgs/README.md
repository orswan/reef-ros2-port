# Vendored upstream `rosflight_msgs`

`rosflight_msgs/` is an **unmodified** copy of the ROS 2 message package from
[rosflight/rosflight_ros_pkgs](https://github.com/rosflight/rosflight_ros_pkgs)
at tag `v2.0.1` (commit `cefdb425d39a4c9f8c87cd1f1e367dd32887f478`,
2026-03-17). `LICENSE.md` is the repository's BSD 3-Clause license. The pin
is recorded in `UPSTREAM.json`.

Why a vendored copy: no Jazzy binary of `rosflight_msgs` is available from the
configured apt sources, and the rest of the repository (firmware, `rosflight_io`,
the simulator) is out of scope. Copying only this package keeps the build
offline and small, and it is still the upstream package, not a look-alike.
REEF uses only `msg/RCRaw.msg`. The package's own dependencies
(`builtin_interfaces`, `geometry_msgs`, `std_msgs`, rosidl) come from ROS Jazzy.

Do not edit files here. `scripts/check_vendor.py` recomputes the Git tree ID of
`rosflight_msgs/` from the working files and fails if it differs from
`UPSTREAM.json`. To move the pin, re-vendor from a clone and update
`UPSTREAM.json` in the same commit:

```bash
git -C reference/rosflight_ros_pkgs archive <tag> rosflight_msgs LICENSE.md \
  | tar -x -C src/third_party/rosflight_ros_pkgs
python3 scripts/check_vendor.py --upstream reference/rosflight_ros_pkgs
```
