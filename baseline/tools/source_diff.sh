#!/usr/bin/env bash
# Source diff of the ROS 2 port against the original REEF files (for R1).
#   baseline/tools/source_diff.sh [OUT_DIR]        (default build/r1)
# Writes OUT_DIR/source_diff.patch (unified diffs, original -> port) and
# OUT_DIR/source_diff_summary.txt (lines added/removed per file). The
# originals are the files as imported by `git subtree`:
#   reef_estimator  c80f824  (master e4179f48 minus docs/Partial_Update.pdf)
#   reef_msgs       d8e3096  (7fb63ff)
#   reef_control    12237b76 (imported unchanged by git subtree in 8d32197)
# Exit: 0 written, 2 a required commit or file is missing.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
out="${1:-$ROOT/build/r1}"
mkdir -p "$out"
E=c80f824 M=d8e3096 R=12237b76b85d755364275ed8c1147f15d8ddf15c
pairs=(
  "$E:src/reef_estimator/upstream/include/estimator.h|HEAD:src/reef_estimator/include/reef_estimator/estimator.h"
  "$E:src/reef_estimator/upstream/src/estimator.cpp|HEAD:src/reef_estimator/src/estimator.cpp"
  "$E:src/reef_estimator/upstream/include/z_estimator.h|HEAD:src/reef_estimator/include/reef_estimator/z_estimator.h"
  "$E:src/reef_estimator/upstream/src/z_estimator.cpp|HEAD:src/reef_estimator/src/z_estimator.cpp"
  "$E:src/reef_estimator/upstream/include/xy_estimator.h|HEAD:src/reef_estimator/include/reef_estimator/xy_estimator.h"
  "$E:src/reef_estimator/upstream/src/xy_estimator.cpp|HEAD:src/reef_estimator/src/xy_estimator.cpp"
  "$E:src/reef_estimator/upstream/include/xyz_estimator.h|HEAD:src/reef_estimator/include/reef_estimator/xyz_estimator.h"
  "$E:src/reef_estimator/upstream/src/xyz_estimator.cpp|HEAD:src/reef_estimator/src/xyz_estimator.cpp"
  "$E:src/reef_estimator/upstream/include/sensor_manager.h|HEAD:src/reef_estimator/include/reef_estimator/sensor_manager.h"
  "$E:src/reef_estimator/upstream/src/sensor_manager.cpp|HEAD:src/reef_estimator/src/sensor_manager.cpp"
  "$E:src/reef_estimator/upstream/src/reef_estimator_node.cpp|HEAD:src/reef_estimator/src/reef_estimator_node.cpp"
  "$M:src/reef_msgs/include/reef_msgs/dynamics.h|HEAD:src/reef_msgs/include/reef_msgs/dynamics.h"
  "$M:src/reef_msgs/src/dynamics.cpp|HEAD:src/reef_msgs/src/dynamics.cpp"
  "$M:src/reef_msgs/include/reef_msgs/matrix_operation.h|HEAD:src/reef_msgs/include/reef_msgs/matrix_operation.h"
  "$R:include/simple_pid.h|HEAD:src/reef_control/include/reef_control/simple_pid.h"
  "$R:src/simple_pid.cpp|HEAD:src/reef_control/src/simple_pid.cpp"
  "$R:include/controller.h|HEAD:src/reef_control/include/reef_control/controller.h"
  "$R:src/controller.cpp|HEAD:src/reef_control/src/controller.cpp"
  "$R:include/PID.h|HEAD:src/reef_control/include/reef_control/PID.h"
  "$R:src/PID.cpp|HEAD:src/reef_control/src/PID.cpp"
  "$R:src/pid_node.cpp|HEAD:src/reef_control/src/reef_control_node.cpp"
)
for m in DeltaToVel DesiredState DesiredVector XYEstimate ZEstimate XYZEstimate XYDebugEstimate ZDebugEstimate XYZDebugEstimate; do
  pairs+=("$M:src/reef_msgs/msg/$m.msg|HEAD:src/reef_msgs/msg/$m.msg")
done
: > "$out/source_diff.patch"
printf '%-58s %6s %6s\n' "file (port)" "+lines" "-lines" > "$out/source_diff_summary.txt"
cd "$ROOT"
for p in "${pairs[@]}"; do
  a="${p%%|*}" b="${p##*|}"
  git cat-file -e "$a" 2>/dev/null || { echo "missing original $a"; exit 2; }
  git cat-file -e "$b" 2>/dev/null || { echo "missing port $b"; exit 2; }
  git diff --no-color "$a" "$b" >> "$out/source_diff.patch" || true
  read -r add del < <(git diff --numstat "$a" "$b" | awk '{print $1, $2}')
  printf '%-58s %6s %6s\n' "${b#HEAD:}" "${add:-0}" "${del:-0}" >> "$out/source_diff_summary.txt"
done
echo "wrote $out/source_diff.patch ($(wc -l < "$out/source_diff.patch") lines) and $out/source_diff_summary.txt"
