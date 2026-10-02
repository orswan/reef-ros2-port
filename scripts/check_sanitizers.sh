#!/usr/bin/env bash
# Sanitizer profile (P09, ACCEPTANCE `release`, code quality): build the
# project's own C++ packages with AddressSanitizer + UndefinedBehaviorSanitizer
# (Debug, -fno-omit-frame-pointer, UBSan non-recoverable) in a separate tree,
# <colcon_tree>-asan, and run their gtests. Python tests are not run here: the
# instrumented libraries cannot be loaded into an uninstrumented Python.
#
#   scripts/check_sanitizers.sh
#
# Exit: 0 every gtest passed and no sanitizer report appeared in the test
# logs; 1 a test failed or a sanitizer reported; 2 the build failed or
# nothing ran. Container terminal; no simulation is started.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env

PKGS=(reef_msgs reef_estimator reef_control reef_fc_standin reef_x3_adapter rgbd_to_velocity reef_rgbd_odometry)
SAN="-fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined"
tree="$(python3 "$REEF_ROOT/scripts/colcon_tree.py")-asan"
mkdir -p "$tree"
cd "$REEF_ROOT" || exit 2
colcon_args=(--log-base "$tree/log")
paths=(--base-paths src --build-base "$tree/build" --install-base "$tree/install")

echo "== sanitizer build (AddressSanitizer + UndefinedBehaviorSanitizer) in ${tree#"$REEF_ROOT"/}"
if ! CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}" colcon "${colcon_args[@]}" build "${paths[@]}" \
    --packages-up-to "${PKGS[@]}" --event-handlers console_cohesion- \
    --cmake-args -DCMAKE_BUILD_TYPE=Debug "-DCMAKE_C_FLAGS=$SAN" "-DCMAKE_CXX_FLAGS=$SAN" \
    "-DCMAKE_EXE_LINKER_FLAGS=-fsanitize=address,undefined" "-DCMAKE_SHARED_LINKER_FLAGS=-fsanitize=address,undefined" \
    >"$tree/build.log" 2>&1; then
  tail -30 "$tree/build.log"
  echo "BLOCKED: sanitizer build failed (${tree#"$REEF_ROOT"/}/build.log)"
  exit 2
fi

echo "== gtests of ${PKGS[*]} (ctest label gtest)"
# stale results and CTest logs of earlier runs must not count
for p in "${PKGS[@]}"; do rm -rf "$tree/build/$p/test_results" "$tree/build/$p/Testing"; done
export ASAN_OPTIONS="halt_on_error=1:detect_leaks=1:abort_on_error=0:strict_string_checks=1"
export UBSAN_OPTIONS="halt_on_error=1:print_stacktrace=1"
colcon "${colcon_args[@]}" test "${paths[@]}" --packages-select "${PKGS[@]}" --ctest-args -L gtest \
  --event-handlers console_cohesion- >"$tree/test.log" 2>&1
summary="$(colcon test-result --test-result-base "$tree/build" --all 2>&1)"
echo "$summary" | tail -1
reports="$(grep -lE 'ERROR: (AddressSanitizer|LeakSanitizer)|runtime error:' -r "$tree/log/latest_test" "$tree/build" \
  --include='*.log' --include='*.xml' --include='LastTest.log' 2>/dev/null | sort -u)"
total="$(echo "$summary" | tail -1 | grep -oE '^Summary: [0-9]+' | grep -oE '[0-9]+')"
if [[ -z "$total" || "$total" == 0 ]]; then
  echo "BLOCKED: no test results (${tree#"$REEF_ROOT"/}/test.log)"
  exit 2
fi
if [[ -n "$reports" ]]; then
  echo "FAIL sanitizer reports in:"
  echo "${reports//"$REEF_ROOT"\//  }"
  exit 1
fi
if ! echo "$summary" | tail -1 | grep -qE ' 0 errors, 0 failures'; then
  echo "$summary" | grep -vE '^$' | tail -15
  echo "FAIL a gtest failed under the sanitizers"
  exit 1
fi
echo "PASS sanitizer profile: $total gtest result(s), no sanitizer report"
exit 0
