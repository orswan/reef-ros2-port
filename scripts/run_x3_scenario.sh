#!/usr/bin/env bash
# Fly the bounded X3 scenario, record it, and analyze the recording.
#   scripts/run_x3_scenario.sh            # headless
#   scripts/run_x3_scenario.sh --gui      # also show Gazebo on the browser desktop (:99)
#   scripts/run_x3_scenario.sh --estimator  # also run the REEF adapter and ported
#       estimator (vertical filter) beside the truth-fed controller, with the
#       IMU vibration overlay (config/x3_reef_overlay.yaml), and score it
#       (analysis_reef/). REEF is not in the control loop.
# Builds in the per-environment tree (scripts/colcon_tree.py).
# Output: recordings/x3_<time>_<id>/ (manifest.yaml, x3_scenario.yaml, bag/,
# scenario_result.json, launch.log, analysis/). Recordings are ignored by Git.
#
# Knobs: REEF_X3_PARAMS (parameters file; default src/reef_sim/config/x3_scenario.yaml),
# REEF_X3_OUT (run directory), REEF_ASSETS_DIR (verified asset directory,
# default assets/models),
# REEF_TEST_ROS_DOMAIN_ID / REEF_TEST_GZ_PARTITION (default: per run),
# REEF_X3_ENABLE_RANGE=0 (test hook: omit the range stream).
#
# Every run is isolated (per-run GZ_PARTITION and ROS domain; the scenario
# requires exactly one publisher per stream) and offline: Gazebo gets an empty
# Fuel cache and an unreachable proxy, and the run fails if anything was
# fetched. The owned session is torn down on exit, failure, timeout, SIGINT,
# and SIGTERM, as in check_clock_demo.sh.
#
# Exit status: 0 pass, 1 analysis checks failed, 2 setup or owned simulation
# failed, 3 scenario runner failed (startup, stall, foreign publishers),
# 124 timeout, 130 SIGINT, 143 SIGTERM.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
source "$(dirname "${BASH_SOURCE[0]}")/sim_lib.sh"
reef_reexec_clean "$@"
reef_setup_env

headless=true
analyze=1
estimator=false
for arg in "$@"; do
  case "$arg" in
    --gui) headless=false ;;
    --no-analysis) analyze=0 ;;
    --estimator) estimator=true ;;
    *) echo "unknown argument: $arg" >&2; exit 2 ;;
  esac
done
[[ "${REEF_HEADLESS:-}" == 0 ]] && headless=false

fail() { echo "FAIL $2"; exit "$1"; }
say() { echo "== $*"; }

# --- build the package (quick; --symlink-install keeps sources live)
mkdir -p "$REEF_ROOT/log"
# Private tree per environment (the shared build/ may hold another container's
# CMake cache). reef_sim depends on reef_estimator/reef_msgs (--estimator), so
# they are built too; after the first run the build is incremental.
tree="$(python3 "$REEF_ROOT/scripts/colcon_tree.py")"
say "building reef_sim and its dependencies in ${tree#"$REEF_ROOT"/}"
if ! (cd "$REEF_ROOT" && colcon --log-base "$tree/log" build --base-paths src --symlink-install \
      --build-base "$tree/build" --install-base "$tree/install" --packages-up-to reef_sim \
      >"$REEF_ROOT/log/reef_sim_build.log" 2>&1); then
  tail -n 20 "$REEF_ROOT/log/reef_sim_build.log"; fail 2 "colcon build failed"
fi
install_setup="$tree/install/setup.bash"
set +u
# shellcheck disable=SC1090,SC1091
source "$install_setup"
set -u

# --- assets: pinned, verified, local (never fetched during a run)
asset_path="${REEF_ASSETS_DIR:-$REEF_ROOT/assets/models}"
REEF_ASSETS_DIR="$asset_path" python3 "$REEF_ROOT/scripts/setup_assets.py" --verify \
  || fail 2 "assets missing or modified in $asset_path; run scripts/setup_assets.py"
asset_status="verified against src/reef_sim/assets/*.json ($asset_path)"

# --- run directory and parameters
params_src="${REEF_X3_PARAMS:-$REEF_ROOT/src/reef_sim/config/x3_scenario.yaml}"
[[ -f "$params_src" ]] || fail 2 "parameters file not found: $params_src"
token="$(printf '%04x%04x%04x' $$ $RANDOM $RANDOM)"
run_dir="${REEF_X3_OUT:-$REEF_ROOT/recordings/x3_$(date +%Y%m%d_%H%M%S)_$token}"
mkdir -p "$run_dir"
[[ -e "$run_dir/bag" ]] && fail 2 "$run_dir/bag already exists"
cp "$params_src" "$run_dir/x3_scenario.yaml"
if [[ "$estimator" == true ]]; then
  python3 "$REEF_ROOT/scripts/x3_merge_params.py" "$run_dir/x3_scenario.yaml" \
    "$REEF_ROOT/src/reef_sim/config/x3_reef_overlay.yaml" || fail 2 "could not merge the REEF overlay"
fi
read -r startup_timeout duration < <(python3 - "$run_dir/x3_scenario.yaml" <<'EOF'
import sys, yaml
p = yaml.safe_load(open(sys.argv[1]))
s = p['simulation']['ros__parameters']
print(int(s['startup_timeout_s']),
      int(sum(p['scenario_runner']['ros__parameters']['phase_durations']) + 1))
EOF
)

# --- isolation and offline guards, inherited by every process of the run
export ROS_DOMAIN_ID="${REEF_TEST_ROS_DOMAIN_ID:-$(( RANDOM % 101 + 1 ))}"
export GZ_PARTITION="${REEF_TEST_GZ_PARTITION:-reef_x3_$token}"
export GZ_SIM_RESOURCE_PATH="$asset_path"
fuel_cache="$run_dir/.fuel_cache_must_stay_empty"
mkdir -p "$fuel_cache"
export GZ_FUEL_CACHE_PATH="$fuel_cache"
export HTTP_PROXY=http://127.0.0.1:9 HTTPS_PROXY=http://127.0.0.1:9 http_proxy=http://127.0.0.1:9 https_proxy=http://127.0.0.1:9
unset NO_PROXY no_proxy
enable_range=true
[[ "${REEF_X3_ENABLE_RANGE:-1}" == 0 ]] && enable_range=false

python3 "$REEF_ROOT/scripts/x3_manifest.py" start "$run_dir" \
  "asset_verification=$asset_status" "headless=$headless" "ros_domain_id=$ROS_DOMAIN_ID" \
  "gz_partition=$GZ_PARTITION" "enable_range=$enable_range" "estimator=$estimator" \
  "command=scripts/run_x3_scenario.sh $*"

# The Gazebo server always runs headless (-s); the GUI, if requested, is a
# separate optional viewer and not required for a valid recording.
required=("^gz sim -r -s " "parameter_bridge" "imu_noise" "scenario_runner" "ros2 bag record")
if [[ "$headless" == false ]]; then
  if ! display_report="$("$REEF_ROOT/scripts/check_display.sh" 2>&1)"; then
    echo "$display_report"
    fail 2 "browser desktop not ready (scripts/check_display.sh output above)"
  fi
fi
gui_seen=0
[[ "$enable_range" == true ]] && required+=("range_sensor")
[[ "$estimator" == true ]] && required+=("reef_estimator_node" "reef_adapter" "x3_imu_adapter")

# --- cleanup and signals
launch_pid="" sid="" pending=""
# shellcheck disable=SC2317  # reached only via traps
on_int() { echo "INTERRUPTED (SIGINT)"; exit 130; }
# shellcheck disable=SC2317  # reached only via traps
on_term() { echo "INTERRUPTED (SIGTERM)"; exit 143; }
# shellcheck disable=SC2317  # reached only via the EXIT trap
cleanup() {
  local rc=$?
  trap '' INT TERM
  sim_stop_session
  if [[ -n "$sid" ]] && pgrep -s "$sid" >/dev/null; then echo "WARN processes remain in session $sid" >&2; fi
  if [[ -f "$run_dir/manifest.yaml" ]] && ! grep -q '^outcome:' "$run_dir/manifest.yaml"; then
    python3 "$REEF_ROOT/scripts/x3_manifest.py" finish "$run_dir" "exit_status=$rc" \
      "result=$( ((rc == 0)) && echo pass || echo fail)" >/dev/null 2>&1 || true
  fi
  if (( rc != 0 )) && [[ -f "$run_dir/launch.log" ]]; then
    echo "--- last 25 lines of $run_dir/launch.log"; tail -n 25 "$run_dir/launch.log"
  fi
  echo "run directory: $run_dir"
  exit "$rc"
}
trap cleanup EXIT
normal_traps

# --- start the owned simulation; signals wait until its session is registered
defer_traps
sim_start_session "$run_dir/launch.log" ros2 launch reef_sim x3_scenario.launch.py \
  "output_dir:=$run_dir" "params_file:=$run_dir/x3_scenario.yaml" "headless:=$headless" \
  "enable_range:=$enable_range" "with_estimator:=$estimator"
[[ -n "$sid" ]] || { normal_traps; fail 2 "simulation session did not start"; }
normal_traps
exit_for_pending
say "run $run_dir"
say "session $sid, ROS_DOMAIN_ID=$ROS_DOMAIN_ID, GZ_PARTITION=$GZ_PARTITION, headless=$headless"

# --- monitor until the scenario runner reports; fail fast if the owned run dies
limit=$(( startup_timeout + duration * 4 + 60 ))   # tolerates real-time factor down to ~0.25
deadline=$(( SECONDS + limit ))
declare -A seen=()
result="$run_dir/scenario_result.json"
while [[ ! -s "$result" ]]; do
  sim_alive "$launch_pid" || fail 2 "owned launch exited before the scenario finished"
  for p in "${required[@]}"; do
    if pid="$(pgrep -s "$sid" -f "$p" | head -1)" && [[ -n "$pid" ]]; then
      if [[ -z "${seen[$p]:-}" ]]; then
        sim_env_matches "$pid" || fail 2 "'$p' (pid $pid) lacks this run's ROS_DOMAIN_ID/GZ_PARTITION"
        seen[$p]=1
      fi
    elif [[ -n "${seen[$p]:-}" ]]; then
      [[ -s "$result" ]] && break 2   # the runner finished while we looked
      fail 2 "required process '$p' exited"
    fi
  done
  if [[ "$headless" == false ]] && (( ! gui_seen )) && pgrep -s "$sid" -f "^gz sim -g" >/dev/null; then gui_seen=1; fi
  (( SECONDS <= deadline )) || fail 124 "timed out after ${limit}s"
  sleep 0.2
done
for p in "${required[@]}"; do
  [[ -n "${seen[$p]:-}" ]] || [[ "$p" == scenario_runner ]] || fail 2 "required process '$p' never started"
done

# --- orderly shutdown: the launch stops itself when the runner exits, and the
#     recorder finalizes the bag on SIGINT. ros2 launch signals only the `gz`
#     Ruby wrapper, so the gz server itself can outlive it; the session teardown
#     then stops such stragglers.
sim_wait_launch_exit 40 || echo "WARN launch did not exit within 40 s; stopping session"
sim_stop_session
sid=""

status_code="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["exit_code"])' "$result")"
status_text="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["status"])' "$result")"
# An "interrupted" runner means the launch shut down around it (a launched
# process exited), not that the runner detected a problem itself.
[[ "$status_code" == 130 ]] && fail 2 "simulation shut down during the scenario (a launched process exited)"
[[ "$status_code" == 0 ]] || fail 3 "scenario runner: $status_text"
[[ -f "$run_dir/bag/metadata.yaml" ]] || fail 2 "bag was not finalized (no metadata.yaml)"
fetched="$(find "$fuel_cache" -type f | wc -l)"
(( fetched == 0 )) || fail 2 "Gazebo fetched $fetched file(s) from Fuel; the scenario must run offline"
rmdir "$fuel_cache" 2>/dev/null || true
say "scenario completed; bag finalized; no Fuel fetches"
if [[ "$headless" == false ]] && (( ! gui_seen )); then echo "WARN the Gazebo GUI viewer never started (recording unaffected)"; fi

rc=0
if (( analyze )); then
  say "analysis"
  ros2 run reef_sim analyze_x3_bag "$run_dir" | tee "$run_dir/analysis.log" || rc=1
  [[ "${PIPESTATUS[0]}" == 0 ]] || rc=1
  if [[ "$estimator" == true ]]; then
    say "REEF vertical estimate vs truth (idealized inputs)"
    ros2 run reef_sim analyze_reef_vertical "$run_dir" | tee "$run_dir/analysis_reef.log" || rc=1
    [[ "${PIPESTATUS[0]}" == 0 ]] || rc=1
  fi
fi
python3 "$REEF_ROOT/scripts/x3_manifest.py" finish "$run_dir" "exit_status=$rc" \
  "result=$( ((rc == 0)) && echo pass || echo 'analysis failed')" "fuel_files_fetched=$fetched"
if (( rc == 0 )); then say "PASS X3 scenario ($run_dir)"; else echo "FAIL analysis checks (see $run_dir/analysis)"; fi
exit "$rc"
