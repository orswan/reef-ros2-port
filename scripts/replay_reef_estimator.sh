#!/usr/bin/env bash
# Replay a recorded X3 run through the REEF adapter and estimator on a ROS graph.
#   scripts/replay_reef_estimator.sh RUN_DIR [--rate R]
# Output: RUN_DIR/reef_replay_<time>_<id>/ with bag/ (replayed inputs and REEF
# outputs), x3_scenario.yaml and scenario_result.json (copied), sources.json,
# launch.log, analysis_reef/.
#
# Isolation: a ROS domain of its own and LOCALHOST discovery; the session is
# owned and torn down on exit (sim_lib.sh). The replay cannot consume an
# unrelated clock or sensor stream: before playback no publisher may exist in
# the domain for /clock, for any played topic, or for any REEF input topic;
# during playback each played topic has exactly one publisher, the bag player,
# and each REEF input exactly one, its adapter (recorded in sources.json). No
# Gazebo is started. For a deterministic replay without any ROS graph use
# `ros2 run reef_sim x3_reef_offline RUN_DIR`.
#
# Exit: 0 replay completed and the analysis limits were met, 1 analysis
# failed or a /clock check failed, 2 setup failed, 124 timeout, 130/143
# interrupted.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
source "$(dirname "${BASH_SOURCE[0]}")/sim_lib.sh"
reef_reexec_clean "$@"
reef_setup_env
fail() { echo "FAIL $2"; exit "$1"; }
say() { echo "== $*"; }

run="${1:-}"; shift || true
rate=1.0
while (( $# )); do
  case "$1" in
    --rate) rate="${2:-}"; shift 2 || fail 2 "--rate needs a value" ;;
    *) fail 2 "unknown argument: $1" ;;
  esac
done
[[ -n "$run" ]] || fail 2 "usage: $0 RUN_DIR [--rate R]"
[[ "$rate" =~ ^[0-9]+([.][0-9]+)?$ ]] || fail 2 "invalid --rate '$rate'"
run="$(cd "$run" && pwd)"
[[ -f "$run/bag/metadata.yaml" ]] || fail 2 "no finalized bag at $run/bag"
[[ -f "$run/scenario_result.json" && -f "$run/x3_scenario.yaml" ]] || fail 2 "$run lacks scenario_result.json or x3_scenario.yaml"

tree="$(python3 "$REEF_ROOT/scripts/colcon_tree.py")"
say "building reef_sim and the REEF estimator in ${tree#"$REEF_ROOT"/}"
mkdir -p "$REEF_ROOT/log"
(cd "$REEF_ROOT" && colcon --log-base "$tree/log" build --base-paths src --symlink-install \
  --build-base "$tree/build" --install-base "$tree/install" --packages-up-to reef_sim \
  >"$REEF_ROOT/log/reef_replay_build.log" 2>&1) || { tail -n 20 "$REEF_ROOT/log/reef_replay_build.log"; fail 2 "colcon build failed"; }
set +u
# shellcheck disable=SC1091  # colcon setup file, generated at build time
source "$tree/install/setup.bash"
set -u

token="$(printf '%04x%04x%04x' $$ $RANDOM $RANDOM)"
out="$run/reef_replay_$(date +%Y%m%d_%H%M%S)_$token"
mkdir -p "$out"
cp "$run/x3_scenario.yaml" "$run/scenario_result.json" "$out/"
export ROS_DOMAIN_ID="${REEF_REPLAY_DOMAIN:-$(( RANDOM % 101 + 1 ))}"
export GZ_PARTITION="reef_replay_$token"   # no Gazebo here; lets sim_env_matches identify our processes
duration="$(python3 -c 'import sys,yaml; print(int(yaml.safe_load(open(sys.argv[1]))["rosbag2_bagfile_information"]["duration"]["nanoseconds"]/1e9))' "$run/bag/metadata.yaml")"

played=(/clock /x3/truth/odom /x3/imu /x3/range /x3/scenario/phase)
reef_inputs=(/x3/reef/imu/data /x3/reef/mocap_velocity/body_level_frame)
pre="$(python3 "$REEF_ROOT/scripts/topic_sources.py" --wait 2 "${played[@]}" "${reef_inputs[@]}")"
python3 -c 'import json,sys; d=json.loads(sys.argv[1]); sys.exit(0 if not any(d.values()) else 1)' "$pre" \
  || fail 2 "domain $ROS_DOMAIN_ID already has publishers of replay inputs: $pre (choose another REEF_REPLAY_DOMAIN)"

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
  echo "replay directory: $out"
  exit "$rc"
}
trap cleanup EXIT
normal_traps

say "replay $run/bag (sim time ${duration} s, rate $rate) in ROS_DOMAIN_ID=$ROS_DOMAIN_ID; no foreign clock or input publishers"
defer_traps
sim_start_session "$out/launch.log" ros2 launch reef_sim reef_replay.launch.py \
  "bag:=$run/bag" "output_dir:=$out" "rate:=$rate"
[[ -n "$sid" ]] || { normal_traps; fail 2 "replay session did not start"; }
normal_traps
exit_for_pending

# During playback (it starts after a 3 s delay): each played topic only from
# the bag player, each REEF input only from the adapter.
sleep 6
sim_alive "$launch_pid" || fail 2 "replay launch exited early (see $out/launch.log)"
during="$(python3 "$REEF_ROOT/scripts/topic_sources.py" --wait 2 "${played[@]}" "${reef_inputs[@]}")"
printf '{"before_playback": %s, "during_playback": %s}\n' "$pre" "$during" > "$out/sources.json"
python3 - "$during" "${#played[@]}" <<'PY' || fail 1 "unexpected publishers during playback: $during"
import json, sys
d, n = json.loads(sys.argv[1]), int(sys.argv[2])
topics = list(d)
ok = all(len(d[t]) == 1 and d[t][0].split('/')[-1].startswith('rosbag2_player') for t in topics[:n] if t != '/x3/scenario/phase') \
    and d['/x3/reef/imu/data'] == ['/x3_imu_adapter'] \
    and d['/x3/reef/mocap_velocity/body_level_frame'] == ['/reef_adapter']
sys.exit(0 if ok else 1)
PY
say "sources during playback: $during"

limit="$(python3 -c "import sys; print(int(float(sys.argv[1]) / float(sys.argv[2]) + 60))" "$duration" "$rate")"
sim_wait_launch_exit "$limit" || fail 124 "replay did not finish within ${limit} s"
sim_stop_session
sid=""
[[ -f "$out/bag/metadata.yaml" ]] || fail 2 "replay bag was not finalized"

say "REEF vertical estimate vs truth (replayed, idealized inputs)"
rc=0
ros2 run reef_sim analyze_reef_vertical "$out" | tee "$out/analysis_reef.log" || rc=1
[[ "${PIPESTATUS[0]}" == 0 ]] || rc=1
exit "$rc"
