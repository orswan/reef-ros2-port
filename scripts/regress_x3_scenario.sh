#!/usr/bin/env bash
# Regression checks for scripts/run_x3_scenario.sh (container terminal, ~7 min):
#   scripts/regress_x3_scenario.sh [--no-gui]
# Each case runs with its own GZ_PARTITION tag. Processes are identified by
# that tag and signalled individually (scripts/test_lib.sh). Existing display
# services are only observed. Case outputs go to log/checks/regress_x3_<time>/.
# Exit status: 0 if every case matched its expectation, 1 otherwise.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
source "$(dirname "${BASH_SOURCE[0]}")/test_lib.sh"
reef_reexec_clean "$@"
reef_setup_env
set -m  # background jobs get their own process group; SIGINT is not ignored

R="$REEF_ROOT/scripts/run_x3_scenario.sh"
gui=1
[[ "${1:-}" == --no-gui ]] && gui=0
out="$REEF_ROOT/log/checks/regress_x3_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$out"
results=()
failures=0
tags=()

record() {  # name expected actual seconds note
  local verdict=PASS
  [[ "$3" =~ ^($2)$ ]] || { verdict=FAIL; failures=$((failures + 1)); }
  results+=("$(printf '%-46s expect=%-4s got=%-10s %4ss  %s  %s' "$1" "$2" "$3" "$4" "$verdict" "$5")")
  echo ">>> ${results[-1]}"
}
alive() { local st; st="$(ps -o stat= -p "$1" 2>/dev/null)" && [[ "$st" != Z* ]]; }
wait_exit() {  # pid seconds -> child's exit status, or 255 if still running
  local rc=0
  for _ in $(seq $(( $2 * 5 ))); do alive "$1" || break; sleep 0.2; done
  alive "$1" && return 255
  wait "$1" || rc=$?
  return $rc
}
new_tag() { tag="reef_rx3_$$_${RANDOM}${RANDOM}"; tags+=("$tag"); }   # sets $tag; no subshell, so cleanup sees it
display_pids() { pgrep -x Xvfb; pgrep -x x11vnc; pgrep -f "^/usr/bin/python3 /usr/bin/websockify"; }

# shellcheck disable=SC2317  # reached only via the EXIT trap
on_exit() {
  trap '' INT TERM
  local t j
  for j in $(jobs -p); do kill -TERM -- "-$j" 2>/dev/null; done
  for t in "${tags[@]}"; do reap_tagged "$t" || echo "WARN tagged processes remain for $t"; done
}
trap on_exit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

run_case() {  # name expected env... -- : foreground run of the scenario
  local name="$1" expected="$2"; shift 2
  local dir="$out/${name//[^A-Za-z0-9]/_}" tag t0=$SECONDS rc=0
  new_tag
  env REEF_X3_OUT="$dir" REEF_TEST_GZ_PARTITION="$tag" "$@" "$R" </dev/null >"$dir.log" 2>&1 || rc=$?
  local left; left="$(tagged_pids "$tag" | wc -l)"
  (( left == 0 )) || { rc="$rc+leak"; reap_tagged "$tag"; }
  record "$name" "$expected" "$rc" "$(( SECONDS - t0 ))" \
    "$(grep -E '^(== PASS|FAIL|ANALYSIS FAILED)' "$dir.log" | head -2 | tr '\n' ' ') tagged left=$left"
  LAST_DIR="$dir"
}

disrupt_case() {  # name action expected: interrupt a running flight
  local name="$1" action="$2" expected="$3"
  local dir="$out/${name//[^A-Za-z0-9]/_}" tag t0=$SECONDS
  new_tag
  REEF_X3_OUT="$dir" REEF_TEST_GZ_PARTITION="$tag" "$R" </dev/null >"$dir.log" 2>&1 &
  local chk=$! flying=0
  for _ in $(seq 600); do   # up to 120 s until the "forward" phase is under way
    grep -q "phase forward" "$dir/launch.log" 2>/dev/null && { flying=1; break; }
    alive "$chk" || break
    sleep 0.2
  done
  if (( ! flying )); then
    record "$name" "$expected" "setup" "$(( SECONDS - t0 ))" "flight never reached 'forward'"
    kill -TERM "$chk" 2>/dev/null; wait_exit "$chk" 60; reap_tagged "$tag"; return
  fi
  case "$action" in
    KILL_GZ)   # the owned gz server: tagged and matching the headless server command
      local g="" p
      for _ in $(seq 20); do   # retry the scan for up to ~10 s
        for p in $(tagged_pids "$tag"); do
          [[ "$(tr '\0' ' ' 2>/dev/null <"/proc/$p/cmdline")" == "gz sim -r -s "* ]] && g="$p"
        done
        [[ -n "$g" ]] && break
        sleep 0.5
      done
      if [[ -z "$g" ]] || ! has_tag "$g" "$tag" || ! kill -KILL "$g"; then
        record "$name" "$expected" "setup" "$(( SECONDS - t0 ))" "owned gz server not found or not signalled"
        kill -TERM "$chk" 2>/dev/null; wait_exit "$chk" 60; reap_tagged "$tag"; return
      fi
      echo "    killed owned gz server pid $g (tag $tag)" ;;
    TERM) kill -TERM "$chk" ;;
    INT)  kill -INT -- "-$chk" ;;   # Ctrl-C: the checker's process group
  esac
  local rc=0; wait_exit "$chk" 60 || rc=$?
  local left; left="$(tagged_pids "$tag" | wc -l)"
  (( left == 0 )) || { rc="$rc+leak"; reap_tagged "$tag"; }
  local outcome; outcome="$(grep -A2 '^outcome:' "$dir/manifest.yaml" 2>/dev/null | tr -s ' \n' ' ')"
  record "$name" "$expected" "$rc" "$(( SECONDS - t0 ))" \
    "$(grep -E '^(FAIL|INTERRUPTED)' "$dir.log" | head -1); tagged left=$left; manifest ${outcome:-none}"
}

echo "X3 scenario regression output: $out"
disp_before="$(display_pids | sort | tr '\n' ' ')"

# 1. Pinned assets verify without network access.
t0=$SECONDS; rc=0
env HTTPS_PROXY=http://127.0.0.1:9 python3 "$REEF_ROOT/scripts/setup_assets.py" --verify >"$out/assets.log" 2>&1 || rc=$?
record "1 assets verify (offline)" 0 "$rc" "$(( SECONDS - t0 ))" "$(head -1 "$out/assets.log")"

# 2. Headless scenario: completes, records, analysis passes, nothing fetched.
run_case "2 headless scenario" 0
headless_dir="$LAST_DIR"
n_png="$(find "$headless_dir/analysis" -name '*.png' 2>/dev/null | wc -l)"
fetched="$(awk '/fuel_files_fetched/{gsub(/\x27/,"",$2); print $2}' "$headless_dir/manifest.yaml" 2>/dev/null)"
record "2 artifacts: 3 plots, manifest, 0 Fuel fetches" 0 \
  "$([[ "$n_png" == 3 && -s "$headless_dir/manifest.yaml" && "$fetched" == 0 ]] && echo 0 || echo 1)" 0 \
  "plots=$n_png fuel_files_fetched=${fetched:-?}"

# 3. GUI scenario on the existing desktop, with a snapshot of the browser view.
if (( gui )) && "$REEF_ROOT/scripts/check_display.sh" >/dev/null 2>&1; then
  dir="$out/3_GUI_scenario"; new_tag; t0=$SECONDS
  REEF_X3_OUT="$dir" REEF_TEST_GZ_PARTITION="$tag" "$R" --gui </dev/null >"$dir.log" 2>&1 &
  g=$!
  for _ in $(seq 600); do grep -q "phase forward" "$dir/launch.log" 2>/dev/null && break; alive "$g" || break; sleep 0.2; done
  sleep 2
  snap="$(python3 "$REEF_ROOT/scripts/vnc_snapshot.py" "$out/gui_flight.png" --port 5900 2>&1)"
  win="$(timeout 3 xwininfo -root -tree 2>/dev/null | grep -c '"Gazebo Sim"')"
  rc=0; wait_exit "$g" 400 || rc=$?
  left="$(tagged_pids "$tag" | wc -l)"; (( left == 0 )) || { rc="$rc+leak"; reap_tagged "$tag"; }
  record "3 GUI scenario" 0 "$rc" "$(( SECONDS - t0 ))" "Gazebo windows=$win; $snap; tagged left=$left"
  gui_dir="$dir"
else
  record "3 GUI scenario" 0 skipped 0 "--no-gui or display not ready"
  gui_dir=""
fi

# 4. Same seed, same IMU noise: settle-phase IMU samples of runs 2 and 3. The
#    noise-free signal can differ slightly between runs (the controller starts
#    driving the rotors when its first command arrives, which depends on ROS
#    timing), so compare the differences against the noise level rather than
#    requiring bitwise equality.
if [[ -n "$gui_dir" ]]; then
  t0=$SECONDS
  cmp="$(python3 - "$headless_dir/bag" "$gui_dir/bag" <<'EOF' 2>&1
import sys, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
def imu(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    out = {}
    while r.has_next():
        t, d, _ = r.read_next()
        if t == '/x3/imu':
            m = deserialize_message(d, Imu)
            s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            if s < 5.0:   # settle phase: vehicle on the ground, no commanded motion yet
                a, w = m.linear_acceleration, m.angular_velocity
                out[round(s, 4)] = (a.x, a.y, a.z, w.x, w.y, w.z)
    return out
import numpy as np
a, b = imu(sys.argv[1]), imu(sys.argv[2])
common = sorted(set(a) & set(b))
d = np.array([np.subtract(a[t], b[t]) for t in common])
rms = lambda x: float(np.sqrt(np.mean(x ** 2)))
acc, gyr = rms(d[:, :3]), rms(d[:, 3:])
# Independent noise would give RMS differences of sqrt(2)*sigma (0.028 / 0.0028);
# require < 10% of sigma (0.002 / 0.0002).
print(f'{len(common)} common samples; RMS diff accel {acc:.1e} m/s^2, gyro {gyr:.1e} rad/s '
      f'(max {np.abs(d[:, :3]).max():.1e} / {np.abs(d[:, 3:]).max():.1e}); independent noise ~2.8e-02 / 2.8e-03')
sys.exit(0 if len(common) > 500 and acc < 0.002 and gyr < 0.0002 else 1)
EOF
)"; rc=$?
  record "4 seed reproducibility (IMU, same seed)" 0 "$rc" "$(( SECONDS - t0 ))" "$cmp"
fi

# 5. A missing required stream makes the run fail (analysis reports it).
run_case "5 missing range stream" 1 REEF_X3_ENABLE_RANGE=0
record "5 analysis names the missing stream" 0 \
  "$(grep -q 'FAIL stream /x3/range: MISSING' "$LAST_DIR/analysis.log" 2>/dev/null && echo 0 || echo 1)" 0 "analysis.log"

# 6. Missing assets are caught before any simulation starts.
empty="$(mktemp -d)"
run_case "6 missing assets" 2 REEF_ASSETS_DIR="$empty"
rmdir "$empty" 2>/dev/null

# 7-9. Failure and interruption during flight: nonzero status, nothing left behind.
disrupt_case "7 gz server killed mid-flight" KILL_GZ 2
disrupt_case "8 SIGTERM mid-flight" TERM 143
disrupt_case "9 SIGINT (Ctrl-C to group) mid-flight" INT 130

# 10. Replay: the recorded /clock drives use_sim_time consumers. Isolated like
#     the clock checker: its own ROS domain, /clock remapped to a per-run
#     topic, exactly one publisher (the player), and replayed times inside the
#     bag's range. Otherwise a simulation left running could answer instead.
t0=$SECONDS
rdomain=$(( RANDOM % 101 + 1 ))
rtopic="/reef_replay_$$_${RANDOM}/clock"
bag_end="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["phases"][-1]["t_end"])' \
  "$headless_dir/scenario_result.json" 2>/dev/null || echo 0)"
ROS_DOMAIN_ID=$rdomain ros2 bag play "$headless_dir/bag" --exclude-topics /x3/cmd_vel \
  --disable-keyboard-controls --remap "/clock:=$rtopic" </dev/null >"$out/replay.log" 2>&1 &
play=$!
replay="$(ROS_DOMAIN_ID=$rdomain python3 "$REEF_ROOT/scripts/clock_check.py" 3 20 --topic "$rtopic" \
  --publisher-node rosbag2_player 2>&1 | grep -E '^(PASS|FAIL)|sim time|publishers')"
t_last="$(echo "$replay" | sed -n 's/.*clock sim time.*-> \([0-9.]*\) s.*/\1/p')"
rc=1
[[ "$replay" == *"PASS sim time"* ]] && python3 -c 'import sys; sys.exit(0 if 0 < float(sys.argv[1]) <= float(sys.argv[2]) + 1 else 1)' \
  "${t_last:-0}" "$bag_end" 2>/dev/null && rc=0
[[ "$rc" == 0 ]] || rc=1
kill -INT -- "-$play" 2>/dev/null; wait_exit "$play" 20 >/dev/null || kill -KILL -- "-$play" 2>/dev/null
record "10 replay drives sim time from the bag" 0 "$rc" "$(( SECONDS - t0 ))" \
  "$(echo "$replay" | tr '\n' ' ') (bag ends at ${bag_end} s)"

disp_after="$(display_pids | sort | tr '\n' ' ')"
record "11 display services untouched" 0 "$([[ "$disp_before" == "$disp_after" ]] && echo 0 || echo 1)" 0 "$disp_after"

echo
echo "===== Summary ($out) ====="
printf '%s\n' "${results[@]}"
exit $(( failures > 0 ))
