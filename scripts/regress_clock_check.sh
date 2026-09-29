#!/usr/bin/env bash
# Regression checks for scripts/check_clock_demo.sh (isolation, liveness,
# cleanup). Every process this script signals is one it started, and ownership
# is verified before signalling. Existing display services are only observed.
#   scripts/regress_clock_check.sh          # all cases (GUI case needs Xvfb :99)
# Exit status: 0 if every case matched its expectation, 1 otherwise.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env
set -m  # background jobs get their own process group; SIGINT is not ignored

C="$REEF_ROOT/scripts/check_clock_demo.sh"
out_dir="$REEF_ROOT/log/checks/regress_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$out_dir"
results=()
failures=0
unrelated_leader="" unrelated_sid=""

record() {  # name expected actual elapsed note
  local verdict=PASS
  [[ "$3" =~ ^($2)$ ]] || { verdict=FAIL; failures=$((failures + 1)); }
  results+=("$(printf '%-44s expect=%-5s got=%-4s %4ss  %s  %s' "$1" "$2" "$3" "$4" "$verdict" "$5")")
  echo ">>> ${results[-1]}"
}

alive() { local st; st="$(ps -o stat= -p "$1" 2>/dev/null)" && [[ "$st" != Z* ]]; }

wait_exit() {  # pid timeout_s -> exit status of our child (or 255 if still alive)
  local pid="$1" t="$2" rc=0
  for _ in $(seq $(( t * 5 ))); do alive "$pid" || break; sleep 0.2; done
  alive "$pid" && return 255
  wait "$pid" || rc=$?
  return $rc
}

display_pids() { pgrep -x Xvfb; pgrep -x x11vnc; pgrep -f "websockify.*8080"; }

start_unrelated_sim() {  # headless demo on plain /clock; stands in for "another simulation"
  local sid_file; sid_file="$(mktemp)"
  ROS_DOMAIN_ID="$1" GZ_PARTITION="$2" REEF_HEADLESS=1 \
    setsid -w bash -c 'echo $$ >"$1"; shift; exec "$@"' _ "$sid_file" \
    "$REEF_ROOT/scripts/run_clock_demo.sh" >"$out_dir/unrelated_sim.log" 2>&1 &
  unrelated_leader=$!
  for _ in $(seq 50); do [[ -s "$sid_file" ]] && break; sleep 0.1; done
  unrelated_sid="$(cat "$sid_file")"; rm -f "$sid_file"
}

stop_owned_session() {  # sid leader_pid: only if the session leader is (or is under) our child
  local sid="$1" leader="$2" sig
  [[ "$sid" =~ ^[0-9]+$ ]] && (( sid > 1 )) || return 0
  if [[ "$sid" != "$leader" && "$(ps -o ppid= -p "$sid" 2>/dev/null | tr -d ' ')" != "$leader" ]]; then
    pgrep -s "$sid" >/dev/null && echo "WARN not signalling session $sid: not owned by pid $leader"
    return 0
  fi
  for sig in INT TERM KILL; do
    pgrep -s "$sid" >/dev/null || break
    pkill "-$sig" -s "$sid"
    for _ in $(seq 25); do pgrep -s "$sid" >/dev/null || break; sleep 0.2; done
  done
  wait "$leader" 2>/dev/null
}

on_exit() {
  trap '' INT TERM
  [[ -n "$unrelated_sid" ]] && stop_owned_session "$unrelated_sid" "$unrelated_leader"
  # Any checker job still running is ours (started below with set -m).
  local j; for j in $(jobs -p); do kill -TERM -- "-$j" 2>/dev/null; done
}
trap on_exit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

run_fg() {  # name expected env... -- command...
  local name="$1" expected="$2"; shift 2
  local envs=(); while [[ "$1" != "--" ]]; do envs+=("$1"); shift; done; shift
  local log="$out_dir/${name//[^A-Za-z0-9]/_}.log" t0=$SECONDS rc=0
  env "${envs[@]}" "$@" >"$log" 2>&1 || rc=$?
  record "$name" "$expected" "$rc" "$(( SECONDS - t0 ))" "$(grep -E '^(PASS|FAIL|INTERRUPTED)' "$log" | head -1)"
}

disrupt_case() {  # name action(TERM|INT|HANG_OBSERVER|KILL_BRIDGE) expected
  local name="$1" sig="$2" expected="$3"
  local log="$out_dir/${name//[^A-Za-z0-9]/_}.log" t0=$SECONDS
  # HANG_OBSERVER: short startup timeout so the checker's deadline (startup +
  # window + 15 s = 20 s) is reached while the stopped observer never exits.
  REEF_HEADLESS=1 REEF_CHECK_SECONDS=$([[ "$sig" == HANG_OBSERVER ]] && echo 3 || echo 30) \
    REEF_STARTUP_TIMEOUT=$([[ "$sig" == HANG_OBSERVER ]] && echo 2 || echo 60) \
    "$C" >"$log" 2>&1 &
  local chk=$! obs="" sid="" note=""
  # Wait (bounded) for the observer and the owned bridge to be running.
  for _ in $(seq 150); do
    sid="$(grep -oP 'session=\K[0-9]+' "$log" 2>/dev/null || true)"
    obs="$(pgrep -P "$chk" -f clock_check.py || true)"
    [[ -n "$sid" && -n "$obs" ]] || { sleep 0.2; continue; }
    [[ "$sig" == HANG_OBSERVER ]] && break  # stop it before its own 2 s timeout
    pgrep -s "$sid" -f parameter_bridge >/dev/null && break
    sleep 0.2
  done
  if [[ -z "$sid" || -z "$obs" ]]; then
    record "$name" "$expected" "setup" "$(( SECONDS - t0 ))" "observer/demo never became active"
    kill -TERM "$chk"; wait_exit "$chk" 20; return
  fi
  [[ "$sig" == HANG_OBSERVER ]] || sleep 2  # observer is inside its measurement window
  # Ownership checks before signalling: observer is the checker's child; demo
  # session leader is the checker's child (or setsid's child under it).
  local sid_parent; sid_parent="$(ps -o ppid= -p "$sid" | tr -d ' ')"
  local sid_grand; sid_grand="$(ps -o ppid= -p "$sid_parent" 2>/dev/null | tr -d ' ')"
  if [[ "$(ps -o ppid= -p "$obs" | tr -d ' ')" != "$chk" || ( "$sid_parent" != "$chk" && "$sid_grand" != "$chk" ) ]]; then
    record "$name" "$expected" "owner" "$(( SECONDS - t0 ))" "ownership check failed; not signalling"
    return
  fi
  local owned_before; owned_before="$(pgrep -s "$sid" | wc -l)"
  case "$sig" in
    INT) kill -INT -- "-$chk" ;;   # Ctrl-C: whole foreground process group (checker + observer)
    TERM) kill -TERM "$chk" ;;     # SIGTERM to the checker pid only
    HANG_OBSERVER) kill -STOP "$obs" ;;  # observer never finishes -> checker timeout
    KILL_BRIDGE) pkill -KILL -s "$sid" -f parameter_bridge ;;  # owned demo breaks mid-run
  esac
  local rc=0; wait_exit "$chk" 45 || rc=$?
  local left_obs=no left_sess=0
  alive "$obs" && left_obs=yes
  left_sess="$(pgrep -s "$sid" | wc -l)"
  note="observer=$obs left=$left_obs; demo session $sid had $owned_before procs, left=$left_sess; $(grep -E '^(FAIL|INTERRUPTED)' "$log" | head -1)"
  [[ "$left_obs" == no && "$left_sess" == 0 ]] || rc="$rc+leak"
  record "$name" "$expected" "$rc" "$(( SECONDS - t0 ))" "$note"
}

bridge_case() {  # name expected domain partition: lone clock bridge + observer
  local name="$1" expected="$2" topic="/reef_regress_bridge_$$_$RANDOM/clock"
  ROS_DOMAIN_ID="$3" GZ_PARTITION="$4" ros2 run ros_gz_bridge parameter_bridge \
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' --ros-args -r "/clock:=$topic" \
    -r __node:=regress_bridge >"$out_dir/bridge_$RANDOM.log" 2>&1 &
  local br=$!
  run_fg "$name" "$expected" ROS_DOMAIN_ID="$3" -- \
    python3 "$REEF_ROOT/scripts/clock_check.py" 2 8 --topic "$topic" --publisher-node regress_bridge
  kill -INT -- "-$br" 2>/dev/null; wait_exit "$br" 10 || { kill -KILL -- "-$br" 2>/dev/null; wait "$br"; }
}

echo "Regression output: $out_dir"
disp_before="$(display_pids | sort | tr '\n' ' ')"
echo "Display service pids before: $disp_before"

# 1. Headless success.
run_fg "1 headless success" 0 REEF_HEADLESS=1 -- "$C"

# 2. GUI success on the existing display.
if "$REEF_ROOT/scripts/check_display.sh" >/dev/null 2>&1; then
  run_fg "2 GUI success on :99" 0 REEF_DISPLAY=:99 -- "$C"
else
  record "2 GUI success on :99" 0 blocked 0 "display :99 not available"
fi

# 3. No clock anywhere on a fresh topic: observer must fail.
run_fg "3 no-clock negative" 1 -- \
  python3 "$REEF_ROOT/scripts/clock_check.py" 1 3 --topic "/reef_regress_$$/clock"

# 4. Unrelated simulation running + checker with an invalid display.
D=$(( RANDOM % 101 + 1 )); P="reef_regress_unrelated_$$"
start_unrelated_sim "$D" "$P"
run_fg "4a unrelated sim visible on /clock (sanity)" 0 ROS_DOMAIN_ID="$D" -- \
  python3 "$REEF_ROOT/scripts/clock_check.py" 2 30 --topic /clock
# Isolation layers on their own (no liveness involved):
run_fg "4e ROS: fresh topic while unrelated /clock" 1 ROS_DOMAIN_ID="$D" -- \
  python3 "$REEF_ROOT/scripts/clock_check.py" 1 5 --topic "/reef_regress_topic_$$/clock"
bridge_case "4f gz: bridge in other partition, no clock" 1 "$D" "reef_regress_other_$$"
bridge_case "4g gz: bridge in same partition (control)" 0 "$D" "$P"
run_fg "4b unrelated sim + invalid display" "2" \
  REEF_DISPLAY=:197 REEF_TEST_ROS_DOMAIN_ID="$D" -- "$C"
# Worst case: force the checker onto the unrelated sim's domain AND partition,
# so only the per-run topic and liveness monitoring stand between it and a pass.
run_fg "4c ...same domain+partition, invalid display" "2" \
  REEF_DISPLAY=:197 REEF_TEST_ROS_DOMAIN_ID="$D" REEF_TEST_GZ_PARTITION="$P" -- "$C"
run_fg "4d unrelated sim + valid headless checker" 0 \
  REEF_HEADLESS=1 REEF_TEST_ROS_DOMAIN_ID="$D" -- "$C"
stop_owned_session "$unrelated_sid" "$unrelated_leader"
unrelated_sid=""

# 5./6. Interruption while the observer is active.
disrupt_case "5 SIGTERM during observation" TERM 143
disrupt_case "6 SIGINT (Ctrl-C to group) during obs." INT 130
disrupt_case "8 timeout: observer hung (SIGSTOP)" HANG_OBSERVER 124
disrupt_case "9 owned bridge killed mid-run" KILL_BRIDGE 2

disp_after="$(display_pids | sort | tr '\n' ' ')"
if [[ "$disp_before" == "$disp_after" ]]; then note="unchanged: $disp_after"; rc=0; else note="before=[$disp_before] after=[$disp_after]"; rc=1; fi
record "7 display services untouched" 0 "$rc" 0 "$note"

echo
echo "===== Summary ($out_dir) ====="
printf '%s\n' "${results[@]}"
exit $(( failures > 0 ))
