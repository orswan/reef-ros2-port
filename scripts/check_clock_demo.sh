#!/usr/bin/env bash
# Automated check: start the clock demo, verify ROS 2 sees advancing sim time
# from *this* demo, then shut down everything the check started.
#   scripts/check_clock_demo.sh            # with GUI on the noVNC display
#   REEF_HEADLESS=1 scripts/check_clock_demo.sh
#
# Knobs: REEF_CHECK_SECONDS (window, default 5), REEF_STARTUP_TIMEOUT (wait for
# first clock message, default 60), REEF_TEST_ROS_DOMAIN_ID (default: random
# 1-101), REEF_TEST_GZ_PARTITION (default: unique per run).
# Test-only: REEF_TEST_REGISTER_DELAY=<s> pauses inside the startup window
# (demo launched, session not yet registered) so regressions can signal there.
#
# Why another simulation cannot make this pass:
#   * Gazebo: a per-run GZ_PARTITION, so the owned bridge only hears the owned
#     gz server.
#   * ROS: the bridge publishes on a per-run topic, and the observer requires
#     exactly one publisher on it, named clock_bridge. The ROS domain only
#     reduces cross-talk; it is not relied on for uniqueness.
#   * Liveness: the check fails as soon as the owned launch or a required
#     process (gz server, bridge, and GUI unless headless) exits.
#
# Exit status: 0 pass, 1 clock check failed, 2 owned demo failed or bad input,
#              124 timeout, 130 SIGINT, 143 SIGTERM.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env

window="${REEF_CHECK_SECONDS:-5}"
startup="${REEF_STARTUP_TIMEOUT:-60}"
num_re='^[0-9]+([.][0-9]+)?$'
if ! [[ "$window" =~ $num_re && "$startup" =~ $num_re ]]; then
  echo "FAIL REEF_CHECK_SECONDS/REEF_STARTUP_TIMEOUT must be positive numbers" >&2
  exit 2
fi

# --- Test-owned communication settings, inherited by the launch and observer.
token="$(printf '%04x%04x%04x' $$ $RANDOM $RANDOM)"
export ROS_DOMAIN_ID="${REEF_TEST_ROS_DOMAIN_ID:-$(( RANDOM % 101 + 1 ))}"
export GZ_PARTITION="${REEF_TEST_GZ_PARTITION:-reef_clock_check_$token}"
clock_topic="/reef_clock_check_${token}/clock"

headless="${REEF_HEADLESS:-0}"
# pgrep -f patterns. Headless `gz sim -s` runs the server in-process; the GUI
# mode forks separate "gz sim server" and "gz sim gui" processes.
if [[ "$headless" == "1" ]]; then
  required=("^gz sim -r -s " "parameter_bridge")
else
  required=("^gz sim server" "^gz sim gui" "parameter_bridge")
fi

log_dir="$REEF_ROOT/log/checks"
mkdir -p "$log_dir"
log="$log_dir/clock_demo_$(date +%Y%m%d_%H%M%S)_$token.log"

launch_pid="" sid="" obs_pid=""
own_sid="$(ps -o sid= -p $$ | tr -d ' ')"

alive() {  # running and not a zombie
  local st
  st="$(ps -o stat= -p "$1" 2>/dev/null)" && [[ "$st" != Z* ]]
}

# shellcheck disable=SC2317  # reached only via the EXIT/INT/TERM traps
stop_pid() {  # stop one owned child: TERM, bounded wait, KILL, reap
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  if alive "$pid"; then
    kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 25); do alive "$pid" || break; sleep 0.2; done
    kill -KILL "$pid" 2>/dev/null || true
  fi
  wait "$pid" 2>/dev/null || true
}

# shellcheck disable=SC2317  # reached only via the EXIT/INT/TERM traps
recover_sid() {  # find the demo session from launch_pid if registration never finished
  [[ -z "$sid" && -n "$launch_pid" ]] || return 0
  local c
  # setsid -w either becomes the session leader itself or forks one child that does.
  for c in "$launch_pid" $(pgrep -P "$launch_pid"); do
    if [[ "$(ps -o sid= -p "$c" 2>/dev/null | tr -d ' ')" == "$c" ]]; then
      sid="$c"; return 0
    fi
  done
}

# shellcheck disable=SC2317  # reached only via the EXIT/INT/TERM traps
stop_session() {  # stop the demo session this script created, and nothing else
  recover_sid
  if ! [[ "$sid" =~ ^[0-9]+$ ]] || (( sid <= 1 )) || [[ "$sid" == "$own_sid" ]]; then
    return 0
  fi
  local sig
  for sig in INT TERM KILL; do
    pgrep -s "$sid" >/dev/null || break
    pkill "-$sig" -s "$sid" 2>/dev/null || true
    for _ in $(seq 25); do pgrep -s "$sid" >/dev/null || break; sleep 0.2; done
  done
  if [[ -n "$launch_pid" ]]; then wait "$launch_pid" 2>/dev/null || true; fi
}

# shellcheck disable=SC2317  # reached only via the EXIT/INT/TERM traps
cleanup() {
  local rc=$?
  trap '' INT TERM
  stop_pid "$obs_pid"
  stop_session
  [[ -n "$sid_file" ]] && rm -f "$sid_file"
  if [[ -n "$sid" ]] && pgrep -s "$sid" >/dev/null; then
    echo "WARN processes remain in demo session $sid" >&2
  fi
  if (( rc != 0 )) && [[ -f "$log" ]]; then
    echo "--- last 30 lines of launch log ($log) ---"
    tail -n 30 "$log"
  fi
  exit "$rc"
}
# shellcheck disable=SC2317  # reached only via the EXIT/INT/TERM traps
on_int() { echo "INTERRUPTED (SIGINT)"; exit 130; }
# shellcheck disable=SC2317  # reached only via the EXIT/INT/TERM traps
on_term() { echo "INTERRUPTED (SIGTERM)"; exit 143; }
normal_traps() { trap on_int INT; trap on_term TERM; }
# During startup a signal is only recorded. Children are started and their pids
# registered first, so cleanup always knows what to stop.
pending=""
exit_for_pending() { if (( pending == 130 )); then on_int; else on_term; fi; }
defer_traps() { trap 'pending=${pending:-130}' INT; trap 'pending=${pending:-143}' TERM; }
sid_file=""
trap cleanup EXIT
normal_traps

fail() { echo "FAIL $2"; exit "$1"; }

env_matches() {  # pid: process runs with this test's ROS domain and Gazebo partition
  local e; e="$(tr '\0' '\n' 2>/dev/null <"/proc/$1/environ")" || return 1
  grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" <<<"$e" && grep -qx "GZ_PARTITION=$GZ_PARTITION" <<<"$e"
}

# --- Owned demo in a new session. gz sim puts its server and GUI in their own
# process groups, so teardown targets the session id. setsid -w keeps $! alive
# for exactly as long as the demo, whether or not setsid needs to fork.
# env --default-signal undoes the SIGINT ignore that bash applies to background
# jobs, so the launch and gz processes respond to INT normally.
defer_traps
sid_file="$(mktemp)"
# shellcheck disable=SC2016  # $$ and $@ belong to the inner bash
setsid -w env --default-signal=INT bash -c 'echo $$ >"$1"; shift; exec "$@"' _ "$sid_file" \
  "$REEF_ROOT/scripts/run_clock_demo.sh" "clock_topic:=$clock_topic" >"$log" 2>&1 &
launch_pid=$!
[[ -n "${REEF_TEST_REGISTER_DELAY:-}" ]] && sleep "$REEF_TEST_REGISTER_DELAY"
for _ in $(seq 50); do [[ -s "$sid_file" ]] && break; sleep 0.1; done
sid="$(cat "$sid_file")"
rm -f "$sid_file"; sid_file=""
if [[ -n "$pending" ]]; then normal_traps; exit_for_pending; fi
if [[ -z "$sid" ]]; then normal_traps; fail 2 "demo session did not start"; fi

echo "Launched clock demo: session=$sid ROS_DOMAIN_ID=$ROS_DOMAIN_ID GZ_PARTITION=$GZ_PARTITION"
echo "  topic=$clock_topic headless=$headless log=$log"

env --default-signal=INT python3 "$REEF_ROOT/scripts/clock_check.py" "$window" "$startup" \
  --topic "$clock_topic" --publisher-node clock_bridge &
obs_pid=$!
normal_traps
if [[ -n "$pending" ]]; then exit_for_pending; fi

# --- Monitor until the observer finishes; fail fast if the owned demo dies.
limit=$(( ${startup%.*} + ${window%.*} + 15 ))
deadline=$(( SECONDS + limit ))
declare -A seen=()
obs_rc=""
while :; do
  if ! alive "$obs_pid"; then
    obs_rc=0; wait "$obs_pid" || obs_rc=$?
    obs_pid=""
  fi
  alive "$launch_pid" || fail 2 "owned demo launch exited early"
  for p in "${required[@]}"; do
    if pid="$(pgrep -s "$sid" -f "$p" | head -1)" && [[ -n "$pid" ]]; then
      if [[ -z "${seen[$p]:-}" ]]; then
        env_matches "$pid" || fail 2 "'$p' (pid $pid) lacks this test's ROS_DOMAIN_ID/GZ_PARTITION"
        echo "  up: '$p' pid $pid (ROS_DOMAIN_ID and GZ_PARTITION verified)"
      fi
      seen[$p]=1
    elif [[ -n "${seen[$p]:-}" ]]; then
      fail 2 "required process '$p' exited"
    fi
  done
  [[ -z "$obs_rc" ]] || break
  (( SECONDS <= deadline )) || fail 124 "timed out after ${limit}s"
  sleep 0.2
done

(( obs_rc == 0 )) || fail "$(( obs_rc >= 128 ? obs_rc : 1 ))" "clock observer exited with status $obs_rc"
for p in "${required[@]}"; do
  [[ -n "${seen[$p]:-}" ]] || fail 2 "required process '$p' never started"
done
echo "PASS owned demo stayed up and its clock advanced in ROS 2"
exit 0
