# shellcheck shell=bash
# Process ownership and cleanup helpers for scripts that launch a simulation
# in a session they own (same approach as the reviewed check_clock_demo.sh).
#
# sim_start_session sets sid and launch_pid. The caller exports ROS_DOMAIN_ID
# and GZ_PARTITION, defines on_int/on_term, calls defer_traps before starting
# children, then normal_traps and exit_for_pending once their pids are known.

sim_alive() {  # pid: running and not a zombie
  local st
  st="$(ps -o stat= -p "$1" 2>/dev/null)" && [[ "$st" != Z* ]]
}

sim_env_matches() {  # pid: runs with this run's ROS domain and Gazebo partition
  local e
  e="$(tr '\0' '\n' 2>/dev/null <"/proc/$1/environ")" || return 1
  grep -qx "ROS_DOMAIN_ID=$ROS_DOMAIN_ID" <<<"$e" && grep -qx "GZ_PARTITION=$GZ_PARTITION" <<<"$e"
}

sim_start_session() {  # log cmd...: start cmd in a new session; sets launch_pid and sid
  local log="$1"; shift
  local sid_file
  sid_file="$(mktemp)"
  # setsid -w keeps $! alive exactly as long as the session leader. env
  # --default-signal undoes bash's SIGINT ignore for background jobs.
  # shellcheck disable=SC2016  # $$ and $@ belong to the inner bash
  # stdin from /dev/null: nothing in the session may read the caller's terminal.
  setsid -w env --default-signal=INT bash -c 'echo $$ >"$1"; shift; exec "$@"' _ "$sid_file" "$@" \
    </dev/null >"$log" 2>&1 &
  launch_pid=$!
  for _ in $(seq 50); do [[ -s "$sid_file" ]] && break; sleep 0.1; done
  sid="$(cat "$sid_file")"
  rm -f "$sid_file"
}

# shellcheck disable=SC2317  # reached from cleanup traps
sim_recover_sid() {  # if registration never finished, find the session from launch_pid
  [[ -z "${sid:-}" && -n "${launch_pid:-}" ]] || return 0
  local c
  for c in "$launch_pid" $(pgrep -P "$launch_pid"); do
    if [[ "$(ps -o sid= -p "$c" 2>/dev/null | tr -d ' ')" == "$c" ]]; then sid="$c"; return 0; fi
  done
}

# shellcheck disable=SC2317  # reached from cleanup traps
sim_stop_session() {  # INT, TERM, KILL the owned session; bounded waits
  sim_recover_sid
  local own
  own="$(ps -o sid= -p $$ | tr -d ' ')"   # never signal the caller's own session
  if ! [[ "${sid:-}" =~ ^[0-9]+$ ]] || (( sid <= 1 )) || [[ "$sid" == "$own" ]]; then
    return 0
  fi
  local sig
  for sig in INT TERM KILL; do
    pgrep -s "$sid" >/dev/null || break
    pkill "-$sig" -s "$sid" 2>/dev/null || true
    for _ in $(seq 25); do pgrep -s "$sid" >/dev/null || break; sleep 0.2; done
  done
  if [[ -n "${launch_pid:-}" ]]; then wait "$launch_pid" 2>/dev/null || true; fi
}

sim_wait_launch_exit() {  # seconds: wait for the session leader (the launch) to exit by itself
  local limit="$1"
  for _ in $(seq $(( limit * 5 ))); do
    sim_alive "$launch_pid" || { wait "$launch_pid" 2>/dev/null; return 0; }
    sleep 0.2
  done
  return 1
}

normal_traps() { trap on_int INT; trap on_term TERM; }
defer_traps() { pending=""; trap 'pending=${pending:-130}' INT; trap 'pending=${pending:-143}' TERM; }
exit_for_pending() {
  [[ -n "${pending:-}" ]] || return 0
  normal_traps
  if (( pending == 130 )); then on_int; else on_term; fi
}
