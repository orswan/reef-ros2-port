#!/usr/bin/env bash
# Exercise docker/reef-desktop on a spare display and spare ports, so it can
# run next to a live desktop (the dev container's :99, or the original
# container's :99/8080) without touching it.
#   scripts/test_desktop.sh      # uses display :150, vnc 5950, web 6150
# Everything this test signals is in a session it created with setsid.
# Exit status: 0 if every case passed, 1 otherwise, 2 if the spare display or
# ports are already in use.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env

D="$REEF_ROOT/docker/reef-desktop"
export REEF_DESKTOP_DISPLAY="${REEF_TEST_DESKTOP_DISPLAY:-150}"
export REEF_DESKTOP_VNC_PORT="${REEF_TEST_DESKTOP_VNC_PORT:-5950}"
export REEF_DESKTOP_WEB_PORT="${REEF_TEST_DESKTOP_WEB_PORT:-6150}"
work="$(mktemp -d)"
export REEF_DESKTOP_STATE="$work/state" REEF_DESKTOP_LOGS="$work/logs"
export HOME="$work/home"   # fluxbox writes ~/.fluxbox; keep the real one untouched
mkdir -p "$HOME"
DISP=":$REEF_DESKTOP_DISPLAY"

busy=""
[[ -e "/tmp/.X11-unix/X$REEF_DESKTOP_DISPLAY" || -e "/tmp/.X$REEF_DESKTOP_DISPLAY-lock" ]] && busy+=" display $DISP"
for p in "$REEF_DESKTOP_VNC_PORT" "$REEF_DESKTOP_WEB_PORT"; do
  timeout 1 bash -c "</dev/tcp/127.0.0.1/$p" 2>/dev/null && busy+=" port $p"
done
if [[ -n "$busy" ]]; then echo "SKIP spare resources in use:$busy"; exit 2; fi

failures=0 sid=""
result() { if [[ "$1" == 0 ]]; then echo "PASS $2"; else echo "FAIL $2"; failures=$((failures + 1)); fi; }

start_sup() {  # new session; the sid is the supervisor's pid
  # shellcheck disable=SC2016  # $$ belongs to the inner bash
  setsid bash -c 'echo $$ >"$1"; exec "$2" run' _ "$work/sid" "$D" >>"$work/supervisor.log" 2>&1 &
  disown
  for _ in $(seq 50); do [[ -s "$work/sid" ]] && break; sleep 0.1; done
  sid="$(cat "$work/sid")"; rm -f "$work/sid"
}
sess_count() { pgrep -s "$sid" | wc -l; }
kill_session() {  # only a session this script created
  [[ "$sid" =~ ^[0-9]+$ ]] && (( sid > 1 )) || return 0
  pkill -KILL -s "$sid" 2>/dev/null
  for _ in $(seq 25); do pgrep -s "$sid" >/dev/null || break; sleep 0.2; done
}
# shellcheck disable=SC2317  # reached only via the EXIT/INT/TERM traps
cleanup() {
  kill_session
  local lock="/tmp/.X$REEF_DESKTOP_DISPLAY-lock" pid
  if [[ -e "$lock" ]]; then
    pid="$(tr -d ' ' <"$lock")"
    kill -0 "$pid" 2>/dev/null || rm -f "$lock" "/tmp/.X11-unix/X$REEF_DESKTOP_DISPLAY"
  fi
  echo "logs: $work"
}
trap cleanup EXIT

echo "reef-desktop test on $DISP (vnc $REEF_DESKTOP_VNC_PORT, web $REEF_DESKTOP_WEB_PORT)"

# 1. Startup: every service ready, and Fluxbox is the running window manager.
start_sup
"$D" wait 30 >/dev/null; result $? "1 startup: all services ready"
wm="$(DISPLAY="$DISP" timeout 2 xprop -root _NET_SUPPORTING_WM_CHECK 2>/dev/null | awk '{print $NF}')"
name="$(DISPLAY="$DISP" timeout 2 xprop -id "$wm" _NET_WM_NAME 2>/dev/null | cut -d'"' -f2)"
[[ "$name" == Fluxbox ]]; result $? "1 window manager is Fluxbox (got '${name}')"
curl -fs --max-time 2 "http://127.0.0.1:$REEF_DESKTOP_WEB_PORT/vnc.html" | grep -qi novnc
result $? "1 noVNC page served on web port"

# 2. A second supervisor must not start duplicate services.
before="$(sess_count)"
"$D" run >"$work/second.log" 2>&1; rc=$?
[[ $rc == 0 && "$(sess_count)" == "$before" ]] && grep -q "already running" "$work/second.log"
result $? "2 second 'run' refused (exit $rc, session procs $before -> $(sess_count))"

# 3. Window manager crash: restarted along with its dependents.
kill -KILL "$(cat "$REEF_DESKTOP_STATE/fluxbox.pid")"; sleep 3
"$D" wait 30 >/dev/null; result $? "3 recovered after fluxbox was killed"

# 4. X server crash: everything restarts; no zombies left behind.
kill -KILL "$(cat "$REEF_DESKTOP_STATE/xvfb.pid")"; sleep 3
"$D" wait 40 >/dev/null; result $? "4 recovered after Xvfb was killed"
z="$(pgrep -s "$sid" -r Z | wc -l)"; [[ "$z" == 0 ]]; result $? "4 no zombie processes ($z)"

# 5. docker stop sends SIGTERM to the main process: all services stop.
kill -TERM "$(cat "$REEF_DESKTOP_STATE/supervisor.pid")"
for _ in $(seq 50); do [[ "$(sess_count)" == 0 ]] && break; sleep 0.2; done
[[ "$(sess_count)" == 0 ]]; result $? "5 SIGTERM stops every service (left: $(sess_count))"

# 6. docker restart after an unclean stop: the stale X lock and socket stay in
#    /tmp. The next start must clear them and come up.
start_sup; "$D" wait 30 >/dev/null
kill_session
[[ -e "/tmp/.X$REEF_DESKTOP_DISPLAY-lock" ]]; result $? "6 setup: stale X lock left by unclean stop"
start_sup
"$D" wait 30 >/dev/null; result $? "6 restart with stale lock: all services ready"
grep -q "removing stale X lock" "$work/supervisor.log"; result $? "6 stale lock was cleared"

echo "== $(( failures == 0 ? 1 : 0 )) (failures: $failures)" | sed 's/== 1/== ALL PASSED/; s/== 0/== FAILED/'
exit $(( failures > 0 ))
