#!/usr/bin/env bash
# Verify the browser desktop that Gazebo's GUI will use.
#
# Dev container (docker/reef-desktop installed): the desktop is started by the
# container itself. A window manager is required, and --start only waits (up
# to 60 s) for it to become ready; it never launches a second desktop.
#
# Original container (ros2_novnc_container, /root/start_vnc.sh): --start runs
# /root/start_vnc.sh if the X server is missing. A missing Fluxbox is reported
# as WARN: start_vnc.sh starts it without waiting for Xvfb, so it can exit
# with "Couldn't connect to XServer" (reproduced 3/5 times).
#
# REEF_REQUIRE_WM=1|0 overrides whether a missing window manager fails.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env

managed=0
command -v reef-desktop >/dev/null && managed=1
require_wm="${REEF_REQUIRE_WM:-$managed}"

x_up() { timeout 2 xdpyinfo >/dev/null 2>&1; }
wm_name() {
  local id
  id="$(timeout 2 xprop -root _NET_SUPPORTING_WM_CHECK 2>/dev/null | awk '/window id/{print $NF}')"
  [[ -n "$id" ]] && timeout 2 xprop -id "$id" _NET_WM_NAME 2>/dev/null | cut -d'"' -f2
}

if [[ "${1:-}" == "--start" ]]; then
  if (( managed )); then
    for _ in $(seq 120); do x_up && [[ -n "$(wm_name)" ]] && break; sleep 0.5; done
  elif ! x_up; then
    echo "X server on $DISPLAY not running; starting /root/start_vnc.sh"
    nohup /root/start_vnc.sh >/tmp/reef_start_vnc.log 2>&1 &
    for _ in $(seq 20); do x_up && break; sleep 0.5; done
  fi
fi

status=0
echo "INFO environment: $( (( managed )) && echo "dev container (reef-desktop)" || echo "original container (start_vnc.sh)")"
if x_up; then
  echo "OK   X server on $DISPLAY"
else
  echo "FAIL X server on $DISPLAY (try: $0 --start)"; status=1
fi

wm="$(wm_name || true)"
if [[ -n "$wm" ]]; then
  echo "OK   window manager: $wm"
elif (( require_wm )); then
  echo "FAIL no window manager on $DISPLAY"; status=1
else
  echo "WARN no window manager on $DISPLAY. start_vnc.sh starts fluxbox before Xvfb is ready, so it may have exited. GUI windows still render, but without decorations."
fi

if pgrep -x x11vnc >/dev/null; then echo "OK   x11vnc"; else echo "WARN x11vnc not running (GUI will not be viewable)"; fi
if pgrep -f "websockify" >/dev/null; then echo "OK   noVNC websockify"; else echo "WARN websockify not running"; fi
if (( managed )); then reef-desktop status | sed 's/^/     /'; fi
exit $status
