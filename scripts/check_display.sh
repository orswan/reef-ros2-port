#!/usr/bin/env bash
# Verify the existing Xvfb/noVNC display is up. With --start, run the
# container's own /root/start_vnc.sh if the X server is missing.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env

display_num="${DISPLAY#:}"
display_num="${display_num%%.*}"

x_up() { [[ -S "/tmp/.X11-unix/X${display_num}" ]] && pgrep -f "Xvfb :${display_num}" >/dev/null; }

if ! x_up; then
  if [[ "${1:-}" == "--start" ]]; then
    echo "Xvfb on $DISPLAY not running; starting /root/start_vnc.sh"
    nohup /root/start_vnc.sh >/tmp/reef_start_vnc.log 2>&1 &
    for _ in $(seq 20); do x_up && break; sleep 0.5; done
  fi
fi

status=0
if x_up; then echo "OK   X server on $DISPLAY"; else echo "FAIL X server on $DISPLAY (try: $0 --start)"; status=1; fi
if pgrep -x x11vnc >/dev/null; then echo "OK   x11vnc"; else echo "WARN x11vnc not running (GUI will not be viewable)"; fi
if pgrep -f "websockify.*8080" >/dev/null; then echo "OK   noVNC websockify on :8080 -> http://localhost:8080/vnc.html"; else echo "WARN websockify not running"; fi
if pgrep -x fluxbox >/dev/null; then echo "OK   fluxbox"; else echo "INFO fluxbox not running (windows lack decorations; not required)"; fi
exit $status
