# shellcheck shell=bash
# Helpers for tests that tag the processes they start with a unique
# GZ_PARTITION. A tag proves ownership of that one process only, never of its
# session or process group, so these helpers signal tagged pids individually.

tagged_pids() {  # tag -> pids whose environment carries GZ_PARTITION=<tag>
  local p
  for p in /proc/[0-9]*; do
    tr '\0' '\n' 2>/dev/null <"$p/environ" | grep -qx "GZ_PARTITION=$1" && echo "${p#/proc/}"
  done
  return 0
}

has_tag() {  # pid tag
  tr '\0' '\n' 2>/dev/null <"/proc/$1/environ" | grep -qx "GZ_PARTITION=$2"
}

reap_tagged() {  # tag: stop every process carrying the tag, and nothing else
  local tag="$1" sig pid pids
  for sig in INT TERM KILL; do
    pids="$(tagged_pids "$tag")"
    [[ -n "$pids" ]] || return 0
    for pid in $pids; do
      # Re-check right before signalling so an exited pid that the kernel
      # has reused is not hit.
      has_tag "$pid" "$tag" && kill "-$sig" "$pid" 2>/dev/null
    done
    for _ in $(seq 25); do [[ -z "$(tagged_pids "$tag")" ]] && return 0; sleep 0.2; done
  done
  [[ -z "$(tagged_pids "$tag")" ]]
}

display_service_pids() {  # long-lived display services, sorted: Xvfb, x11vnc, websockify listener(s)
  # websockify forks a child per browser connection; those come and go with
  # the viewer, so only listeners (a parent that is not websockify) count.
  local p pp
  {
    pgrep -x Xvfb
    pgrep -x x11vnc
    for p in $(pgrep -f "^/usr/bin/python3 /usr/bin/websockify"); do
      pp="$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')"
      tr '\0' ' ' 2>/dev/null <"/proc/$pp/cmdline" | grep -q websockify || echo "$p"
    done
  } | sort -n | tr '\n' ' '
}
