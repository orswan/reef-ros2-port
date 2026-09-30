#!/usr/bin/env bash
# Ensure the pinned upstream repositories used by the reference harness are
# present in reference/ (ignored by Git), cloning them if missing, and that
# the pinned commits exist. Container terminal.
# Exit: 0 ok, 2 missing and could not be fetched (e.g. offline).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
need() {  # dir url commit...
  local dir="$ROOT/reference/$1" url="$2"; shift 2
  if [[ ! -d "$dir/.git" ]]; then
    echo "cloning $url -> reference/$(basename "$dir")"
    git clone -q "$url" "$dir" || { echo "BLOCKED: cannot clone $url"; exit 2; }
  fi
  local c
  for c in "$@"; do
    git -C "$dir" cat-file -e "$c^{commit}" 2>/dev/null \
      || { git -C "$dir" fetch -q origin || true; git -C "$dir" cat-file -e "$c^{commit}" 2>/dev/null; } \
      || { echo "BLOCKED: commit $c not available in reference/$(basename "$dir")"; exit 2; }
  done
  echo "OK   reference/$(basename "$dir"): $*"
}
need reef_estimator https://github.com/uf-reef-avl/reef_estimator.git \
  e4179f48c3f26e22bd1366b71ee1e117ce2f5f7f 95987b5118b624208910d9e51424300022e1f512
need reef_msgs https://github.com/uf-reef-avl/reef_msgs.git 7fb63ff93269040316b71d346dbc32919da1f63d
need reef_control https://github.com/uf-reef-avl/reef_control.git 12237b76b85d755364275ed8c1147f15d8ddf15c
