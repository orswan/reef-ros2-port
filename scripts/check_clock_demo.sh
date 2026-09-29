#!/usr/bin/env bash
# Automated check: start the clock demo, verify ROS 2 sees advancing sim time,
# then shut everything down.
#   scripts/check_clock_demo.sh            # with GUI on the noVNC display
#   REEF_HEADLESS=1 scripts/check_clock_demo.sh
# REEF_CHECK_SECONDS sets the measurement window (default 5).
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env

log_dir="$REEF_ROOT/log/checks"
mkdir -p "$log_dir"
log="$log_dir/clock_demo_$(date +%Y%m%d_%H%M%S).log"

# Run in a new session. gz sim puts its server and GUI in their own process
# groups, so teardown targets the session id rather than a process group.
sid_file="$(mktemp)"
setsid bash -c 'echo $$ >"$1"; exec "$2"' _ "$sid_file" \
  "$REEF_ROOT/scripts/run_clock_demo.sh" >"$log" 2>&1 &
for _ in $(seq 50); do [[ -s "$sid_file" ]] && break; sleep 0.1; done
sid="$(cat "$sid_file")"
rm -f "$sid_file"

cleanup() {
  if pgrep -s "$sid" >/dev/null; then
    pkill -INT -s "$sid" || true
    for _ in $(seq 20); do pgrep -s "$sid" >/dev/null || break; sleep 0.5; done
    pkill -KILL -s "$sid" || true
  fi
}
trap cleanup EXIT

echo "Launched clock demo (session $sid), log: $log"
status=0
python3 "$REEF_ROOT/scripts/clock_check.py" "${REEF_CHECK_SECONDS:-5}" 60 || status=$?

if (( status != 0 )); then
  echo "--- last 30 lines of launch log ---"
  tail -n 30 "$log"
fi
exit $status
