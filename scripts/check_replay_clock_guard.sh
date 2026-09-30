#!/usr/bin/env bash
# Negative check for replay_reef_estimator.sh: with a foreign publisher of the
# clock or of a sensor stream already in the replay domain, the replay must
# refuse to start (exit 2).
#   scripts/check_replay_clock_guard.sh RUN_DIR [clock|imu]
# Starts its own publisher (/clock or /x3/imu; a short-lived Python process in
# a new session, in a domain of its own), runs the replay against that domain,
# and stops only the publisher it started. Exit: 0 the replay refused as
# required, 1 it did not, 2 setup failed.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env
run="${1:-}"
kind="${2:-clock}"
[[ -n "$run" && -f "$run/bag/metadata.yaml" ]] || { echo "usage: $0 RUN_DIR [clock|imu] (with a finalized bag)"; exit 2; }
[[ "$kind" == clock || "$kind" == imu ]] || { echo "kind must be clock or imu"; exit 2; }
tree="$(python3 "$REEF_ROOT/scripts/colcon_tree.py")"
set +u
# shellcheck disable=SC1091
source "$tree/install/setup.bash" 2>/dev/null || source /opt/ros/jazzy/setup.bash
set -u
domain="$(( RANDOM % 101 + 1 ))"   # safe DDS range; the replay refuses a busy domain anyway
pub_pid=""
# shellcheck disable=SC2317  # reached only via the EXIT trap
cleanup() {
  local rc=$?
  # Stop only the publisher this script started (its own session leader).
  if [[ -n "$pub_pid" ]] && kill -0 "$pub_pid" 2>/dev/null \
     && [[ "$(ps -o sid= -p "$pub_pid" | tr -d ' ')" == "$pub_pid" ]]; then
    kill -INT "$pub_pid" 2>/dev/null || true
    for _ in $(seq 25); do kill -0 "$pub_pid" 2>/dev/null || break; sleep 0.2; done
    kill -KILL "$pub_pid" 2>/dev/null || true
  fi
  exit "$rc"
}
trap cleanup EXIT
ROS_DOMAIN_ID="$domain" REEF_FOREIGN="$kind" setsid python3 - </dev/null >/dev/null 2>&1 <<'PY' &
import os, rclpy, time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
rclpy.init()
imu = os.environ['REEF_FOREIGN'] == 'imu'
n = rclpy.create_node('foreign_imu' if imu else 'foreign_clock')
p = n.create_publisher(Imu, '/x3/imu', 10) if imu else n.create_publisher(Clock, '/clock', 10)
end = time.monotonic() + 60
try:
    while time.monotonic() < end:
        p.publish(Imu() if imu else Clock())
        rclpy.spin_once(n, timeout_sec=0.05)
except KeyboardInterrupt:
    pass
PY
pub_pid=$!
echo "foreign $kind publisher pid $pub_pid in ROS_DOMAIN_ID=$domain"
sleep 2
rc=0
REEF_REPLAY_DOMAIN="$domain" "$REEF_ROOT/scripts/replay_reef_estimator.sh" "$run" || rc=$?
if (( rc == 2 )); then
  echo "PASS replay refused (exit 2) with a foreign $kind publisher in its domain"
  exit 0
fi
echo "FAIL replay exit $rc with a foreign $kind publisher (expected 2)"
exit 1
