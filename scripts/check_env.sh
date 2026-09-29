#!/usr/bin/env bash
# Checks below are strings evaluated by check(), single-quoted on purpose.
# shellcheck disable=SC2016
# Print and sanity-check the explicit environment used by project scripts.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env
reef_print_env

status=0
check() { if eval "$2" >/dev/null 2>&1; then echo "OK   $1"; else echo "FAIL $1"; status=1; fi; }
check "ROS_DISTRO is jazzy"            '[[ "$ROS_DISTRO" == jazzy ]]'
check "ros2 CLI available"             'command -v ros2'
check "gz CLI available"               'command -v gz'
check "gz sim is Harmonic (8.x)"       '[[ "$(gz sim --versions)" == 8.* ]]'
check "ros_gz_bridge installed"        'ros2 pkg prefix ros_gz_bridge'
check "ros_gz_sim installed"           'ros2 pkg prefix ros_gz_sim'
check "no /root/ros2_ws/install overlay" '[[ ":$AMENT_PREFIX_PATH:" != *":/root/ros2_ws/install/"* ]]'
check "reference/ is COLCON_IGNOREd"   '[[ -f "$REEF_ROOT/reference/COLCON_IGNORE" ]]'
exit $status
