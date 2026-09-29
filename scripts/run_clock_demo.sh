#!/usr/bin/env bash
# Launch the simple Gazebo world with the /clock bridge (interactive demo).
#   scripts/run_clock_demo.sh              # GUI on the noVNC display
#   REEF_HEADLESS=1 scripts/run_clock_demo.sh
# Extra arguments are forwarded to `ros2 launch`, e.g. world:=/path/to.sdf
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env

headless=false
if [[ "${REEF_HEADLESS:-0}" == "1" ]]; then
  headless=true
else
  "$REEF_ROOT/scripts/check_display.sh"
fi

exec ros2 launch "$REEF_ROOT/sim/launch/clock_demo.launch.py" headless:=$headless "$@"
