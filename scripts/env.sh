# shellcheck shell=bash
# Explicit ROS 2 + display environment for REEF ROS 2 scripts.
#
# Usage (first lines of every script in scripts/):
#   source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
#   reef_reexec_clean "$@"   # re-runs the calling script under `env -i`
#   reef_setup_env
#
# Why: agent / CI shells do not reliably read ~/.bashrc, and in this container
# ~/.bashrc sources an unrelated overlay (/root/ros2_ws/install). Re-executing
# under `env -i` guarantees that neither missing nor leaked state affects us.

REEF_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export REEF_ROOT

# Variables allowed through the clean re-exec. Everything else is dropped.
REEF_PASSTHROUGH_VARS=(HOME USER LOGNAME TERM LANG LC_ALL TZ
  ROS_DOMAIN_ID RMW_IMPLEMENTATION
  REEF_DISCOVERY_RANGE REEF_DISPLAY REEF_HEADLESS REEF_CHECK_SECONDS)

reef_reexec_clean() {
  if [[ "${REEF_CLEAN_ENV:-}" == "1" ]]; then
    return 0
  fi
  local args=() v
  for v in "${REEF_PASSTHROUGH_VARS[@]}"; do
    [[ -n "${!v:-}" ]] && args+=("$v=${!v}")
  done
  exec env -i "${args[@]}" \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    REEF_CLEAN_ENV=1 \
    bash --noprofile --norc "$0" "$@"
}

reef_setup_env() {
  # --- Display (reuses the existing Xvfb/noVNC setup from /root/start_vnc.sh)
  export DISPLAY="${REEF_DISPLAY:-:99}"
  export LIBGL_ALWAYS_SOFTWARE=1
  export MESA_GL_VERSION_OVERRIDE=3.3
  # Xvfb has no GPU; force Qt onto X11 so the Gazebo GUI does not probe Wayland.
  export QT_QPA_PLATFORM=xcb

  # --- ROS 2 Jazzy underlay only (setup scripts reference unset vars)
  local had_u=0
  [[ $- == *u* ]] && had_u=1 && set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  # Project overlay, if one has been built in this repository.
  if [[ -f "$REEF_ROOT/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$REEF_ROOT/install/setup.bash"
  fi
  (( had_u )) && set -u

  # Keep DDS discovery on this host. Jazzy's ros_environment hook always sets
  # SUBNET, so the override is REEF_DISCOVERY_RANGE (SUBNET|LOCALHOST|OFF|...).
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${REEF_DISCOVERY_RANGE:-LOCALHOST}"

  # Guard: the unrelated /root/ros2_ws/install overlay must not be active.
  if [[ ":${AMENT_PREFIX_PATH:-}:" == *":/root/ros2_ws/install/"* ]]; then
    echo "ERROR: unrelated overlay /root/ros2_ws/install is on AMENT_PREFIX_PATH" >&2
    return 1
  fi
}

reef_print_env() {
  echo "REEF_ROOT=$REEF_ROOT"
  echo "ROS_DISTRO=${ROS_DISTRO:-<unset>}"
  echo "DISPLAY=$DISPLAY LIBGL_ALWAYS_SOFTWARE=$LIBGL_ALWAYS_SOFTWARE MESA_GL_VERSION_OVERRIDE=$MESA_GL_VERSION_OVERRIDE"
  echo "ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<default 0>}"
  echo "AMENT_PREFIX_PATH=${AMENT_PREFIX_PATH:-<unset>}"
}
