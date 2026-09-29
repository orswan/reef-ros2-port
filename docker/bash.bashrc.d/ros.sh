# shellcheck shell=bash
# Sourced from /etc/bash.bashrc in the REEF dev image, for interactive shells
# only (VS Code terminals, `docker exec -it ... bash`).
#
# Sources ROS 2 Jazzy. It deliberately does NOT source any workspace overlay:
# source the project's own install/setup.bash explicitly when you need it.
# Scripts in /root/ros2_ws/reef_ros2/scripts set up their own environment and
# do not depend on this file.
if [ -r /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  . /opt/ros/jazzy/setup.bash
fi
