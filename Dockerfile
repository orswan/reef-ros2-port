# syntax=docker/dockerfile:1
#
# REEF ROS 2 development image: ROS 2 Jazzy + Gazebo Harmonic, with a browser
# desktop (Xvfb + Fluxbox + x11vnc + noVNC) and software OpenGL.
#
# Reproduces the original ros2_novnc_container recipe
# (docs/setup/GazeboMacDockerSetup.md). The intentional differences are listed
# in README.md, "Differences from the original container".
#
# Build and run through compose.yaml (terminal) or .devcontainer/ (VS Code),
# which both use this file.

# ros:jazzy (= jazzy-ros-base). This is the multi-arch index digest, verified
# against registry-1.docker.io on 2026-09-29. Its linux/amd64 manifest is
# sha256:efbc8cb259b6346f1c8944ffd49d3fbffd647116ec05006df8b36444a13b7bdf
# (image created 2026-09-16).
ARG BASE_IMAGE=ros:jazzy@sha256:c3706ef0a0aa45413c07803cf433602f543b22e45b4855f6fca955c2d8ecc4e8
# DL3006: the tag and digest are in BASE_IMAGE. DL3029: linux/amd64 is a
# requirement (Intel Mac host; matches the original container).
# hadolint ignore=DL3006,DL3029
FROM --platform=linux/amd64 ${BASE_IMAGE}

# Fail a RUN if any command in a pipeline fails.
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Same package set as the original `apt install -y ros-jazzy-ros-gz xvfb x11vnc
# novnc fluxbox`, with apt's default of installing recommends (as the original
# did). Also listed explicitly, because the scripts call them:
#   x11-utils   xdpyinfo/xprop/xwininfo (readiness and window checks)
#   procps      pgrep/pkill/ps (process ownership and cleanup)
#   util-linux  setsid/flock (session-based teardown, single supervisor)
#   curl        noVNC readiness probe
#   shellcheck, python3-pyflakes
#               lint for the project's shell and Python code (P09
#               scripts/check_code_quality.sh, scripts/ci.sh)
#   ros-jazzy-cv-bridge, libopencv-dev
#               P08 reef_rgbd_odometry (present via ros-gz dependencies;
#               listed because package.xml declares them)
#   g++, libeigen3-dev, libboost-dev
#               P02 reference harness: builds the pinned original estimator
#               sources (present in the base image or as dependencies; listed
#               because baseline/ depends on them)
#   python3-matplotlib, python3-numpy
#               reef_sim bag analysis and plots (otherwise only present via
#               recommends)
#   ros-jazzy-rosbag2, ros-jazzy-rosbag2-py, ros-jazzy-rosbag2-storage-mcap
#               recording and reading the X3 scenario bags (already in the
#               ros-base image; listed because reef_sim depends on them)
#   ros-jazzy-rosidl-default-generators, -rosidl-default-runtime,
#   -eigen3-cmake-module, -rclcpp, -rclpy, -ament-cmake-gtest,
#   -ament-cmake-pytest, -rosidl-runtime-py, python3-pytest, python3-yaml
#               P03 packages (reef_msgs, reef_estimator, vendored
#               rosflight_msgs) and their tests; expected in the ros-base
#               image already, listed because package.xml declares them
# The ROS apt repository keeps only current versions, so versions are recorded
# at build time (/etc/reef-image-packages.txt), not pinned.
# DL3008: see above. DL3015: recommends are installed on purpose (as in the original).
# hadolint ignore=DL3008,DL3015
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ros-jazzy-ros-gz \
        xvfb \
        x11vnc \
        novnc \
        fluxbox \
        x11-utils \
        procps \
        util-linux \
        curl \
        shellcheck \
        python3-pyflakes \
        ros-jazzy-cv-bridge \
        libopencv-dev \
        g++ \
        libeigen3-dev \
        libboost-dev \
        python3-matplotlib \
        python3-numpy \
        ros-jazzy-rosbag2 \
        ros-jazzy-rosbag2-py \
        ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-rosidl-default-generators \
        ros-jazzy-rosidl-default-runtime \
        ros-jazzy-eigen3-cmake-module \
        ros-jazzy-rclcpp \
        ros-jazzy-rclpy \
        ros-jazzy-ament-cmake-gtest \
        ros-jazzy-ament-cmake-pytest \
        ros-jazzy-rosidl-runtime-py \
        python3-pytest \
        python3-yaml \
    && rm -rf /var/lib/apt/lists/* \
    && dpkg-query -W -f='${Package}\t${Version}\n' | sort > /etc/reef-image-packages.txt

# Rendering and display settings from the original container (set there in
# ~/.bashrc; here image-wide so every process, including `docker exec`, gets them).
ENV DISPLAY=:99 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    MESA_GL_VERSION_OVERRIDE=3.3

# Interactive shells source ROS 2 Jazzy only, never a workspace overlay. The
# project scripts source their own environment and do not rely on this.
COPY docker/bash.bashrc.d/ros.sh /etc/reef/bashrc-ros.sh
RUN printf '\n# REEF dev image: see /etc/reef/bashrc-ros.sh\n[ -r /etc/reef/bashrc-ros.sh ] && . /etc/reef/bashrc-ros.sh\n' >> /etc/bash.bashrc

# Desktop supervisor. It is the container's main process; the base image's
# /ros_entrypoint.sh runs it after sourcing ROS.
COPY docker/reef-desktop /usr/local/bin/reef-desktop
RUN chmod 0755 /usr/local/bin/reef-desktop

# noVNC inside the container. compose.yaml publishes it on 127.0.0.1:8081 of
# the Mac. x11vnc's 5900 listens on localhost only and is not published.
EXPOSE 6080

HEALTHCHECK --interval=10s --timeout=8s --start-period=20s --retries=3 \
    CMD ["reef-desktop", "status", "--quiet"]

WORKDIR /root/ros2_ws/reef_ros2
CMD ["reef-desktop", "run"]
