#!/usr/bin/env bash
# The check functions below are invoked indirectly via check "$@", so the
# linter would report them as unreachable.
# shellcheck disable=SC2317  # check functions are called indirectly via check "$@"
# Validate the REEF dev container from inside it (container terminal):
#   scripts/validate_devcontainer.sh          # environment, desktop, clock checks (~1 min)
#   scripts/validate_devcontainer.sh --full   # also the clock and X3 regression suites (~9 min;
#                                             # X3 needs scripts/setup_assets.py first)
# Writes a VNC snapshot of the Gazebo GUI to log/checks/ so rendering can be
# inspected, not just inferred from running processes.
# Exit status: 0 if every check passed, 1 otherwise.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
reef_reexec_clean "$@"
reef_setup_env

failures=0
ok()   { echo "OK   $*"; }
bad()  { echo "FAIL $*"; failures=$((failures + 1)); }
check() { local name="$1"; shift; if "$@" >/dev/null 2>&1; then ok "$name"; else bad "$name"; fi; }

# Individual checks (each returns 0 on success), called through check "$@".
image_env()        { tr '\0' '\n' </proc/1/environ | grep -qx "$1"; }   # PID 1 carries the image ENV
gz_is_harmonic()   { [[ "$(gz sim --versions)" == 8.* ]]; }
ros_gz_installed() { ros2 pkg prefix ros_gz_bridge && ros2 pkg prefix ros_gz_sim; }
# shellcheck disable=SC2016  # expanded by the interactive shell being tested
shell_ament_path() { env -i HOME=/root TERM=dumb PATH=/usr/bin:/bin bash -ic 'echo "$AMENT_PREFIX_PATH"' 2>/dev/null | tail -1; }
shell_ros_only()   { [[ "$(shell_ament_path)" == /opt/ros/jazzy ]]; }
no_overlay()       { [[ ":$AMENT_PREFIX_PATH:" != *":/root/ros2_ws/install/"* ]]; }
is_mount()         { grep -q " $1 " /proc/self/mountinfo; }
is_project()       { [[ -d "$REEF_ROOT/.git" && -x "$REEF_ROOT/scripts/check_clock_demo.sh" ]]; }
novnc_served()     { curl -fs --max-time 3 http://127.0.0.1:6080/vnc.html | grep -qi novnc; }
logged()           { local log="$1"; shift; "$@" >"$log" 2>&1; }   # run, keeping output

out="$REEF_ROOT/log/checks/devcontainer_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$out"
S="$REEF_ROOT/scripts"
echo "== REEF dev container validation ($out)"

echo "-- image and environment"
check "running in the REEF dev image (/usr/local/bin/reef-desktop)" test -x /usr/local/bin/reef-desktop
check "architecture x86_64 (got $(uname -m))" test "$(uname -m)" = x86_64
check "ROS_DISTRO=jazzy" test "$ROS_DISTRO" = jazzy
check "gz sim is Harmonic 8.x ($(gz sim --versions 2>/dev/null))" gz_is_harmonic
check "ros_gz_bridge and ros_gz_sim installed" ros_gz_installed
for v in DISPLAY=:99 LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3; do
  check "image ENV $v" image_env "$v"
done
check "interactive shell sources only /opt/ros/jazzy (got '$(shell_ament_path)')" shell_ros_only
check "no /root/ros2_ws/install overlay in this environment" no_overlay
check "workspace is a bind mount at /root/ros2_ws/reef_ros2" is_mount /root/ros2_ws/reef_ros2
check "workspace is this project (git repo with scripts/)" is_project
check "Gazebo data volume mounted at /root/.gz" is_mount /root/.gz
check "shellcheck available ($(shellcheck --version 2>/dev/null | awk '/^version/{print $2}'))" command -v shellcheck
check "package manifest recorded (/etc/reef-image-packages.txt)" test -s /etc/reef-image-packages.txt

echo "-- desktop (container-managed)"
check "reef-desktop status: xvfb, fluxbox, x11vnc, websockify" reef-desktop wait 30
wm="$("$S/check_display.sh" | awk -F': ' '/window manager/{print $2}')"
check "window manager is Fluxbox (got '${wm}')" test "$wm" = Fluxbox
check "noVNC page served on :6080 (published as 127.0.0.1:8081 on the Mac)" novnc_served
check "desktop supervisor tests on a spare display (scripts/test_desktop.sh)" \
  logged "$out/test_desktop.log" "$S/test_desktop.sh"

echo "-- Gazebo to ROS clock"
check "headless clock check" logged "$out/headless.log" env REEF_HEADLESS=1 "$S/check_clock_demo.sh"
# GUI check on :99, with a VNC snapshot taken while Gazebo is up.
REEF_CHECK_SECONDS=15 "$S/check_clock_demo.sh" >"$out/gui.log" 2>&1 &
gui=$!
sleep 14
python3 "$S/vnc_snapshot.py" "$out/gazebo_gui.png" --port 5900 >"$out/snapshot.log" 2>&1
gw="$(timeout 3 xwininfo -root -tree 2>/dev/null | grep -c '"Gazebo Sim"')"
if wait "$gui"; then ok "GUI clock check on :99"; else bad "GUI clock check on :99 (see $out/gui.log)"; fi
check "Gazebo GUI window was mapped on :99 (count $gw)" test "$gw" -ge 1
check "VNC snapshot written ($(cat "$out/snapshot.log" 2>/dev/null))" test -s "$out/gazebo_gui.png"
# Negative case: nothing publishes on a fresh topic, so the observer must fail.
python3 "$S/clock_check.py" 1 3 --topic "/reef_validate_$$/clock" >"$out/negative.log" 2>&1
rc=$?
check "no-simulator negative case exits 1 (got $rc)" test "$rc" = 1

if [[ "${1:-}" == --full ]]; then
  echo "-- regression suites"
  check "scripts/regress_clock_check.sh" logged "$out/regress.log" "$S/regress_clock_check.sh"
  check "X3 assets present and verified" logged "$out/assets.log" python3 "$S/setup_assets.py" --verify
  check "scripts/regress_x3_scenario.sh" logged "$out/regress_x3.log" "$S/regress_x3_scenario.sh"
fi

echo "== $failures failure(s). Logs and snapshot: $out"
exit $(( failures > 0 ))
