#!/usr/bin/env bash
# Read-only diagnostics. Run on the robot; does not command motors.
set +e
section() { printf '\n### %s\n' "$1"; }
section 'System'
uname -a
cat /etc/os-release 2>/dev/null
section 'ROS environment'
printf 'ROS_VERSION=%s\nROS_DISTRO=%s\n' "${ROS_VERSION:-unset}" "${ROS_DISTRO:-unset}"
ls /opt/ros 2>/dev/null || true
section 'Workspaces'
for d in "$HOME/ros2_ws/src" "$HOME/catkin_ws/src"; do
  if [ -d "$d" ]; then printf '%s\n' "$d"; ls -la "$d"; fi
done
section 'Camera and serial devices'
ls -l /dev/video* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
if command -v v4l2-ctl >/dev/null 2>&1; then v4l2-ctl --list-devices; fi
section 'ROS 2 commands and active graph'
if command -v ros2 >/dev/null 2>&1; then
  command -v ros2
  timeout 8 ros2 node list 2>&1 || true
  timeout 8 ros2 topic list -t 2>&1 || true
else
  printf 'ros2 command not found in current shell (may require sourcing).\n'
fi
section 'ROS 1 commands and active graph'
if command -v rostopic >/dev/null 2>&1; then
  command -v rostopic
  timeout 8 rostopic list 2>&1 || true
fi
printf '\nDiagnostics complete. Review output before sharing.\n'
