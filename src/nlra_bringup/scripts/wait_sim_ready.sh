#!/usr/bin/env bash
# Wait until the ros2_control arm_controller is active, i.e. the Gazebo sim,
# robot spawn and controllers are fully up. Used by nlra.launch.py to start
# move_group and the skill servers as soon as the sim is ready instead of a
# fixed delay.
#
# Why this exists: on macOS the `ros2` CLI must run with the colcon overlay's
# lib dirs on DYLD_LIBRARY_PATH to dlopen the controller_manager_msgs
# typesupport. AMENT_PREFIX_PATH survives the SIP purge (unlike
# DYLD_LIBRARY_PATH), so rebuild the latter from the former — same trick as
# scripts/ros2_exec.sh and run_rosbridge.sh.
set -euo pipefail

TIMEOUT="${1:-90}"

DYLD_LIBRARY_PATH=""
for prefix in $(printf '%s' "${AMENT_PREFIX_PATH:-}" | tr ':' '\n'); do
  if [ -d "$prefix/lib" ]; then
    DYLD_LIBRARY_PATH="$prefix/lib:${DYLD_LIBRARY_PATH}"
  fi
done
export DYLD_LIBRARY_PATH="${DYLD_LIBRARY_PATH%:}"

deadline=$(( $(date +%s) + TIMEOUT ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  if ros2 control list_controllers 2>/dev/null | grep -q 'arm_controller.*active'; then
    exit 0
  fi
  sleep 0.5
done

echo "arm_controller did not become active within ${TIMEOUT}s" >&2
exit 1
