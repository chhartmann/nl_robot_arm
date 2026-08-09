#!/usr/bin/env bash
# Execute a ROS 2 command with the colcon overlay's lib dirs on the dynamic
# linker search path.
#
# Why this exists: macOS purges DYLD_LIBRARY_PATH when a child is spawned
# through a protected system shell (/bin/sh, /bin/zsh, launched via SIP /
# hardened runtime). Pixi runs tasks through such a shell, so any
# DYLD_LIBRARY_PATH set by the pixi activation (e.g. the colcon overlay's
# install/<pkg>/lib dirs, where the nlra_interfaces rosidl typesupport dylibs
# live) is stripped before the task process starts. That makes rclpy fail with
# "Could not load library libnlra_interfaces__rosidl_typesupport_fastrtps_c.dylib".
#
# AMENT_PREFIX_PATH is NOT a DYLD_* variable, so it survives the purge. We
# rebuild DYLD_LIBRARY_PATH from it here, after pixi's shell has already been
# spawned, then exec the requested command.
set -euo pipefail

DYLD_LIBRARY_PATH=""
for prefix in $(printf '%s' "${AMENT_PREFIX_PATH:-}" | tr ':' '\n'); do
  if [ -d "$prefix/lib" ]; then
    DYLD_LIBRARY_PATH="$prefix/lib:${DYLD_LIBRARY_PATH}"
  fi
done
export DYLD_LIBRARY_PATH="${DYLD_LIBRARY_PATH%:}"

exec "$@"
