#!/usr/bin/env bash
# Launch rosbridge_websocket so its action clients can load the workspace
# overlay's rosidl typesupport dylibs.
#
# Why this wrapper exists (macOS-specific):
#   * rosbridge_server's `rosbridge_websocket` script starts with
#     `#!/usr/bin/env python3`. The kernel therefore launches it through
#     /usr/bin/env, a SIP-protected binary, which strips DYLD_LIBRARY_PATH
#     before python starts.
#   * Without the overlay's install/<pkg>/lib dirs on DYLD_LIBRARY_PATH,
#     rosbridge cannot dlopen e.g.
#     libnlra_interfaces__rosidl_typesupport_fastrtps_c.dylib when creating
#     an action client, so every action goal sent from the web UI fails with
#     "Type support not from this implementation".
#   * Running the script directly under the pixi python with DYLD_LIBRARY_PATH
#     rebuilt from AMENT_PREFIX_PATH avoids both problems. Other workspace
#     nodes are unaffected because colcon installs them with an absolute
#     python3.12 shebang (no /usr/bin/env hop).
set -euo pipefail

RB_SCRIPT="$(
  python3 -c 'import ament_index_python, os, sys; sys.stdout.write(os.path.join(ament_index_python.packages.get_package_prefix("rosbridge_server"), "lib", "rosbridge_server", "rosbridge_websocket"))'
)"

DYLD_LIBRARY_PATH=""
for prefix in $(printf '%s' "${AMENT_PREFIX_PATH:-}" | tr ':' '\n'); do
  if [ -d "$prefix/lib" ]; then
    DYLD_LIBRARY_PATH="$prefix/lib:${DYLD_LIBRARY_PATH}"
  fi
done
export DYLD_LIBRARY_PATH="${DYLD_LIBRARY_PATH%:}"

exec python3 "$RB_SCRIPT" "$@"