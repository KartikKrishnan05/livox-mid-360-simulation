#!/usr/bin/env bash
# Launch Gazebo + Livox Mid-360 test rig + bridge + RViz. Extra args go to ros2 launch,
# e.g. `pixi run sim gui:=false rviz:=false`.
# macOS SIP strips DYLD_LIBRARY_PATH whenever /bin/bash starts, so re-source the
# workspace here (no-op on Linux).
if [ -f install/setup.bash ]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi
if [ "$(uname -s)" = "Darwin" ] && [ -z "${GZ_IP:-}" ]; then
  export GZ_IP=127.0.0.1
fi

set -euo pipefail
exec ros2 launch livox_mid360_sim sim.launch.py "$@"
