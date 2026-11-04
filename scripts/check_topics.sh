#!/usr/bin/env bash
# Verify the simulated Mid-360 lidar + IMU topics (run while the simulation is up).
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
exec ros2 run livox_mid360_sim check_livox_cloud.py "$@"
