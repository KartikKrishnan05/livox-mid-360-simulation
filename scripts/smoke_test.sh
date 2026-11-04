#!/usr/bin/env bash
# Headless end-to-end test: start the simulation without GUI/RViz, run the Livox
# checks, report CPU usage of the Gazebo server, then shut everything down.
# Usage: scripts/smoke_test.sh [extra ros2 launch args...]
# macOS SIP strips DYLD_LIBRARY_PATH whenever /bin/bash starts, so re-source the
# workspace here (no-op on Linux).
if [ -f install/setup.bash ]; then
  # shellcheck disable=SC1091
  source install/setup.bash
fi
if [ "$(uname -s)" = "Darwin" ] && [ -z "${GZ_IP:-}" ]; then
  export GZ_IP=127.0.0.1
fi

set -uo pipefail

LOG_DIR="${LOG_DIR:-$PWD/log/smoke}"
mkdir -p "$LOG_DIR"
ros2 launch livox_mid360_sim sim.launch.py gui:=false rviz:=false "$@" \
  > "$LOG_DIR/launch.log" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT "$LAUNCH_PID" 2>/dev/null
  sleep 3
  pkill -f "gz sim -s" 2>/dev/null
  pkill -f "livox_driver_emulator|parameter_bridge|robot_state_publisher|ros_gz_sim/create" 2>/dev/null
}
trap cleanup EXIT

echo "Waiting for /livox/lidar ..."
for _ in $(seq 1 90); do
  if ros2 topic list 2>/dev/null | grep -qx /livox/points; then
    break
  fi
  sleep 1
done
# Give the server time to load the world, spawn the robot and render.
sleep 8

grep -a -E "LivoxMid360|LivoxRaycast|\[Err\]|rror" "$LOG_DIR/launch.log" | head -20

GZ_PID=$(pgrep -f "gz sim -s" | head -1)
if [ -n "$GZ_PID" ]; then
  # Sample the server's CPU usage while the check runs.
  (for _ in 1 2 3 4 5; do ps -o %cpu= -p "$GZ_PID"; sleep 1; done) > "$LOG_DIR/cpu.txt" &
fi

ros2 run livox_mid360_sim check_livox_cloud.py --duration 8
STATUS=$?

if [ -s "$LOG_DIR/cpu.txt" ]; then
  awk '{s+=$1; n++} END {if (n) printf "gz server CPU: %.0f%% (avg of %d samples, 100%% = 1 core)\n", s/n, n}' \
    "$LOG_DIR/cpu.txt"
fi
exit $STATUS
