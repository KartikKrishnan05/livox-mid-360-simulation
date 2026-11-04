#!/usr/bin/env bash
# Headless smoke test: launch the simulation (no GUI / RViz), wait for /livox/points,
# run check_livox_cloud.py, report gz server CPU% and real-time factor, then tear
# everything down (including ros_gz_sim create).
#
#   ros2 run livox_mid360_gazebo smoke_test.sh [extra launch args, e.g. xfer_format:=0]
#
# Env: SMOKE_FRAMES (20), SMOKE_TIMEOUT (120 s), SMOKE_GROUND_Z (-0.722, test rig;
#      "" disables the ground check), SMOKE_LOG (log file).
set -u

FRAMES=${SMOKE_FRAMES:-20}
TIMEOUT=${SMOKE_TIMEOUT:-120}
GROUND_Z=${SMOKE_GROUND_Z--0.722}
LOG=${SMOKE_LOG:-$(mktemp -t livox_smoke_XXXXXX.log)}

LAUNCH_PID=""
cleanup() {
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null
    for _ in $(seq 20); do kill -0 "$LAUNCH_PID" 2>/dev/null || break; sleep 0.5; done
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
  fi
  # Stragglers that may escape the process group (gz server, spawner).
  pkill -f '[r]os_gz_sim/create' 2>/dev/null
  pkill -f '^gz sim .*-s' 2>/dev/null
  pkill -f '[l]ivox_driver_emulator' 2>/dev/null
  pkill -f '[l]ivox_gz_bridge' 2>/dev/null
  sleep 1
  pkill -KILL -f '^gz sim .*-s' 2>/dev/null
  pkill -KILL -f '[r]os_gz_sim/create' 2>/dev/null
  true
}
trap cleanup EXIT INT TERM

echo "smoke_test: launching (log: $LOG)"
setsid ros2 launch livox_mid360_gazebo sim.launch.py gui:=false rviz:=false "$@" \
  >"$LOG" 2>&1 </dev/null &
LAUNCH_PID=$!
disown "$LAUNCH_PID" 2>/dev/null

echo -n "smoke_test: waiting for /livox/points "
t0=$(date +%s)
until ros2 topic list 2>/dev/null | grep -qx /livox/points; do
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null || [ $(( $(date +%s) - t0 )) -gt "$TIMEOUT" ]; then
    echo; echo "smoke_test: FAIL - /livox/points never appeared"; tail -30 "$LOG"; exit 1
  fi
  echo -n "."; sleep 1
done
echo " ok ($(( $(date +%s) - t0 )) s)"

# The server is the process whose argv starts with "gz sim" (not its "sh -c ruby ..." parent).
GZ_PID=$(pgrep -f '^gz sim .*-s' | head -1)

CHECK_ARGS=(--frames "$FRAMES" --timeout "$TIMEOUT")
[ -n "$GROUND_Z" ] && CHECK_ARGS+=(--ground-z "$GROUND_Z")
ros2 run livox_mid360_gazebo check_livox_cloud.py "${CHECK_ARGS[@]}"
RC=$?

# CPU% of the gz server over 5 s (all threads; 100% = one core) and real-time factor.
if [ -n "$GZ_PID" ]; then
  ticks() { awk '{print $14 + $15}' "/proc/$GZ_PID/stat" 2>/dev/null; }
  c0=$(ticks); w0=$(date +%s.%N); sleep 5; c1=$(ticks); w1=$(date +%s.%N)
  hz=$(getconf CLK_TCK)
  cpu=$(awk -v a="$c0" -v b="$c1" -v t0="$w0" -v t1="$w1" -v hz="$hz" \
        'BEGIN{printf "%.0f", 100*(b-a)/hz/(t1-t0)}')
  echo "smoke_test: gz server pid $GZ_PID CPU ${cpu}% (100% = 1 core, $(nproc) cores)"
else
  echo "smoke_test: gz server pid not found"
fi
RTF=$(timeout 10 gz topic -e -t /stats -n 5 2>/dev/null | awk '/real_time_factor/{s+=$2; n++} END{if(n) printf "%.3f", s/n}')
echo "smoke_test: real-time factor ${RTF:-n/a}"
if command -v nvidia-smi >/dev/null; then
  echo "smoke_test: GPU $(nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader)"
fi

if [ $RC -eq 0 ]; then echo "smoke_test: PASS"; else echo "smoke_test: FAIL"; fi
exit $RC
