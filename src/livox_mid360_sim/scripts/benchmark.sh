#!/usr/bin/env bash
# Performance of the simulation on this machine: for each configuration, launch headless,
# let it settle, then measure the gz server CPU% (100% = one core), GPU utilisation and
# memory, and the real-time factor.
#   ros2 run livox_mid360_sim benchmark.sh "dense_rate:=10" "dense_rate:=50" "dense_rate:=100"
set -u
SETTLE=${BENCH_SETTLE:-10}
WINDOW=${BENCH_WINDOW:-10}
[ $# -eq 0 ] && set -- ""
printf '%-28s %8s %8s %10s %8s\n' config RTF "CPU%" "GPU util%" "GPU MiB"
for cfg in "$@"; do
  LOG=$(mktemp -t livox_bench_XXXXXX.log)
  # shellcheck disable=SC2086
  setsid ros2 launch livox_mid360_sim sim.launch.py gui:=false rviz:=false $cfg >"$LOG" 2>&1 </dev/null &
  LP=$!; disown "$LP" 2>/dev/null
  for _ in $(seq 60); do ros2 topic list 2>/dev/null | grep -qx /livox/points && break; sleep 1; done
  sleep "$SETTLE"
  GZ_PID=$(pgrep -f '^gz sim .*-s' | head -1)
  hz=$(getconf CLK_TCK)
  c0=$(awk '{print $14 + $15}' "/proc/$GZ_PID/stat"); w0=$(date +%s.%N)
  GPU=$(nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader,nounits \
        -lms 250 2>/dev/null & GP=$!; sleep "$WINDOW"; kill $GP)
  c1=$(awk '{print $14 + $15}' "/proc/$GZ_PID/stat"); w1=$(date +%s.%N)
  cpu=$(awk -v a="$c0" -v b="$c1" -v t0="$w0" -v t1="$w1" -v hz="$hz" \
        'BEGIN{printf "%.0f", 100*(b-a)/hz/(t1-t0)}')
  gutil=$(echo "$GPU" | awk -F, '{s+=$1; n++} END{if(n) printf "%.0f", s/n}')
  gmem=$(echo "$GPU" | awk -F, '{if($2>m) m=$2} END{printf "%d", m}')
  rtf=$(timeout 10 gz topic -e -t /stats -n 20 2>/dev/null | \
        awk '/real_time_factor/{s+=$2; n++} END{if(n) printf "%.3f", s/n}')
  printf '%-28s %8s %8s %10s %8s\n' "${cfg:-default}" "${rtf:-n/a}" "$cpu" "$gutil" "$gmem"
  kill -INT -- "-$LP" 2>/dev/null; sleep 4; kill -KILL -- "-$LP" 2>/dev/null
  pkill -KILL -f '^gz sim .*-s' 2>/dev/null; pkill -KILL -f '[r]os_gz_sim/create' 2>/dev/null
  sleep 2
done
