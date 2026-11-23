#!/bin/bash
# Replay a recorded sensor bag into the estimator -- no Gazebo, no ArduPilot -- and record the
# estimate together with the truth for evaluate.py.
#
#   lrs_localization/scripts/replay.sh bags/a2_flight_seed42 /tmp/replay_flight [rate]
#   python3 lrs_localization/scripts/evaluate.py /tmp/replay_flight --out /tmp/figures
#
# The estimator works on header stamps only, so a faster replay (rate > 1) gives the same result.
# Run from the repository root after `colcon build` and `source install/setup.bash`.
set -e
IN=${1:?usage: replay.sh IN_BAG OUT_BAG [rate]}
OUT=${2:?usage: replay.sh IN_BAG OUT_BAG [rate]}
RATE=${3:-2}

SHARE=$(ros2 pkg prefix lrs_localization)
NODE="$SHARE/lib/lrs_localization/localization_node"
PARAMS="$SHARE/share/lrs_localization/config/localization.yaml"
LOG="${OUT%/}.node.log"
rm -rf "$OUT"

# Background jobs of a script ignore SIGINT, so everything is stopped with SIGTERM.
"$NODE" --ros-args --params-file "$PARAMS" >"$LOG" 2>&1 &
NODE_PID=$!
ros2 bag record -s mcap -o "$OUT" /lrs_localization/odom /evaluation/ground_truth /localization/gps \
  >/dev/null 2>&1 &
REC_PID=$!
trap 'kill -TERM $NODE_PID $REC_PID 2>/dev/null' EXIT
sleep 2

ros2 bag play "$IN" --clock 100 --rate "$RATE" >/dev/null
sleep 1
kill -TERM $REC_PID; wait $REC_PID 2>/dev/null || true
kill -TERM $NODE_PID; wait $NODE_PID 2>/dev/null || true
echo "estimate + truth: $OUT    node log: $LOG"
grep -E '\[filter\]' "$LOG" | sed -E 's/^\[INFO\] \[[0-9.]+\] \[lrs_localization\]: //' | tail -4
