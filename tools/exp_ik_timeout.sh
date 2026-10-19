#!/usr/bin/env bash
# 启动 move_group（纯规划模式）后运行 IK 超时实测
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS" || exit 1
LOG="$WS/.logs/exp_ik.launch.log"

pkill -f "move_group" 2>/dev/null; pkill -f "ros2_control_node" 2>/dev/null; sleep 3
source "$WS/setup_env.sh" > /dev/null 2>&1

ros2 launch onero_a1_moveit_config moveit.launch.py \
    rviz:=false hardware_mode:=sim > "$LOG" 2>&1 &

for i in $(seq 1 60); do
  grep -q "You can start planning now" "$LOG" 2>/dev/null && break
  sleep 1
done
if ! grep -q "You can start planning now" "$LOG" 2>/dev/null; then
  echo "✗ move_group 未就绪"; tail -15 "$LOG"; exit 1
fi
sleep 3
echo "================================================================"
echo " IK 超时实测（7-DOF KDL）"
echo "================================================================"
python3 tools/exp_ik_timeout.py 2>&1 | grep -v "^\["

pkill -f "move_group" 2>/dev/null; pkill -f "ros2_control_node" 2>/dev/null
