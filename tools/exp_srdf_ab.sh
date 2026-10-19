#!/usr/bin/env bash
# SRDF 碰撞矩阵 A/B 对照实验
#
# 做法：同一套 MoveIt 配置，只替换 SRDF 文件，分别测量「状态有效性」判定结果。
#       两份 SRDF 除 disable_collisions 外完全一致（已 diff 验证），
#       因此差异只能归因于碰撞矩阵。
#
# 测量使用 sim 模式，不连接 CAN。
#
# 注意：本脚本会临时改写 src 下的 SRDF，结束后用 git 恢复。
#       若中途中断，请手动执行 git checkout -- <srdf 路径>。

# 不要用 set -u：ROS 的 setup.bash 会引用未绑定变量并中止
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"

SRDF_REL="src/onero_a1_moveit_config/config/onero_a1.srdf"
SRDF="$WS/$SRDF_REL"
FOREIGN="/home/initialxx/project/armsdk/ros2/onero_moveit_config/config/a1.srdf"
LOGDIR="$WS/.logs"
mkdir -p "$LOGDIR"

[ -f "$FOREIGN" ] || { echo "✗ 找不到对照 SRDF: $FOREIGN"; exit 1; }

restore() {
  git -C "$WS" checkout -- "$SRDF_REL" 2>/dev/null
  pkill -f "move_group" 2>/dev/null
  pkill -f "ros2_control_node" 2>/dev/null
  pkill -f "robot_state_publisher" 2>/dev/null
}
trap restore EXIT

source "$WS/setup_env.sh" > /dev/null 2>&1
cleanup() {
  pkill -f "move_group" 2>/dev/null
  pkill -f "ros2_control_node" 2>/dev/null
  pkill -f "robot_state_publisher" 2>/dev/null
  sleep 2
}

run_variant() {
  local label="$1" log="$LOGDIR/exp_srdf_${label}.launch.log"
  cleanup
  ros2 launch onero_a1_moveit_config moveit.launch.py \
      rviz:=false hardware_mode:=sim > "$log" 2>&1 &

  local i
  for i in $(seq 1 60); do
    grep -q "You can start planning now" "$log" 2>/dev/null && break
    sleep 1
  done
  if ! grep -q "You can start planning now" "$log" 2>/dev/null; then
    echo "  ✗ move_group 未就绪，见 $log"
    tail -20 "$log"
    return 1
  fi
  sleep 3
  python3 tools/exp_srdf_validity.py "$label" 2>&1 | grep -v "^\["
}

echo "================================================================"
echo " 变体 A：当前工作区 SRDF（碰撞矩阵来自 SDK 权威文件，28 对）"
echo "================================================================"
run_variant "workspace(28对)" || true

echo
echo "================================================================"
echo " 变体 B：armsdk/ros2 的 a1.srdf（碰撞矩阵 10 对，缺 18 对）"
echo "================================================================"
cp "$SRDF" "$LOGDIR/srdf_backup.srdf"
cp "$FOREIGN" "$SRDF"
run_variant "armsdk_ros2(10对)" || true

# 立即恢复，不依赖 trap
cp "$LOGDIR/srdf_backup.srdf" "$SRDF"
echo
echo "================================================================"
echo " SRDF 已恢复。校验："
echo "   disable_collisions 条目数 = $(grep -c disable_collisions "$SRDF")（应为 28）"
echo "   git 状态：$(git -C "$WS" status --short "$SRDF_REL" | wc -l) 处改动（应为 0）"
echo "================================================================"
