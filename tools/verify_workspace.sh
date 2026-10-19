#!/usr/bin/env bash
# 工作区迁移后的端到端验收脚本
#
# 背景：DSH 沙箱用 bwrap --die-with-parent 包裹每条 bash 调用，
#       后台进程会随调用结束被杀。因此「启动 + 测试 + 收尾」必须写在同一次调用里。
#
# 用法: bash tools/verify_workspace.sh [关节名] [目标位置]
#
# 注意: 不要开 set -u。ROS 的 /opt/ros/jazzy/setup.bash 会引用未绑定的
#       AMENT_TRACE_SETUP_FILES 并直接中止（表现为脚本一行输出都没有、退出码 127）。
set +u

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

JOINT="${1:-joint1-a1_r}"
TARGET="${2:-0.4}"
LOGDIR="$WS_DIR/.logs"
mkdir -p "$LOGDIR"
LAUNCH_LOG="$LOGDIR/verify_launch.log"
TEST_LOG="$LOGDIR/verify_e2e.log"

cleanup() {
  pkill -f "onero_a1_moveit_config/moveit.launch.py" 2>/dev/null
  pkill -f "ros2_control_node" 2>/dev/null
  pkill -f "move_group" 2>/dev/null
  pkill -f "onero_a1_bringup/topic_bridge" 2>/dev/null
  pkill -f "robot_state_publisher" 2>/dev/null
  pkill -f "health_watchdog" 2>/dev/null
}
trap cleanup EXIT

cleanup
sleep 1

# shellcheck disable=SC1091
source "$WS_DIR/setup_env.sh" >/dev/null 2>&1
echo "=== [1/4] 启动 demo（话题桥模式 / 不连真机）==="
ros2 launch onero_a1_moveit_config moveit.launch.py \
    rviz:=false use_fake_hardware:=false simulate:=true > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

# 等待 move_group 就绪，最多 60s
for i in $(seq 1 60); do
  if grep -q "You can start planning now" "$LAUNCH_LOG" 2>/dev/null; then
    echo "  move_group 就绪（${i}s）"
    break
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "  ✗ launch 进程已退出，日志尾部："
    tail -30 "$LAUNCH_LOG"
    exit 1
  fi
  sleep 1
done
grep -q "You can start planning now" "$LAUNCH_LOG" || { echo "  ✗ 超时未就绪"; tail -30 "$LAUNCH_LOG"; exit 1; }

echo
echo "=== [2/4] 关键进程 ==="
for p in ros2_control_node move_group topic_bridge robot_state_publisher; do
  n=$(pgrep -fc "$p" 2>/dev/null || echo 0)
  printf "  %-24s %s\n" "$p" "$([ "$n" -gt 0 ] && echo "✓ 运行中($n)" || echo "✗ 缺失")"
done

echo
echo "=== [3/4] 端到端「规划 → 执行」：$JOINT → $TARGET rad ==="
python3 tools/test_moveit_e2e.py "$JOINT" "$TARGET" 2>&1 | tee "$TEST_LOG"
E2E_RC=${PIPESTATUS[0]}
echo "  (test_moveit_e2e.py 退出码: $E2E_RC)"

echo
echo "=== [4/4] 结果判定 ==="
if grep -qE "error_code\s*=\s*1" "$TEST_LOG" && grep -qiE "误差|error" "$TEST_LOG"; then
  echo "  ✓ 关键指标命中（见上方输出）"
else
  echo "  ⚠ 未匹配到预期指标，请人工核对 $TEST_LOG"
fi

exit "$E2E_RC"
