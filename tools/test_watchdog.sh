#!/usr/bin/env bash
# 看门狗端到端验证：失联停用 → 恢复重新激活
#
# 写成脚本执行的原因：pkill -f 会匹配调用它的 shell 自身的命令行，
# 若命令行里出现 "topic_bridge" 字面量就会自杀。放进脚本可规避。

# 注意：不要用 `set -u` —— ROS 的 setup.bash 会引用未定义变量，
# 在 set -u 下 source 会直接失败退出（表现为脚本无任何输出）。
cd "$(dirname "${BASH_SOURCE[0]}")/.."
WS="$PWD"
source "${WS}/setup_env.sh" > /dev/null 2>&1

LOG="${WS}/.logs/watchdog_e2e.log"
BRIDGE_LOG="${WS}/.logs/bridge_restart.log"

echo "=== 启动完整链路 ==="
ros2 launch onero_a1_moveit_config moveit.launch.py \
    rviz:=false use_fake_hardware:=false simulate:=true > "${LOG}" 2>&1 &
LAUNCH_PID=$!

timeout 90 bash -c 'until ros2 control list_controllers 2>/dev/null | grep -q arm_controller; do sleep 2; done' >/dev/null
sleep 5

state() { ros2 control list_controllers 2>&1 | grep -oP '^arm_controller\s+\S+\s+\K\S+'; }

echo "[1] 初始状态          : $(state)"

echo "[2] 杀掉桥接节点 ..."
pkill -f "[t]opic_bridge"
sleep 5
S2=$(state)
echo "    停用后状态        : ${S2}"

echo "[3] 重启桥接节点 ..."
ros2 run onero_a1_bringup topic_bridge --ros-args -p simulate:=true > "${BRIDGE_LOG}" 2>&1 &
BRIDGE_PID=$!
sleep 10
S3=$(state)
echo "    恢复后状态        : ${S3}"

echo
echo "=== 看门狗日志 ==="
grep -iE "看门狗已启动|失联|已停用|已重新激活|状态已恢复" "${LOG}" | tail -6

echo
echo "=== 判定 ==="
PASS=1
[ "${S2}" = "inactive" ] && echo "  ✓ 失联后自动停用" || { echo "  ✗ 失联后未停用（实际 ${S2}）"; PASS=0; }
[ "${S3}" = "active" ]   && echo "  ✓ 恢复后自动重新激活" || { echo "  ✗ 恢复后未激活（实际 ${S3}）"; PASS=0; }

kill "${BRIDGE_PID}" 2>/dev/null
kill "${LAUNCH_PID}" 2>/dev/null
pkill -f "[t]opic_bridge" 2>/dev/null
wait 2>/dev/null

echo
[ "${PASS}" = "1" ] && echo "看门狗验证通过 ✓" || echo "看门狗验证失败 ✗"
exit $((1 - PASS))
