#!/usr/bin/env bash
# 逐步定位 armsdk/ros2 的 move_group.launch.py 为何跑不起来，以及修掉表层问题后
# 暴露出的下一层问题（参数命名空间）。
#
# 步骤：
#   1) 原样跑他们的 launch        -> 观察第一个阻断点
#   2) 只去掉 SRDF 注释里的 ": "  -> 观察是否越过，并探测参数路径
#
# 注意：全程不改 armsdk 仓库。步骤 2 用的是 onero_moveit_config 的**副本**。

AS="/home/initialxx/project/armsdk/ros2"
WS="/home/initialxx/project/moveit2_ws"
LOGDIR="$WS/.logs"
PREFIX="$LOGDIR/exp_fake_prefix2"
PATCHED="$LOGDIR/exp_patched_moveit_config"

rm -rf "$PREFIX" "$PATCHED"
mkdir -p "$PREFIX/share/ament_index/resource_index/packages"

# onero_ros2 直接符号链接（只用来找 xacro）
ln -sfn "$AS/onero_ros2" "$PREFIX/share/onero_ros2"
touch "$PREFIX/share/ament_index/resource_index/packages/onero_ros2"

# onero_moveit_config 用副本，方便打补丁
cp -r "$AS/onero_moveit_config" "$PATCHED"
ln -sfn "$PATCHED" "$PREFIX/share/onero_moveit_config"
touch "$PREFIX/share/ament_index/resource_index/packages/onero_moveit_config"

source /opt/ros/jazzy/setup.bash > /dev/null 2>&1
export AMENT_PREFIX_PATH="$PREFIX:${AMENT_PREFIX_PATH}"
export ROS_HOME="$WS/.ros"
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"

probe() {
  for q in \
      "arm.kinematics_solver" \
      "robot_description_kinematics.arm.kinematics_solver" \
      "joint_limits.joint1-a1_r.max_velocity" \
      "robot_description_planning.joint_limits.joint1-a1_r.max_velocity" \
      "planning_plugins" \
      "ompl.planning_plugins"
  do
    out=$(ros2 param get /move_group "$q" 2>&1 | tr '\n' ' ')
    if echo "$out" | grep -qiE "not set|does not|Parameter not"; then
      printf "    %-58s %s\n" "$q" "✗ 不存在"
    else
      printf "    %-58s %s\n" "$q" "✓ $(echo "$out" | sed 's/.*: //' | cut -c1-38)"
    fi
  done
}

run_once() {
  local tag="$1" log="$LOGDIR/exp_step_${tag}.log"
  pkill -f "move_group" 2>/dev/null; pkill -f "rviz2" 2>/dev/null; sleep 3
  ros2 launch onero_moveit_config move_group.launch.py > "$log" 2>&1 &
  sleep 18
  if pgrep -f "moveit_ros_move_group/move_group" >/dev/null; then
    echo "  ✔ move_group 已启动"
    probe
  else
    echo "  ✘ move_group 未启动。关键错误："
    grep -iE "ERROR|Unable to|cannot|No planning|what\(\)" "$log" | head -4 | sed 's/^/      /'
  fi
  pkill -f "move_group" 2>/dev/null; pkill -f "rviz2" 2>/dev/null; sleep 2
}

echo "================================================================"
echo " 步骤 1：原样运行他们的 move_group.launch.py"
echo "================================================================"
run_once "1_original"

echo
echo "================================================================"
echo " 步骤 2：仅去掉 SRDF 注释里的 \": \"（补丁打在副本上）"
echo "================================================================"
sed -i 's/<!-- Planning group: full arm chain -->/<!-- Planning group - full arm chain -->/;
        s/<!-- Virtual joint: fixed to world -->/<!-- Virtual joint - fixed to world -->/' \
    "$PATCHED/config/a1.srdf"
echo "  改动行："
grep -n "Planning group\|Virtual joint" "$PATCHED/config/a1.srdf" | sed 's/^/    /'
echo "  剩余含 \": \" 的行数: $(grep -c ': ' "$PATCHED/config/a1.srdf")"
echo
run_once "2_patched"

echo
echo "（日志目录: $LOGDIR/exp_step_*.log）"
