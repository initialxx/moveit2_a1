#!/usr/bin/env bash
# Onero A1 ROS 2 工作区环境
#
# 用法:  source setup_env.sh
#
# 说明:
#   1) 本环境（容器）中 /opt/ros 与 $HOME 为只读，ros2 launch 默认要写
#      ~/.ros/log 会直接失败，因此必须把 ROS_HOME / ROS_LOG_DIR 指到工作区内。
#   2) overlay/ 是一个【临时替代方案】: 当系统未安装
#      ros-jazzy-joint-state-topic-hardware-interface 时，用解包的 .deb 顶替，
#      以便在无 root 权限下验证全链路。
#      正式环境请用 apt 安装，然后删掉 overlay/。

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash

# 只读文件系统的规避
export ROS_HOME="${WS_DIR}/.ros"
export ROS_LOG_DIR="${ROS_HOME}/log"
mkdir -p "${ROS_LOG_DIR}"

# 临时 overlay（见上文说明 2）
if [ -d "${WS_DIR}/overlay/share" ]; then
  export AMENT_PREFIX_PATH="${WS_DIR}/overlay:${AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="${WS_DIR}/overlay/lib:${LD_LIBRARY_PATH}"
  export CMAKE_PREFIX_PATH="${WS_DIR}/overlay:${CMAKE_PREFIX_PATH}"
  OVERLAY_STATUS="已启用（临时替代 apt 安装）"
else
  OVERLAY_STATUS="未使用"
fi

# ── onero-sdk（真机模式必需）──────────────────────────────────────────
# 话题桥在 simulate:=false 时会 import onero_sdk；缺了它节点会在启动时
# 直接抛 RuntimeError 退出，表现为「看门狗一直报启动后从未收到状态」。
#
# 优先用已安装的（pip install onero-sdk）；
# 没装则回退到源码目录，可用 ONERO_SDK_DIR 覆盖。
if python3 -c "import onero_sdk" 2>/dev/null; then
  SDK_STATUS="已安装（import 可用）"
else
  ONERO_SDK_DIR="${ONERO_SDK_DIR:-/home/initialxx/project/armsdk}"
  if [ -d "${ONERO_SDK_DIR}/onero_sdk" ]; then
    export PYTHONPATH="${ONERO_SDK_DIR}:${PYTHONPATH:-}"
    SDK_STATUS="源码目录 ${ONERO_SDK_DIR}"
  else
    SDK_STATUS="✗ 缺失 —— 真机模式会启动失败（pip install onero-sdk，或设 ONERO_SDK_DIR）"
  fi
fi

# Use the isolated SDK runtime only for the hardware bridge. Other ROS nodes
# retain their existing Python environment; the SDK source path above is shared.
if [ -x "${WS_DIR}/.venv-sdk/bin/python" ]; then
  export ONERO_SDK_PYTHON="${ONERO_SDK_PYTHON:-${WS_DIR}/.venv-sdk/bin/python}"
fi

# 工作区 overlay
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  source "${WS_DIR}/install/setup.bash"
fi

echo "Onero A1 工作区已就绪"
echo "  WS_DIR      = ${WS_DIR}"
echo "  ROS_HOME    = ${ROS_HOME}"
echo "  ROS_DISTRO  = ${ROS_DISTRO}"
echo "  overlay     = ${OVERLAY_STATUS}"
echo "  onero-sdk   = ${SDK_STATUS}"

# 提示话题桥插件是否可用
if ros2 pkg prefix joint_state_topic_hardware_interface >/dev/null 2>&1; then
  echo "  话题桥插件  = 可用 ✓"
else
  echo "  话题桥插件  = ✗ 缺失 —— 请 sudo apt install ros-jazzy-joint-state-topic-hardware-interface"
fi
