#!/usr/bin/env bash
# 干净克隆构建检查 —— rosdistro 的验收标准
#
# 为什么需要它：
#   `git archive HEAD` 只包含**被 git 追踪的**文件。空目录不会被追踪，
#   而 CMake 的 install(DIRECTORY) 遇到不存在的目录会失败。
#   这类问题在本机（有未追踪文件/残留产物）永远发现不了，
#   但 bloom / ROS build farm 是从干净源码树构建的，会直接失败。
#
# 本脚本：把 HEAD 导出到临时目录 → 当作全新工作区 → colcon build
#         任何失败都会打印出来。
#
# 用法: bash tools/check_clean_build.sh

# 不要用 set -u（ROS 的 setup.bash 会引用未绑定变量）
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPO="$WS"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "================================================================"
echo " 干净克隆构建检查"
echo "================================================================"
echo "  源仓库   : $REPO"
echo "  临时目录 : $TMP"

#  0) 提醒：本检查导出的是 HEAD，不是工作区。
#     未提交的修改不会被测到 —— 改了 CMakeLists 之类要先 commit 再跑。
if [ -n "$(git -C "$REPO" status --porcelain)" ]; then
  echo
  echo "  ⚠ 工作区有未提交改动，本检查测的是 HEAD（不含这些改动）："
  git -C "$REPO" status --porcelain | head -8 | sed 's/^/      /'
fi

# 1) 从 git 导出 HEAD（等价于一次全新 clone）
git -C "$REPO" archive HEAD | tar -x -C "$TMP"
SRC="$TMP/src"
if [ ! -d "$SRC" ]; then
  echo "  ✗ 导出结果里没有 src"
  exit 1
fi
echo "  导出包   : $(ls "$SRC" | tr '\n' ' ')"

# 2) 列出「源码树里缺了什么」（git 追踪的空目录会在这里现形）
echo
echo "--- 各包 CMakeLists 声明的安装目录 vs 实际存在情况 ---"
FAIL_DIR=0
for cm in "$SRC"/*/CMakeLists.txt; do
  pkg="$(basename "$(dirname "$cm")")"
  dirs=$(sed -n '/^install(/,/^)/p' "$cm" | grep -i DIRECTORY | sed 's/.*DIRECTORY//')
  [ -z "$dirs" ] && continue
  for d in $dirs; do
    if [ -d "$(dirname "$cm")/$d" ]; then
      printf "  %-26s %-12s ✓\n" "$pkg" "$d"
    else
      printf "  %-26s %-12s ✗ 不存在（install(DIRECTORY) 会失败）\n" "$pkg" "$d"
      FAIL_DIR=1
    fi
  done
done

# 3) 真正编一遍
echo
echo "--- colcon build（在干净树上） ---"
source /opt/ros/jazzy/setup.bash > /dev/null 2>&1
export ROS_HOME="$TMP/.ros"
export ROS_LOG_DIR="$ROS_HOME/log"
mkdir -p "$ROS_LOG_DIR"
# 临时 overlay（本容器无 root，用解包 .deb 顶替 apt 包）
if [ -d "$WS/overlay/share" ]; then
  export AMENT_PREFIX_PATH="$WS/overlay:${AMENT_PREFIX_PATH}"
  export LD_LIBRARY_PATH="$WS/overlay/lib:${LD_LIBRARY_PATH}"
  export CMAKE_PREFIX_PATH="$WS/overlay:${CMAKE_PREFIX_PATH}"
fi

mkdir -p "$TMP/ws/src"
cp -r "$SRC"/* "$TMP/ws/src/"
cd "$TMP/ws" || exit 1
BUILD_LOG="$TMP/build.log"
if colcon build --symlink-install > "$BUILD_LOG" 2>&1; then
  echo "  ✓ 干净树构建通过"
  grep -E "^Summary" "$BUILD_LOG" | sed 's/^/    /'
  BUILD_RC=0
else
  echo "  ✗ 干净树构建失败："
  grep -B3 -A8 -iE "CMake Error|Failed   <<<" "$BUILD_LOG" | head -30 | sed 's/^/    /'
  BUILD_RC=1
fi

echo
echo "================================================================"
if [ "$FAIL_DIR" = 0 ] && [ "$BUILD_RC" = 0 ]; then
  echo " 结论：干净克隆可构建 ✓"
else
  echo " 结论：干净克隆存在问题 ✗（缺目录=$FAIL_DIR, 构建失败=$BUILD_RC）"
fi
echo "================================================================"
exit $(( FAIL_DIR + BUILD_RC > 0 ? 1 : 0 ))
