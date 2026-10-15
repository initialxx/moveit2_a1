"""让机械臂卸力（安全收尾工具）。

为什么需要它
------------
真机跑完 RViz / 轨迹测试后，电机**仍然是使能状态**，因为：

  * `topic_bridge` 正常退出时调的是 `close(disable=False)` ——
    刻意不自动卸力（带载的臂突然卸力会掉下来）
  * 用 Ctrl+C 停 launch 时，桥接节点被 SIGINT 终止，
    更不会主动发失能帧

结果就是：机械臂保持刚性、但已经没有任何东西在命令它。
这个工具用来**显式、可控地**把它放开。

⚠️ 卸力 = 电机不再出力。带载且无支撑的机械臂**会掉下来**。
   确认机械臂处于支撑状态，或该姿态下能靠摩擦/结构自己撑住再用。

用法
----
    PYTHONPATH=/home/initialxx/project/armsdk python3 tools/arm_disable.py

    --channel slcan0    CAN 接口
    --model A1R         型号
    --yes               跳过确认
"""

from __future__ import annotations

import argparse
import sys


def parse_args():
    p = argparse.ArgumentParser(description="让机械臂卸力")
    p.add_argument("--channel", default="slcan0")
    p.add_argument("--model", default="A1R")
    p.add_argument("--yes", action="store_true", help="跳过确认")
    return p.parse_args()


def main() -> int:
    a = parse_args()

    print("=" * 66)
    print(" 机械臂卸力")
    print("=" * 66)
    print(f"  接口 {a.channel} | 型号 {a.model}")
    print()
    print("  ⚠️  卸力后电机不再出力。带载且无支撑的机械臂会掉下来。")
    print()

    if not a.yes and sys.stdin.isatty():
        try:
            if input("  确认机械臂已处于支撑状态？输入 yes 继续: ").strip().lower() \
                    not in ("y", "yes"):
                print("  已取消。")
                return 1
        except (EOFError, KeyboardInterrupt):
            print("\n  已取消。")
            return 1
    else:
        print("  （--yes 或非交互环境，跳过确认）")

    from onero_sdk.can_backend import CanBackend
    from onero_sdk.model_registry import get_model

    model = get_model(a.model)
    print(f"\n  连接 {a.channel} ...")
    be = CanBackend(channel=a.channel, joint_specs=model.joints, recv_timeout=0.003)
    print("  ✓ 已连接")

    rc = 0
    try:
        state = be.get_state()
        print("  卸力前位置: " + "  ".join(f"{v:+.3f}" for v in state.joints))
    except Exception as e:  # noqa: BLE001
        print(f"  （读位置失败，继续卸力: {e}）")

    try:
        be.disable()
        print("  ✓ 已发送失能指令 —— 机械臂现在是松的")
    except Exception as e:  # noqa: BLE001
        print(f"  ✗ 卸力失败: {type(e).__name__}: {e}")
        rc = 2
    finally:
        try:
            be.close(disable=False)
        except Exception:  # noqa: BLE001
            pass
        print("  ✓ CAN 已关闭")

    return rc


if __name__ == "__main__":
    sys.exit(main())
