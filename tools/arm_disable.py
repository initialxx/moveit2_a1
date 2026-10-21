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
    source setup_env.sh
    python3 tools/arm_disable.py --channel can0 --model A1R
    python3 tools/arm_disable.py --both

    --channel slcan0     单臂 CAN 接口（默认 slcan0）
    --model A1R          单臂型号（默认 A1R）
    --both               依次向左、右臂发送失能指令
    --left-channel can1  双臂模式左臂接口（默认 can1，型号 A1L）
    --right-channel can0 双臂模式右臂接口（默认 can0，型号 A1R）
    --yes                跳过确认
"""

from __future__ import annotations

import argparse
import sys


def parse_args(argv=None):
    p = argparse.ArgumentParser(description="让机械臂卸力")
    p.add_argument("--channel", help="单臂 CAN 接口，默认 slcan0")
    p.add_argument("--model", help="单臂型号，默认 A1R")
    p.add_argument("--both", action="store_true", help="依次失能两条臂")
    p.add_argument("--left-channel", help="双臂左侧 CAN 接口，默认 can1")
    p.add_argument("--right-channel", help="双臂右侧 CAN 接口，默认 can0")
    p.add_argument("--yes", action="store_true", help="跳过确认")
    args = p.parse_args(argv)
    if args.both:
        if args.channel is not None or args.model is not None:
            p.error("--both 使用 --left-channel/--right-channel，不与 --channel/--model 混用")
        args.targets = [(args.left_channel if args.left_channel is not None else "can1", "A1L"),
                        (args.right_channel if args.right_channel is not None else "can0", "A1R")]
    else:
        if args.left_channel is not None or args.right_channel is not None:
            p.error("--left-channel/--right-channel 需要同时指定 --both")
        args.targets = [(args.channel if args.channel is not None else "slcan0",
                         args.model if args.model is not None else "A1R")]
    args.targets = [(channel.strip(), model.strip()) for channel, model in args.targets]
    if any(not channel for channel, _ in args.targets):
        p.error("CAN 接口名不能为空")
    if any(not model for _, model in args.targets):
        p.error("型号不能为空")
    if len({channel for channel, _ in args.targets}) != len(args.targets):
        p.error("两条臂必须使用不同的 CAN 接口")
    return args


def _disable_arm(channel, model_name) -> int:
    backend = None
    rc = 0
    try:
        from onero_sdk.can_backend import CanBackend
        from onero_sdk.model_registry import get_model

        model = get_model(model_name)
        print(f"\n  连接 {channel} | {model_name} ...")
        backend = CanBackend(channel=channel, joint_specs=model.joints, recv_timeout=0.003)
        try:
            state = backend.get_state()
            print("  卸力前位置: " + "  ".join(f"{v:+.3f}" for v in state.joints))
        except Exception as exc:
            print(f"  （读位置失败，继续卸力: {exc}）")
        backend.disable()
        print(f"  ✓ {channel} | {model_name} 已发送失能指令")
    except Exception as exc:
        print(f"  ✗ {channel} | {model_name} 卸力失败: {type(exc).__name__}: {exc}")
        rc = 2
    finally:
        if backend is not None:
            try:
                backend.close(disable=False)
                print(f"  ✓ {channel} CAN 已关闭")
            except Exception as exc:
                print(f"  ✗ {channel} CAN 关闭失败: {exc}")
                rc = 2
    return rc


def main(argv=None) -> int:
    a = parse_args(argv)

    print("=" * 66)
    print(" 机械臂卸力")
    print("=" * 66)
    for channel, model in a.targets:
        print(f"  接口 {channel} | 型号 {model}")
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

    results = [_disable_arm(channel, model) for channel, model in a.targets]
    if a.both:
        print(f"\n  失能操作成功：{results.count(0)}/{len(results)} 条臂")
    return 0 if all(result == 0 for result in results) else 2


if __name__ == "__main__":
    sys.exit(main())
