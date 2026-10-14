"""使能测试：只上力、保持当前位置、不发任何轨迹。

用途
----
真机接入的**第二步**（第一步是只读状态，已完成）。
目标是回答三个问题，且**不产生任何运动意图**：

    1. 上力后机械臂能不能稳住？（不发散、不抖）
    2. 保持精度如何？（目标 vs 实际）
    3. 电机本体状态正常吗？（力矩、温度、错误码）

安全设计（重要）
----------------
1. **软启动**：kp 从 0 线性升到目标值（默认 1.5 s）。
   直接给满 kp 会让电机瞬间变钢，可能产生冲击；渐进上力温和得多。
2. **目标位置固定 = 使能瞬间的实测位置**，全程不变。
   即"保持在它现在所在的地方"，不产生任何运动意图。
3. **不发轨迹**：不涉及 move_group / JTC / 话题桥。
4. **退出必收尾**：`finally` 保证无论正常结束、异常还是 Ctrl+C，
   都会按 `--on-exit` 的设定处置电机（默认 `disable` 卸力）。
5. **硬超时**：即使主循环出问题，`--duration` 到了也会退出。

用法
----
    PYTHONPATH=/home/initialxx/project/armsdk python3 tools/test_enable_hold.py

    # 常用参数
    --duration 10        保持多少秒（默认 10）
    --kp 50 --kd 1.0     MIT 增益（默认与节点一致）
    --ramp 1.5           软启动时长秒（0 = 不软启）
    --on-exit disable   退出时处置：disable（默认）/ hold
    --channel slcan0     CAN 接口
    --model A1R          型号

⚠️ 上力前请确认：机械臂已固定、运动范围内无人、急停在手边。
"""

from __future__ import annotations

import argparse
import math
import sys
import time

BASELINE_HINT = "上力前请确认：机械臂已固定、范围内无人、急停在手边"


def parse_args():
    p = argparse.ArgumentParser(description="使能保持测试（不发轨迹）")
    p.add_argument("--channel", default="slcan0", help="CAN 接口（默认 slcan0）")
    p.add_argument("--model", default="A1R", help="型号（默认 A1R）")
    p.add_argument("--kp", type=float, default=50.0)
    p.add_argument("--kd", type=float, default=1.0)
    p.add_argument("--duration", type=float, default=10.0, help="保持秒数")
    p.add_argument("--ramp", type=float, default=1.5, help="软启动秒数，0=不软启")
    p.add_argument("--rate", type=float, default=200.0, help="控制频率 Hz")
    p.add_argument("--on-exit", choices=["disable", "hold"], default="disable",
                   help="退出时：disable 卸力（默认）/ hold 保持上力")
    p.add_argument("--yes", action="store_true", help="跳过交互确认")
    return p.parse_args()


def fmt(vals, width=9):
    return "  ".join(f"{v:+{width}.3f}" for v in vals)


def main() -> int:
    a = parse_args()

    print("=" * 74)
    print(" 使能保持测试 —— 只上力，不发轨迹")
    print("=" * 74)
    print(f"  接口 {a.channel} | 型号 {a.model} | kp={a.kp} kd={a.kd}")
    print(f"  软启动 {a.ramp}s | 保持 {a.duration}s | 退出处置 {a.on_exit}")
    print()
    print(f"  ⚠️  {BASELINE_HINT}")
    print()

    if not a.yes and sys.stdin.isatty():
        try:
            if input("  确认无误请输入 yes 继续: ").strip().lower() not in ("y", "yes"):
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
    n = model.num_joints

    print(f"\n  [1/4] 连接 {a.channel} ...")
    be = CanBackend(channel=a.channel, joint_specs=model.joints, recv_timeout=0.003)
    print("        ✓ 已连接（尚未使能）")

    # 使能前的实测位置 —— 作为全程固定的目标
    pre = be.get_state()
    target = [float(v) for v in pre.joints]
    print(f"        ✓ 使能前位置: {fmt(target)}")
    if any(not math.isfinite(v) for v in target):
        print("        ✗ 读到的位置含非有限值，已中止（不使能）")
        be.close(disable=False)
        return 2

    dt = 1.0 / a.rate
    enabled = False
    traj = []           # (t, pos, vel, tau)
    t_start = None

    try:
        print(f"\n  [2/4] 使能 + 软启动（kp 0 → {a.kp}，{a.ramp}s）...")
        be.enable()
        enabled = True
        print("        ✓ 电机已使能")

        t_start = time.monotonic()
        next_t = t_start
        hold_until = t_start + a.duration
        last_print = 0.0
        state = pre

        while True:
            now = time.monotonic()
            if now >= hold_until:
                break

            elapsed = now - t_start
            # 软启动：kp 线性爬升
            if a.ramp > 0:
                kp = a.kp * min(1.0, elapsed / a.ramp)
            else:
                kp = a.kp

            state = be.command(
                positions=target,               # 目标恒为初始位置
                velocities=[0.0] * n,
                kps=[kp] * n,
                kds=[a.kd] * n,
                torques=[0.0] * n,
            )
            traj.append((elapsed, list(state.joints), list(state.velocities),
                         list(state.torques)))

            # 每秒打一次进度
            if elapsed - last_print >= 1.0:
                last_print = elapsed
                err = max(abs(p - t) for p, t in zip(state.joints, target))
                vel = max(abs(v) for v in state.velocities)
                print(f"        t={elapsed:5.1f}s  kp={kp:5.1f}  "
                      f"最大误差 {err:.4f} rad  最大速度 {vel:.3f} rad/s")

            next_t += dt
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()

        # ── 汇总 ─────────────────────────────────────────────
        print(f"\n  [3/4] 保持结果（共 {len(traj)} 个控制周期）")
        final_pos = traj[-1][1]
        final_vel = traj[-1][2]
        final_tau = traj[-1][3]

        print(f"  关节     目标       最终       误差       速度      力矩")
        print("  " + "-" * 66)
        max_err = 0.0
        max_vel = 0.0
        for j in range(n):
            e = final_pos[j] - target[j]
            max_err = max(max_err, abs(e))
            max_vel = max(max_vel, abs(final_vel[j]))
            print(f"  joint{j+1} {target[j]:+8.3f} {final_pos[j]:+8.3f} "
                  f"{e:+8.4f} {final_vel[j]:+9.3f} {final_tau[j]:+9.3f}")

        # 稳定性：后半段误差是否收敛（发散 = 危险）
        half = traj[len(traj) // 2:]
        errs = [max(abs(p - t) for p, t in zip(s[1], target)) for s in half]
        print()
        print(f"  最大跟踪误差 : {max_err:.4f} rad")
        print(f"  最大速度     : {max_vel:.4f} rad/s")
        print(f"  后半段误差   : 首 {errs[0]:.4f} → 末 {errs[-1]:.4f} rad")
        trend = errs[-1] - errs[0]
        if abs(max_err) < 0.05 and max_vel < 0.1:
            print("  ✓ 判定：稳住了（误差与速度都在合理范围）")
        elif trend > 0.02:
            print("  ⚠️ 判定：误差在增大，可能在发散 —— 建议降低 kp 或检查零位")
        else:
            print("  ⚠️ 判定：数值偏大，需人工核对")

    except KeyboardInterrupt:
        print("\n  ⚠️ 收到 Ctrl+C")
    except Exception as e:  # noqa: BLE001
        print(f"\n  ✗ 异常: {type(e).__name__}: {e}")
    finally:
        print(f"\n  [4/4] 收尾（--on-exit {a.on_exit}）...")
        try:
            if a.on_exit == "disable":
                if enabled:
                    be.disable()
                    print("        ✓ 已卸力（机械臂会变松）")
                else:
                    print("        （从未使能，无需卸力）")
            else:
                print("        ⚠️ 保持上力 —— 电机仍硬，注意机械臂状态")
        except Exception as e:  # noqa: BLE001
            print(f"        ✗ 收尾失败: {type(e).__name__}: {e}")
            print("        ⚠️ 机械臂可能仍处于使能状态，请手动处置")
        try:
            be.close(disable=False)
        except Exception:  # noqa: BLE001
            pass
        print("        ✓ CAN 已关闭")

    return 0


if __name__ == "__main__":
    sys.exit(main())
