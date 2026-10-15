#!/usr/bin/env python3
"""独立测试话题桥节点（不依赖 ros2_control）。

验证：
  1. 节点能启动并发布 /robot_joint_states
  2. 收到 /robot_joint_commands 后状态能跟随
  3. 关节按名字映射（故意打乱顺序发送）

用法：
    python3 test_bridge.py
需先启动桥接节点：
    ros2 run onero_a1_bringup topic_bridge --ros-args -p simulate:=true
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

JOINTS = [
    "joint1-a1_r", "joint2-a1_r", "joint3-a1_r", "joint4-a1_r",
    "joint5-a1_r", "joint6-a1_r", "joint7-a1_r",
]


class BridgeTest(Node):
    def __init__(self):
        super().__init__("bridge_test")
        self.states = []
        self.create_subscription(JointState, "/robot_joint_states", self._on_state, 10)
        self.pub = self.create_publisher(
            JointState,
            "/robot_joint_commands",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
            ),
        )

    def _on_state(self, msg):
        self.states.append(msg)

    def wait_states(self, n=5, timeout=10.0):
        t0 = time.time()
        while len(self.states) < n and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return len(self.states) >= n

    def send(self, mapping):
        """mapping: {joint_name: position} — 故意乱序"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(mapping.keys())
        msg.position = list(mapping.values())
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BridgeTest()

    print("[1] 等待状态话题有数据 ...")
    if not node.wait_states(5, timeout=15.0):
        print("  ✗ 没收到 /robot_joint_states —— 桥接节点没起来？")
        rclpy.shutdown()
        return 1
    last = node.states[-1]
    print(f"  ✓ 收到 {len(node.states)} 条状态")
    print(f"    关节数: {len(last.name)}  名字: {last.name[:3]}...")
    if not last.velocity:
        print("  ! 警告: velocity 为空")

    print("[2] 发送乱序命令（倒序 + 只发部分关节）...")
    target = {JOINTS[6]: 0.7, JOINTS[0]: 0.3, JOINTS[3]: -0.4}
    for _ in range(60):
        node.send(target)
        rclpy.spin_once(node, timeout_sec=0.02)
    print(f"  发送: {target}")

    print("[3] 等待跟随 ...")
    time.sleep(0.6)
    node.wait_states(len(node.states) + 10, timeout=5.0)

    final = node.states[-1]
    idx = {n: i for i, n in enumerate(final.name)}
    ok = True
    for name, want in target.items():
        got = final.position[idx[name]]
        err = abs(got - want)
        flag = "✓" if err < 0.02 else "✗"
        if err >= 0.02:
            ok = False
        print(f"  {flag} {name}: 目标 {want:+.3f}  实际 {got:+.3f}  误差 {err:.4f}")

    print("[4] 检查未命令的关节是否保持 ...")
    for name in JOINTS[1:3]:
        print(f"    {name}: {final.position[idx[name]]:+.4f}")

    # ── 回归测试：NaN 必须在桥接处被拦下 ────────────────────────────
    # 背景：ros2_control 对【没有 initial_value 的接口】会用 NaN 初始化，
    #      控制器认领命令接口之前话题里会出现 NaN。若桥接原样转发，
    #      NaN 会污染状态并永久传播（NaN 与任何数运算仍是 NaN），
    #      同时让 JTC 的到位判断失效。真机上更会把 NaN 下发给电机。
    print("[5] 回归测试：注入 NaN 命令，验证桥接不会污染状态 ...")
    node.states.clear()
    nan_cmd = {j: 0.3 for j in JOINTS}
    nan_cmd[JOINTS[2]] = float("nan")
    nan_cmd[JOINTS[4]] = float("inf")
    frames = []
    for _ in range(40):
        node.send(nan_cmd)
        rclpy.spin_once(node, timeout_sec=0.02)
    time.sleep(0.3)
    node.wait_states(10, timeout=5.0)
    bad = node.states[-1]
    n_ok = True
    for i, name in enumerate(bad.name):
        v = bad.position[i]
        if v != v or v in (float("inf"), float("-inf")):  # NaN 或 inf
            print(f"    ✗ {name} 被污染: {v}")
            n_ok = False
    if n_ok:
        nan_j = bad.position[bad.name.index(JOINTS[2])]
        inf_j = bad.position[bad.name.index(JOINTS[4])]
        print(f"    ✓ 无 NaN/inf 泄漏（joint3={nan_j:+.4f}, joint5={inf_j:+.4f}）")

    print()
    print("=" * 50)
    print(f"  结果: {'全部通过 ✓' if (ok and n_ok) else '存在失败 ✗'}")
    print("=" * 50)

    rclpy.shutdown()
    return 0 if (ok and n_ok) else 1


if __name__ == "__main__":
    sys.exit(main())
