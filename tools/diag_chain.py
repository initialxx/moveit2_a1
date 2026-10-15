#!/usr/bin/env python3
"""链路诊断：手动往 /robot_joint_commands 注入命令，看每一段是否响应。

用于定位「JTC 说成功但关节不动」这类问题出在哪一段。

用法：
    python3 diag_chain.py
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

JOINTS = [
    "joint1-a1_r", "joint2-a1_r", "joint3-a1_r", "joint4-a1_r",
    "joint5-a1_r", "joint6-a1_r", "joint7-a1_r",
]

TEST = {j: 0.25 for j in JOINTS}


class Diag(Node):
    def __init__(self):
        super().__init__("diag_chain")
        q = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.cmds = []
        self.rstates = []
        self.jstates = []
        self.create_subscription(JointState, "/robot_joint_commands", lambda m: self.cmds.append(m), q)
        self.create_subscription(JointState, "/robot_joint_states", lambda m: self.rstates.append(m), q)
        self.create_subscription(JointState, "/joint_states", lambda m: self.jstates.append(m), 10)
        self.pub = self.create_publisher(JointState, "/robot_joint_commands", q)

    def spin(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)

    def send(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(TEST.keys())
        m.position = list(TEST.values())
        self.pub.publish(m)

    @staticmethod
    def snapshot(msgs, label):
        if not msgs:
            print(f"    {label}: 无数据 ✗")
            return None
        m = msgs[-1]
        idx = {n: i for i, n in enumerate(m.name)}
        v = m.position[idx["joint1-a1_r"]] if "joint1-a1_r" in idx else float("nan")
        print(f"    {label}: {len(msgs)} 条, joint1 = {v:+.4f}")
        return v


def main():
    rclpy.init()
    n = Diag()

    print("[0] 观察 2 秒基线 ...")
    n.spin(2.0)
    print("  基线：")
    Diag.snapshot(n.cmds, "/robot_joint_commands")
    Diag.snapshot(n.rstates, "/robot_joint_states")
    Diag.snapshot(n.jstates, "/joint_states")

    n.cmds.clear(); n.rstates.clear(); n.jstates.clear()

    print(f"[1] 手动注入命令 joint1 = {TEST['joint1-a1_r']}，持续 2 秒 ...")
    t0 = time.time()
    while time.time() - t0 < 2.0:
        n.send()
        rclpy.spin_once(n, timeout_sec=0.02)

    print("[2] 观察 1.5 秒 ...")
    n.spin(1.5)

    print("  结果：")
    a = Diag.snapshot(n.cmds, "/robot_joint_commands")
    b = Diag.snapshot(n.rstates, "/robot_joint_states")
    c = Diag.snapshot(n.jstates, "/joint_states")

    print()
    print("=" * 58)
    print("  判读：")
    print(f"    [段1] 命令话题        : {'有数据 ✓' if a is not None else '无数据 ✗'}")
    print(f"    [段2] 桥接发出状态    : {'有数据 ✓' if b is not None else '无数据 ✗'}")
    print(f"    [段3] /joint_states   : {'有数据 ✓' if c is not None else '无数据 ✗'}")
    if b is not None and abs(b - TEST["joint1-a1_r"]) < 0.05:
        print(f"    [桥接] 状态已跟随命令 ({b:+.4f}) ✓")
    elif b is not None:
        print(f"    [桥接] 状态未跟随命令 ({b:+.4f}) ✗")
    if c is not None and abs(c - TEST["joint1-a1_r"]) < 0.05:
        print(f"    [硬件接口 read()] 状态已回传 ({c:+.4f}) ✓")
    elif c is not None:
        print(f"    [硬件接口 read()] 状态未回传 ({c:+.4f}) ✗ ← 问题在这里")
    print("=" * 58)

    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
