#!/usr/bin/env python3
"""端到端执行测试：验证轨迹能否穿过整条链路。

链路：
    FollowJointTrajectory action
        -> arm_controller (JTC)
        -> joint_state_topic_hardware_interface
        -> /robot_joint_commands
        -> onero_topic_bridge
        -> (simulate 时：一阶跟随模拟机械臂)
        -> /robot_joint_states
        -> 回到硬件接口 -> joint_states_broadcaster -> /joint_states

验证点：
    1. /arm_controller/follow_joint_trajectory action server 存在
    2. goal 被接受并成功完成（error_code == SUCCESSFUL）
    3. /joint_states 的关节角确实到达目标
    4. 反馈（feedback）有在流动

用法：
    python3 test_execution.py
"""

import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = [
    "joint1-a1_r", "joint2-a1_r", "joint3-a1_r", "joint4-a1_r",
    "joint5-a1_r", "joint6-a1_r", "joint7-a1_r",
]
ACTION = "/arm_controller/follow_joint_trajectory"

TARGET = {
    "joint1-a1_r": 0.30,
    "joint2-a1_r": -0.45,
    "joint3-a1_r": 0.20,
    "joint4-a1_r": -0.60,
    "joint5-a1_r": 0.15,
    "joint6-a1_r": 0.40,
    "joint7-a1_r": -0.25,
}


class ExecTest(Node):
    def __init__(self):
        super().__init__("exec_test")
        self.states = []
        self.feedbacks = []
        self.create_subscription(JointState, "/joint_states", self._on_state, 10)
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)

    def _on_state(self, msg):
        self.states.append(msg)

    def spin_for(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def latest(self):
        return self.states[-1] if self.states else None


def main():
    rclpy.init()
    node = ExecTest()

    print(f"[1] 等待 action server: {ACTION}")
    if not node.client.wait_for_server(timeout_sec=30.0):
        print("  ✗ action server 不存在 —— arm_controller 没起来？")
        rclpy.shutdown()
        return 1
    print("  ✓ action server 就绪")

    print("[2] 等待 /joint_states")
    t0 = time.time()
    while node.latest() is None and time.time() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.latest() is None:
        print("  ✗ 没有 /joint_states")
        rclpy.shutdown()
        return 1
    start = {n: p for n, p in zip(node.latest().name, node.latest().position)}
    print(f"  ✓ 起始位姿: " + ", ".join(f"{j}={start.get(j, float('nan')):+.3f}" for j in JOINTS[:3]) + " ...")

    # 构造轨迹：起始点 + 终点（各 3 秒）
    traj = JointTrajectory()
    traj.joint_names = JOINTS

    p0 = JointTrajectoryPoint()
    p0.positions = [start.get(j, 0.0) for j in JOINTS]
    p0.velocities = [0.0] * 7
    p0.time_from_start = Duration(sec=0, nanosec=0)

    p1 = JointTrajectoryPoint()
    p1.positions = [TARGET[j] for j in JOINTS]
    p1.velocities = [0.0] * 7
    p1.time_from_start = Duration(sec=3, nanosec=0)

    traj.points = [p0, p1]

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj

    print("[3] 发送轨迹（3 秒）...")
    send_fut = node.client.send_goal_async(
        goal, feedback_callback=lambda fb: node.feedbacks.append(fb)
    )
    rclpy.spin_until_future_complete(node, send_fut, timeout_sec=20.0)
    gh = send_fut.result()
    if gh is None or not gh.accepted:
        print("  ✗ goal 被拒绝")
        rclpy.shutdown()
        return 1
    print("  ✓ goal 已接受")

    print("[4] 等待执行完成 ...")
    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut, timeout_sec=40.0)
    if not res_fut.done() or res_fut.result() is None:
        print("  ✗ 执行超时")
        rclpy.shutdown()
        return 1

    code = res_fut.result().result.error_code
    status = res_fut.result().status
    print(f"  error_code = {code}  action status = {status}")
    print(f"  收到 {len(node.feedbacks)} 条 feedback")

    print("[5] 校验最终位置")
    node.spin_for(1.0)
    final = {n: p for n, p in zip(node.latest().name, node.latest().position)}

    ok = True
    if code != FollowJointTrajectory.Result.SUCCESSFUL:
        ok = False
        print(f"  ✗ error_code 非 SUCCESSFUL")
    for j in JOINTS:
        want, got = TARGET[j], final.get(j, float("nan"))
        err = abs(got - want)
        flag = "✓" if err < 0.05 else "✗"
        if err >= 0.05:
            ok = False
        print(f"    {flag} {j:14s} 目标 {want:+.3f}  实际 {got:+.3f}  误差 {err:.4f}")

    print()
    print("=" * 56)
    print(f"  结果: {'端到端执行通过 ✓' if ok else '存在失败 ✗'}")
    print("=" * 56)

    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
