#!/usr/bin/env python3
"""最终验收：经 move_group 完成「规划 → 执行」端到端。

与 test_execution.py 的区别：
  test_execution.py 直接给 arm_controller 发轨迹（只验证执行层）
  本脚本走 /move_action，验证包含【规划】在内的完整链路：

    MoveGroup action
      -> move_group (OMPL 规划)
      -> arm_controller (JTC)
      -> joint_state_topic_hardware_interface
      -> /robot_joint_commands -> onero_topic_bridge -> 机械臂
      -> /robot_joint_states -> 回到硬件接口
      -> /joint_states

用法：
    python3 test_moveit_e2e.py [关节名] [目标角]
默认 joint1-a1_r -> 0.4
"""

import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory  # noqa: F401  (类型检查用)
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from sensor_msgs.msg import JointState

JOINTS = [
    "joint1-a1_r", "joint2-a1_r", "joint3-a1_r", "joint4-a1_r",
    "joint5-a1_r", "joint6-a1_r", "joint7-a1_r",
]


class E2E(Node):
    def __init__(self):
        super().__init__("moveit_e2e")
        self.states = []
        self.create_subscription(JointState, "/joint_states", lambda m: self.states.append(m), 10)
        self.client = ActionClient(self, MoveGroup, "/move_action")

    def spin_for(self, sec):
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def pos(self):
        if not self.states:
            return {}
        m = self.states[-1]
        return {n: p for n, p in zip(m.name, m.position)}


def main():
    joint = sys.argv[1] if len(sys.argv) > 1 else "joint1-a1_r"
    target = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4

    rclpy.init()
    node = E2E()

    print("[1] 等待 /move_action（move_group）")
    if not node.client.wait_for_server(timeout_sec=45.0):
        print("  ✗ move_group 未就绪")
        rclpy.shutdown()
        return 1
    print("  ✓ 就绪")

    print("[2] 等待 /joint_states")
    t0 = time.time()
    while not node.states and time.time() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    before = node.pos()
    if not before:
        print("  ✗ 无关节状态")
        rclpy.shutdown()
        return 1
    print(f"  ✓ 起始 {joint} = {before.get(joint, float('nan')):+.4f}")

    # 构造关节空间目标
    jc = JointConstraint()
    jc.joint_name = joint
    jc.position = target
    jc.tolerance_above = 0.02
    jc.tolerance_below = 0.02
    jc.weight = 1.0
    cons = Constraints()
    cons.joint_constraints.append(jc)

    goal = MoveGroup.Goal()
    goal.request.group_name = "arm"
    goal.request.num_planning_attempts = 5
    goal.request.allowed_planning_time = 5.0
    goal.request.goal_constraints.append(cons)
    goal.planning_options.plan_only = False        # 规划并执行
    goal.planning_options.replan = False
    goal.planning_options.look_around = False

    print(f"[3] 发送 MoveGroup 目标：{joint} -> {target}")
    fut = node.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print("  ✗ goal 被拒绝")
        rclpy.shutdown()
        return 1
    print("  ✓ goal 已接受（规划中…）")

    res_fut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res_fut, timeout_sec=60.0)
    if not res_fut.done() or res_fut.result() is None:
        print("  ✗ 执行超时")
        rclpy.shutdown()
        return 1

    r = res_fut.result()
    code = r.result.error_code.val
    status = r.status
    print(f"  error_code = {code}   action status = {status}")
    if r.result.planned_trajectory.joint_trajectory.points:
        pts = len(r.result.planned_trajectory.joint_trajectory.points)
        print(f"  规划轨迹点数 = {pts}")

    print("[4] 校验最终位置")
    node.spin_for(1.5)
    after = node.pos()
    got = after.get(joint, float("nan"))
    err = abs(got - target)
    # 注意：MoveGroup.Result 没有 SUCCESS 常量，成功码在 moveit_msgs/MoveItErrorCodes
    ok = (code == MoveItErrorCodes.SUCCESS) and err < 0.05

    print(f"    {joint}: 起始 {before.get(joint, 0):+.4f} -> 目标 {target:+.4f} -> 实际 {got:+.4f}  (误差 {err:.4f})")

    print()
    print("=" * 58)
    print(f"  结果: {'MoveIt 规划→执行 端到端通过 ✓' if ok else '失败 ✗'}")
    print("=" * 58)

    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
