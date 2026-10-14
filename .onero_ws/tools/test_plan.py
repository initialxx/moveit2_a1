#!/usr/bin/env python3
"""
验证 MoveIt 规划链路是否可用。

向运行中的 move_group 发送一个关节空间规划请求，检查是否返回有效轨迹。
用法：
    python3 test_plan.py [group_name] [joint_name] [target_position]
默认：
    group_name = panda_arm, joint = panda_joint1, target = 0.5
"""

import sys

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState


def main() -> int:
    group = sys.argv[1] if len(sys.argv) > 1 else "panda_arm"
    joint = sys.argv[2] if len(sys.argv) > 2 else "panda_joint1"
    target = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5

    rclpy.init()
    node = rclpy.create_node("plan_link_test")

    cli = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    node.get_logger().info("等待 /plan_kinematic_path 服务...")
    if not cli.wait_for_service(timeout_sec=30.0):
        node.get_logger().error("服务不可用 —— move_group 没起来？")
        return 1
    node.get_logger().info("服务已就绪")

    # 等一个 /joint_states，确保起始状态已知
    got_state = []
    sub = node.create_subscription(
        JointState, "/joint_states", lambda m: got_state.append(m), 10
    )
    node.get_logger().info("等待 /joint_states ...")
    import time

    t0 = time.time()
    while not got_state and time.time() - t0 < 15.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not got_state:
        node.get_logger().error("/joint_states 无数据 —— joint_state_broadcaster 没起来？")
        return 1
    node.get_logger().info(
        f"收到关节状态: {len(got_state[-1].name)} 个关节, "
        f"首个={got_state[-1].name[0] if got_state[-1].name else '?'}"
    )
    del sub

    req = GetMotionPlan.Request()
    mpr = req.motion_plan_request
    mpr.group_name = group
    mpr.num_planning_attempts = 5
    mpr.allowed_planning_time = 5.0

    jc = JointConstraint()
    jc.joint_name = joint
    jc.position = target
    jc.tolerance_above = 0.01
    jc.tolerance_below = 0.01
    jc.weight = 1.0

    c = Constraints()
    c.joint_constraints.append(jc)
    mpr.goal_constraints.append(c)

    node.get_logger().info(f"规划请求: group={group} {joint} -> {target}")
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=40.0)

    if not fut.done() or fut.result() is None:
        node.get_logger().error("规划请求超时或无响应")
        return 1

    res = fut.result().motion_plan_response
    code = res.error_code.val
    traj = res.trajectory.joint_trajectory
    n = len(traj.points)

    print()
    print("=" * 56)
    print(f"  error_code     : {code}  ({'成功' if code == 1 else '失败'})")
    if n:
        t = traj.points[-1].time_from_start
        print(f"  轨迹点数       : {n}")
        print(f"  关节           : {list(traj.joint_names)}")
        print(f"  总时长         : {t.sec + t.nanosec * 1e-9:.3f} s")
    print("=" * 56)

    rclpy.shutdown()
    return 0 if (code == 1 and n > 0) else 1


if __name__ == "__main__":
    sys.exit(main())
