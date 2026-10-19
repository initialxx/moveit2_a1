"""Wait until ros2_control publishes a complete, finite joint state."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


DEFAULT_JOINTS = [
    "joint1-a1_r", "joint2-a1_r", "joint3-a1_r", "joint4-a1_r",
    "joint5-a1_r", "joint6-a1_r", "joint7-a1_r",
]


class JointStateCache:
    """Merge standard partial JointState publishers without refreshing old joints."""
    def __init__(self, names):
        self.names = list(names)
        self.values = {}

    def update(self, msg, now_s):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not -.1 <= now_s-stamp <= 1.:
            return None
        for name, value in zip(msg.name, msg.position):
            if name in self.names:
                if not math.isfinite(value):
                    self.values.pop(name, None)
                    continue
                previous = self.values.get(name)
                if previous is None or stamp >= previous[1]:
                    self.values[name] = (float(value), stamp)
        if not all(n in self.values and -.1 <= now_s-self.values[n][1] <= 1. for n in self.names):
            return None
        return {n:self.values[n][0] for n in self.names}


class WaitForJointStates(Node):
    """Exit only after the state broadcaster reflects all live joint positions."""

    def __init__(self) -> None:
        super().__init__("wait_for_valid_joint_states")
        self.declare_parameter("joints", DEFAULT_JOINTS)
        self.declare_parameter("topic", "/joint_states")
        self.declare_parameter("gripper_model", "none")
        self._joints = list(self.get_parameter("joints").value)
        if self.get_parameter("gripper_model").value == "woan_picker":
            self._joints.append("woan_picker_left_finger_joint")
        self._cache = JointStateCache(self._joints)
        self._topic = str(self.get_parameter("topic").value)
        self._valid_messages = 0
        self.ready = False
        self.create_subscription(
            JointState, self._topic, self._on_state, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"等待 {self._topic} 提供 {len(self._joints)} 个有限关节位置，"
            "状态有效前 arm_controller 保持未启动"
        )

    def _on_state(self, msg: JointState) -> None:
        positions = self._cache.update(msg, self.get_clock().now().nanoseconds*1e-9)
        if positions is None:
            self._valid_messages = 0
            return

        self._valid_messages += 1
        if self._valid_messages < 3:
            return
        self.ready = True
        self.get_logger().info(f"已收到 {self._topic} 的连续有效关节状态")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaitForJointStates()
    ready = False
    try:
        while rclpy.ok() and not node.ready:
            rclpy.spin_once(node, timeout_sec=0.5)
        ready = node.ready
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not ready:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
