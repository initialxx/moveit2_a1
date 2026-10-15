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


class WaitForJointStates(Node):
    """Exit only after the state broadcaster reflects all live joint positions."""

    def __init__(self) -> None:
        super().__init__("wait_for_valid_joint_states")
        self.declare_parameter("joints", DEFAULT_JOINTS)
        self.declare_parameter("topic", "/joint_states")
        self._joints = list(self.get_parameter("joints").value)
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
        positions = {
            name: float(msg.position[i])
            for i, name in enumerate(msg.name)
            if i < len(msg.position)
        }
        if not all(name in positions and math.isfinite(positions[name]) for name in self._joints):
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
