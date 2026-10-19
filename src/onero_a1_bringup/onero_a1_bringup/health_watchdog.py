"""话题桥健康看门狗。

存在的理由
----------
``joint_state_topic_hardware_interface`` 的 ``read()`` 只是把**最近收到的**
状态抄进状态接口，**没有任何时间戳校验**（源码可证）：

    joint_states_[POSITION_INTERFACE_INDEX][j] = latest_joint_state_.position[i];

后果：如果话题桥节点崩溃或卡死，硬件接口会**永远返回最后一帧状态**，
MoveIt 会认为"机械臂静止、一切正常"，继续规划并下发轨迹。

本节点从外部监控状态话题的新鲜度，失联时**停用轨迹控制器**，
让 MoveIt 无法再下达新轨迹。

注意
----
本节点只负责"让 MoveIt 停下来"。它**不能**代替机械臂本身的硬件急停。
真正的安全链应包含：驱动器过流保护、物理急停按钮、以及机械限位。
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from controller_manager_msgs.srv import SwitchController
from sensor_msgs.msg import JointState


class BridgeWatchdog(Node):
    """监控话题桥状态新鲜度，失联时停用控制器。"""

    STATE_OK = "ok"
    STATE_STALE = "stale"
    STATE_RECOVERED = "recovered"

    def __init__(self) -> None:
        super().__init__("onero_bridge_watchdog")

        self.declare_parameter("joint_states_topic", "/robot_joint_states")
        self.declare_parameter("controller_name", "arm_controller")
        self.declare_parameter("controller_manager_ns", "/controller_manager")
        self.declare_parameter("timeout", 1.0)
        self.declare_parameter("check_rate", 10.0)
        self.declare_parameter("auto_deactivate", True)
        self.declare_parameter("auto_reactivate", True)

        gp = self.get_parameter
        self._topic = gp("joint_states_topic").value
        self._controller = gp("controller_name").value
        self._cm_ns = gp("controller_manager_ns").value.rstrip("/")
        self._timeout = float(gp("timeout").value)
        self._auto_deactivate = bool(gp("auto_deactivate").value)
        self._auto_reactivate = bool(gp("auto_reactivate").value)
        check_dt = 1.0 / max(float(gp("check_rate").value), 0.1)

        self._last_msg_time: float | None = None
        self._msg_count = 0
        self._status = self.STATE_OK
        self._deactivated_by_us = False
        self._switch_pending = False

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(JointState, self._topic, self._on_state, qos)
        self._switch_cli = self.create_client(
            SwitchController, f"{self._cm_ns}/switch_controller"
        )

        self._timer = self.create_timer(check_dt, self._check)

        self.get_logger().info(
            f"看门狗已启动 | 监控 {self._topic} | 超时 {self._timeout:.1f}s "
            f"| 控制器 {self._controller} | 自动停用={self._auto_deactivate}"
        )

    # ─── 回调 ────────────────────────────────────────────────────────

    def _on_state(self, _msg: JointState) -> None:
        self._last_msg_time = time.monotonic()
        self._msg_count += 1
        if self._status == self.STATE_STALE:
            self._status = self.STATE_RECOVERED
            self.get_logger().warn("状态已恢复")
            self._sync_controller()

    # ─── 周期检查 ────────────────────────────────────────────────────

    def _check(self) -> None:
        if self._last_msg_time is None:
            # 启动后还没收到过任何状态。
            #
            # 必须判断 _status：否则会以 check_rate（默认 10Hz）反复刷屏，
            # 把终端里真正有用的启动错误（例如 topic_bridge 的崩溃原因）淹没掉。
            if (self._msg_count == 0
                    and self._status != self.STATE_STALE
                    and self._elapsed_since_start() > self._timeout):
                self._handle_stale("启动后从未收到状态", never_received=True)
            return

        age = time.monotonic() - self._last_msg_time
        if age > self._timeout:
            if self._status != self.STATE_STALE:
                self._handle_stale(f"状态已陈旧 {age:.2f}s（阈值 {self._timeout:.2f}s）")

    def _start_time(self) -> float:
        if not hasattr(self, "_t0"):
            self._t0 = time.monotonic()
        return self._t0

    def _elapsed_since_start(self) -> float:
        return time.monotonic() - self._start_time()

    def _handle_stale(self, reason: str, never_received: bool = False) -> None:
        self._status = self.STATE_STALE
        if never_received:
            # 和「曾经有、后来断了」是两回事，排查方向完全不同：
            # 这里一帧都没来过，硬件接口读到的是 initial_value（0.0），不是最后一帧。
            # 最常见的原因就是 topic_bridge 自己没起来。
            self.get_logger().error(
                "话题桥从未发布过状态。硬件接口读到的会是 initial_value（0.0），"
                "不是真实位置。\n"
                "        最可能的原因：topic_bridge 节点启动失败。"
                "请查看 launch 输出中 topic_bridge 的报错 —— 常见的是 "
                "onero_sdk 未安装（source setup_env.sh，或 pip install onero-sdk）、"
                "CAN 接口不存在或未 UP。"
            )
        else:
            self.get_logger().error(
                f"话题桥失联：{reason}。"
                "硬件接口会持续返回最后一帧状态，MoveIt 可能误判为正常。"
            )
        if self._auto_deactivate and not self._deactivated_by_us:
            self._switch(activate=False)

    # ─── 控制器切换 ──────────────────────────────────────────────────

    def _sync_controller(self) -> None:
        """Reconcile a completed async switch with the latest feedback state."""
        if self._switch_pending:
            return
        fresh = (self._last_msg_time is not None
                 and time.monotonic() - self._last_msg_time <= self._timeout)
        if fresh:
            if self._auto_reactivate and self._deactivated_by_us:
                self._switch(activate=True)
        elif self._auto_deactivate and not self._deactivated_by_us:
            self._switch(activate=False)

    def _switch(self, activate: bool) -> None:
        if self._switch_pending:
            return
        if not self._switch_cli.service_is_ready():
            if not self._switch_cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().error(
                    f"{self._cm_ns}/switch_controller 不可用，无法{'激活' if activate else '停用'}控制器"
                )
                return

        req = SwitchController.Request()
        if activate:
            req.activate_controllers = [self._controller]
            req.deactivate_controllers = []
        else:
            req.activate_controllers = []
            req.deactivate_controllers = [self._controller]
        # BEST_EFFORT=1：不因资源冲突而拒绝
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True

        self._switch_pending = True
        try:
            future = self._switch_cli.call_async(req)
        except Exception as e:  # noqa: BLE001
            self._switch_pending = False
            self.get_logger().error(f"控制器切换调用失败: {e}")
            return
        future.add_done_callback(
            lambda f: self._on_switch_done(f, activate)
        )

    def _on_switch_done(self, future, activate: bool) -> None:
        self._switch_pending = False
        try:
            ok = future.result().ok
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"控制器切换调用失败: {e}")
            return

        if ok:
            self._deactivated_by_us = not activate
            self.get_logger().warn(
                f"{'已重新激活' if activate else '已停用'}控制器 '{self._controller}'"
            )
            # Feedback can recover before the deactivation response arrives.
            # Conversely, it can become stale while activation is in flight.
            # Check freshness again after recording the completed switch.
            self._sync_controller()
        else:
            self.get_logger().error(
                f"{'激活' if activate else '停用'}控制器 '{self._controller}' 被拒绝"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BridgeWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
