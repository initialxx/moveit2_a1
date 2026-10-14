"""Onero A1 话题桥适配节点。

职责（只做这一件事）：
    把 ros2_control 的 ``joint_state_topic_hardware_interface`` 接到 armsdk。

数据流::

    joint_trajectory_controller
        │  /robot_joint_commands   (sensor_msgs/JointState)   ← 订阅
        ▼
    本节点 ──── 200Hz 控制循环 ────▶ armsdk CanBackend.command()  ──▶ CAN ──▶ 电机
        │
        │  /robot_joint_states     (sensor_msgs/JointState)   → 发布
        ▼
    joint_state_topic_hardware_interface  →  controller_manager

设计边界（有意为之）：
  * **不做轨迹处理** —— 插值、容差判断、取消、反馈全部由
    ``joint_trajectory_controller`` 负责。本节点只接收逐周期目标点。
  * **自己拥有控制循环** —— 以固定频率向硬件发命令，与 ros2_control 的
    update_rate 解耦。
  * **关节按名字映射**，不假设话题里的顺序。
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

DEFAULT_JOINT_NAMES = [
    "joint1-a1_r", "joint2-a1_r", "joint3-a1_r", "joint4-a1_r",
    "joint5-a1_r", "joint6-a1_r", "joint7-a1_r",
]


class OneroTopicBridge(Node):
    """把 JointState 话题桥接到 armsdk 的适配节点。"""

    def __init__(self) -> None:
        super().__init__("onero_topic_bridge")

        # ─── 参数 ────────────────────────────────────────────────────
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("joint_commands_topic", "/robot_joint_commands")
        self.declare_parameter("joint_states_topic", "/robot_joint_states")
        self.declare_parameter("control_rate", 200.0)
        self.declare_parameter("state_publish_rate", 100.0)

        # simulate=true 时不连硬件，用一阶跟随模拟机械臂（用于无真机联调）
        self.declare_parameter("simulate", True)
        self.declare_parameter("sim_time_constant", 0.02)

        # 真机参数（simulate=false 时生效）
        self.declare_parameter("can_channel", "can0")
        self.declare_parameter("model", "A1R")
        self.declare_parameter("kp", 50.0)
        self.declare_parameter("kd", 1.0)

        # CAN 健康与故障处置
        #   log     —— 仅记录（默认，最保守）
        #   hold    —— 冻结目标，保持当前位置
        #   disable —— 断使能电机（⚠️ 带载时会掉落，仅在机械臂有支撑时使用）
        self.declare_parameter("on_fault_action", "log")
        self.declare_parameter("max_consecutive_misses", 10)

        gp = self.get_parameter
        self._joint_names: list[str] = list(gp("joint_names").value)
        self._n = len(self._joint_names)
        self._idx = {n: i for i, n in enumerate(self._joint_names)}

        self._control_rate = float(gp("control_rate").value)
        self._control_dt = 1.0 / self._control_rate
        self._publish_every = max(
            1, int(round(self._control_rate / float(gp("state_publish_rate").value)))
        )

        self._simulate = bool(gp("simulate").value)
        self._sim_tau = float(gp("sim_time_constant").value)

        # 故障处置策略 —— 在启动时就校验，避免真机跑到一半才发现配置写错
        self._on_fault_action = str(gp("on_fault_action").value).lower()
        self._max_misses = int(gp("max_consecutive_misses").value)
        if self._on_fault_action not in ("log", "hold", "disable"):
            raise RuntimeError(
                f"on_fault_action 取值非法: '{self._on_fault_action}'"
                "（可选 log / hold / disable）"
            )

        # ─── 状态（由控制线程写，锁内交换）────────────────────────────
        self._lock = threading.Lock()
        self._cmd_pos = [0.0] * self._n          # 最新目标位置
        self._state_pos = [0.0] * self._n        # 当前关节位置
        self._state_vel = [0.0] * self._n        # 当前关节速度
        self._last_cmd_stamp: Optional[float] = None   # 收到命令的单调时刻

        # CAN 健康计数（仅真机模式会增长）
        self._can_misses = 0
        self._can_faults = 0
        self._hold = False          # on_fault_action=hold 时置位，冻结目标

        # ─── 硬件后端 ────────────────────────────────────────────────
        self._backend = None
        if not self._simulate:
            self._init_hardware()
        else:
            self.get_logger().warn(
                "simulate=true —— 不连接硬件，用一阶跟随模拟机械臂。"
                "接真机请设置 simulate:=false"
            )

        # ─── ROS 接口 ────────────────────────────────────────────────
        # 命令用 reliable + depth 1：永远只关心最新目标，不积压
        cmd_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        # 状态用 reliable：
        #   硬件插件用 SensorDataQoS (best_effort) 订阅 —— reliable 发布者可以兼容；
        #   若发 best_effort，则所有默认 reliable 的订阅者（ros2 topic echo 等）
        #   都会收不到消息。reliable 是严格更优的选择。
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._cmd_sub = self.create_subscription(
            JointState,
            gp("joint_commands_topic").value,
            self._on_command,
            cmd_qos,
        )
        self._state_pub = self.create_publisher(
            JointState, gp("joint_states_topic").value, state_qos
        )

        # ─── 控制线程 ────────────────────────────────────────────────
        self._running = True
        self._thread = threading.Thread(target=self._control_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"话题桥已启动 | 关节 {self._n} 个 | 控制 {self._control_rate:.0f}Hz "
            f"| 状态发布 {gp('state_publish_rate').value}Hz | "
            f"simulate={self._simulate}"
        )
        self.get_logger().info(
            f"  订阅 {gp('joint_commands_topic').value}"
            f"  发布 {gp('joint_states_topic').value}"
        )

    # ─── 硬件初始化 ──────────────────────────────────────────────────

    def _init_hardware(self) -> None:
        """连接 armsdk 的 CAN 后端。失败则抛异常，让节点启动失败（不静默降级）。"""
        try:
            from onero_sdk.can_backend import CanBackend
            from onero_sdk.model_registry import get_model
        except ImportError as e:
            raise RuntimeError(
                "无法导入 onero_sdk。请先安装：pip install -e /path/to/armsdk"
            ) from e

        gp = self.get_parameter
        model_name = gp("model").value
        channel = gp("can_channel").value

        model = get_model(model_name)
        if model.num_joints != self._n:
            raise RuntimeError(
                f"模型 {model_name} 有 {model.num_joints} 个关节，"
                f"但 joint_names 参数给了 {self._n} 个"
            )
        self._model = model
        self._kp = float(gp("kp").value)
        self._kd = float(gp("kd").value)

        # SDK 提供 on_miss / on_fault 回调，用于感知 CAN 链路健康
        self._backend = CanBackend(
            channel=channel,
            joint_specs=model.joints,
            recv_timeout=0.003,
            on_miss=self._on_can_miss,
            on_fault=self._on_can_fault,
        )
        state = self._backend.enable()
        self._state_pos = list(state.joints)
        self._cmd_pos = list(state.joints)
        self.get_logger().info(
            f"已连接 CAN '{channel}' | 模型 {model_name} | 电机已使能 | "
            f"故障处置={self._on_fault_action}"
        )

    # ─── CAN 健康回调（仅真机模式）────────────────────────────────────

    def _on_can_miss(self, joint_index: int, consecutive: int) -> None:
        """CAN 帧丢失。偶尔丢帧正常，连续丢帧说明链路有问题。"""
        with self._lock:
            self._can_misses += 1
        if consecutive == self._max_misses:
            self.get_logger().error(
                f"关节 {joint_index + 1} 连续丢帧 {consecutive} 次 —— CAN 链路可能异常"
            )
        elif consecutive < self._max_misses:
            self.get_logger().debug(
                f"关节 {joint_index + 1} 丢帧（连续 {consecutive} 次）"
            )

    def _on_can_fault(self, joint_index: int, error_code: int) -> None:
        """电机上报硬件故障。这是需要立即处置的事件。"""
        with self._lock:
            self._can_faults += 1
        self.get_logger().error(
            f"关节 {joint_index + 1} 上报故障码 0x{int(error_code):X}"
            f"（累计 {self._can_faults} 次）| 处置={self._on_fault_action}"
        )

        if self._on_fault_action == "hold":
            # 冻结目标：把目标锁到当前实际位置，后续命令一律忽略
            with self._lock:
                self._hold = True
                self._cmd_pos = list(self._state_pos)
            self.get_logger().warn("已进入保持模式，忽略后续轨迹命令")
        elif self._on_fault_action == "disable":
            self.get_logger().warn(
                "正在断使能电机 —— ⚠️ 若机械臂带载且无支撑，可能会掉落"
            )
            try:
                self._backend.disable()
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f"断使能失败: {e}")

    # ─── 命令回调 ────────────────────────────────────────────────────

    def _on_command(self, msg: JointState) -> None:
        """接收逐周期目标点。按名字映射关节，不假设顺序。

        ⚠️ 必须过滤非有限值（NaN / inf）：
        ros2_control 对【没有 initial_value 的接口】会用 NaN 初始化
        （见 hardware_interface/handle.hpp），因此控制器认领命令接口之前，
        话题里可能短暂出现 NaN。若原样转发给电机，后果不可预期。
        """
        if not msg.name:
            self.get_logger().warn("收到无关节名的命令，已忽略", throttle_duration_sec=5.0)
            return

        accepted = 0
        with self._lock:
            if self._hold:
                # 已因硬件故障进入保持模式：忽略一切新目标
                return
            for name, pos in zip(msg.name, msg.position):
                i = self._idx.get(name)
                if i is None:
                    self.get_logger().warn(
                        f"命令中有未知关节 '{name}'，已忽略",
                        throttle_duration_sec=5.0,
                    )
                    continue
                value = float(pos)
                if not math.isfinite(value):
                    self.get_logger().warn(
                        f"命令中关节 '{name}' 的值非有限（{value}），已忽略",
                        throttle_duration_sec=5.0,
                    )
                    continue
                self._cmd_pos[i] = value
                accepted += 1
            if accepted:
                self._last_cmd_stamp = time.monotonic()

    # ─── 控制循环 ────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        cycle = 0
        next_t = time.monotonic()

        while self._running:
            # 1) 取最新目标
            with self._lock:
                cmd = list(self._cmd_pos)

            # 2) 与硬件交互（真机 or 模拟）
            if self._backend is not None:
                try:
                    state = self._backend.command(
                        positions=cmd,
                        velocities=[0.0] * self._n,
                        kps=[self._kp] * self._n,
                        kds=[self._kd] * self._n,
                        torques=[0.0] * self._n,
                    )
                    with self._lock:
                        self._state_pos = list(state.joints)
                        self._state_vel = list(state.velocities)
                except Exception as e:  # noqa: BLE001 — 控制循环不能死
                    self.get_logger().error(
                        f"控制周期失败: {e}", throttle_duration_sec=1.0
                    )
            else:
                self._step_simulation(cmd)

            # 3) 发布状态
            cycle += 1
            if cycle % self._publish_every == 0:
                self._publish_state()

            # 4) 定时（用绝对时刻累加，避免漂移）
            next_t += self._control_dt
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                # 落后了就重置基准，避免雪崩式追赶
                next_t = time.monotonic()

    def _step_simulation(self, cmd: list[float]) -> None:
        """一阶跟随：state += (cmd - state) * dt / tau

        带 NaN 恢复：若状态已是非有限值（例如启动期被 NaN 命令污染过），
        直接重置到目标值，避免 NaN 永久传播。
        """
        alpha = min(1.0, self._control_dt / max(self._sim_tau, 1e-6))
        with self._lock:
            for i in range(self._n):
                target = cmd[i]
                current = self._state_pos[i]

                if not math.isfinite(current):
                    # 从污染中恢复
                    self._state_pos[i] = target if math.isfinite(target) else 0.0
                    self._state_vel[i] = 0.0
                    continue

                delta = (target - current) * alpha
                self._state_vel[i] = delta / self._control_dt
                self._state_pos[i] = current + delta

    def _publish_state(self) -> None:
        with self._lock:
            pos = list(self._state_pos)
            vel = list(self._state_vel)

        # 最后一道防线：绝不发布非有限值。
        # NaN 一旦进入 /robot_joint_states，会污染硬件接口的状态接口，
        # 进而让 JTC 的到位判断失效（它可能误判为"已到位"）。
        for i in range(self._n):
            if not math.isfinite(pos[i]):
                pos[i] = 0.0
            if not math.isfinite(vel[i]):
                vel[i] = 0.0

        msg = JointState()
        # 必须用节点时钟，不能用 time.monotonic —— 否则 use_sim_time 下 TF 会崩
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = pos
        msg.velocity = vel
        self._state_pub.publish(msg)

    # ─── 生命周期 ────────────────────────────────────────────────────

    def destroy_node(self) -> bool:
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._backend is not None:
            # 不自动 disable：带载的机械臂突然卸力会掉落
            try:
                self._backend.close(disable=False)
            except Exception:  # noqa: BLE001
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OneroTopicBridge()
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
