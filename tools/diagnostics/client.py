"""Strict model/trajectory/precision diagnostics layered on the common client."""
from copy import deepcopy
import argparse
from datetime import datetime
from itertools import combinations
import json
import math
from pathlib import Path
import sys
import time
import uuid
import xml.etree.ElementTree as ET

import rclpy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import GetParameters
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from onero_a1_common.constants import JOINTS
from onero_a1_common.moveit_client import MoveItClient, MoveItError, seconds
from .trial_records import BRIDGE_IDENTITY, fingerprint

DiagnosticError = MoveItError


def diagnostic_options(parser, argv=None):
    parser.add_argument("--execute", action="store_true", help="显式执行运动；默认只读/规划")
    parser.add_argument("--frame", choices=["world", "base_link"], default="base_link")
    parser.add_argument("--velocity", type=float, default=.05)
    parser.add_argument("--acceleration", type=float, default=.05)
    parser.add_argument("--planning-time", type=float, default=10.)
    parser.add_argument("--goal-tolerance", type=float, default=.05)
    parser.add_argument("--position-tolerance", type=float, default=.005)
    parser.add_argument("--orientation-tolerance", type=float, default=.05)
    parser.add_argument("--output", type=Path, help="诊断报告，默认 .logs/diagnostics/")
    args, args_ros = parser.parse_known_args(argv)
    for name in ("velocity", "acceleration"):
        if not math.isfinite(getattr(args, name)) or not 0 < getattr(args, name) <= 1:
            parser.error(f"{name} 必须在 (0, 1] 内")
    for name in ("planning_time", "goal_tolerance", "position_tolerance", "orientation_tolerance"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"{name} 必须为有限正数")
    args.ros_args = args_ros
    return args

def pose_errors(actual, target):
    """Pose dictionaries in one frame; quaternion sign does not affect angle."""
    delta = {axis: actual["position"][axis] - target["position"][axis] for axis in "xyz"}
    position_error = math.sqrt(sum(value * value for value in delta.values()))
    a = [actual["orientation"][axis] for axis in "xyzw"]
    b = [target["orientation"][axis] for axis in "xyzw"]
    norm = math.sqrt(sum(v * v for v in a) * sum(v * v for v in b))
    if not math.isfinite(norm) or norm < 1e-12 or not math.isfinite(position_error):
        raise DiagnosticError("FK 返回了无效位姿")
    cosine = min(1.0, max(0.0, abs(sum(x * y for x, y in zip(a, b))) / norm))
    return delta, position_error, 2.0 * math.acos(cosine)

class DiagnosticClient(MoveItClient):
    def __init__(self, args):
        super().__init__(args)
        self.report = {"case": args.case, "execute_requested": args.execute,
                       "frame": args.frame, "tip_link": "Link7",
                       "velocity_scaling": args.velocity, "acceleration_scaling": args.acceleration,
                       "position_tolerance_m": args.position_tolerance,
                       "orientation_tolerance_rad": args.orientation_tolerance}

    def on_action_result(self, client, response):
        key = "execution_action_result" if client is self.execution_client else "planning_action_result"
        self.report[key] = {"status": response.status, "error_code": response.result.error_code.val}

    def load_model(self):
        limit_names = [f"robot_description_planning.joint_limits.{joint}.{field}"
                       for joint in JOINTS for field in
                       ("has_velocity_limits", "max_velocity", "has_acceleration_limits", "max_acceleration")]
        request = GetParameters.Request(names=["robot_description", "robot_description_semantic", *limit_names])
        response = self.call(GetParameters, "/move_group/get_parameters", request)
        if len(response.values) != len(request.names) or any(v.type != 4 for v in response.values[:2]):
            raise DiagnosticError("move_group 未提供 URDF/SRDF")
        self.model_parameter_values = [message_to_ordereddict(v) for v in response.values]
        self.report["model_fingerprint"] = fingerprint(self.model_parameter_values)
        urdf, srdf = [ET.fromstring(v.string_value) for v in response.values[:2]]
        self.bounds = {}
        for joint in urdf.findall("joint"):
            if joint.get("name") in JOINTS:
                limit = joint.find("limit")
                self.bounds[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
        if set(self.bounds) != set(JOINTS):
            raise DiagnosticError("本示例要求 A1R 七关节模型")
        if urdf.find("link[@name='Link7']") is None:
            raise DiagnosticError("模型缺少法兰 Link7")
        self.named = {p.get("name"): {j.get("name"): float(j.get("value")) for j in p.findall("joint")}
                      for p in srdf.findall("group_state") if p.get("group") == "arm"}
        links = [link.get("name") for link in urdf.findall("link") if link.find("collision") is not None]
        pairs = {tuple(sorted(pair)) for pair in combinations(links, 2)}
        disabled = {tuple(sorted((e.get("link1"), e.get("link2")))) for e in srdf.findall("disable_collisions")}
        self.report["self_collision_pairs"] = {"total": len(pairs), "disabled": len(pairs & disabled),
                                                "checked": len(pairs - disabled)}
        self.report["hardware_plugins"] = [p.text for p in urdf.findall("ros2_control/hardware/plugin")]
        if not pairs - disabled:
            raise DiagnosticError("全部本体自碰撞对被排除，请重建并重启加载更新后的 SRDF")
        self.report["bridge_settings"] = None
        if "joint_state_topic_hardware_interface/JointStateTopicSystem" in self.report["hardware_plugins"]:
            names = [*BRIDGE_IDENTITY, "kp", "kd", "velocity_feedforward",
                     "velocity_feedforward_scale", "velocity_feedforward_timeout"]
            values = self.call(GetParameters, "/onero_topic_bridge/get_parameters",
                               GetParameters.Request(names=names)).values
            if len(values) != len(names) or any(v.type not in (1, 2, 3, 4, 8) for v in values):
                raise DiagnosticError("无法读取话题桥安装、增益和速度前馈配置；请重建并重启服务")
            attributes = {1: "bool_value", 2: "integer_value", 3: "double_value", 4: "string_value"}
            self.report["bridge_settings"] = {
                name: (list(value.double_array_value) if value.type == 8
                       else getattr(value, attributes[value.type]))
                for name, value in zip(names, values)
            }
            if self.report["bridge_settings"].get("gripper_model") == "woan_picker":
                # Auto-calibration updates the gripper controller's active endpoints.
                endpoints = self.call(GetParameters, "/gripper_controller/get_parameters",
                                      GetParameters.Request(names=["closed_sdk_opening", "open_sdk_opening"])).values
                if len(endpoints) != 2 or any(v.type != 3 for v in endpoints):
                    raise DiagnosticError("无法读取夹爪活动标定")
                for key, value in zip(("gripper_closed_sdk_opening", "gripper_open_sdk_opening"), endpoints):
                    self.report["bridge_settings"][key] = value.double_value

    def check_bounds(self, q):
        for name in JOINTS:
            lo, hi = self.bounds[name]
            if not math.isfinite(q[name]) or not lo - 1e-6 <= q[name] <= hi + 1e-6:
                raise DiagnosticError(f"目标超出关节限位：{name}={q[name]}，允许 [{lo}, {hi}]")

    def require_valid(self, q, constraints=None):
        self.check_bounds(q)
        result = self.validity(q, constraints)
        if not result.valid:
            contacts = [(c.contact_body_1, c.contact_body_2) for c in result.contacts]
            raise DiagnosticError(f"状态碰撞或违反约束：{contacts}")

    def validate_plan(self, trajectory, constraints=None):
        if trajectory.multi_dof_joint_trajectory.points or trajectory.multi_dof_joint_trajectory.joint_names:
            raise DiagnosticError("A1 单臂示例不接受多自由度关节轨迹")
        joint = trajectory.joint_trajectory
        if len(joint.joint_names) != len(JOINTS) or set(joint.joint_names) != set(JOINTS):
            raise DiagnosticError("规划轨迹的关节名不匹配")
        if len(joint.points) < 2 or seconds(joint.points[-1].time_from_start) <= 0:
            raise DiagnosticError("返回轨迹没有有效运动/时间参数；选择不同于起点的目标")
        previous = -1.0
        for point in joint.points:
            now = seconds(point.time_from_start)
            if now <= previous or now < 0 or len(point.positions) != len(JOINTS):
                raise DiagnosticError("轨迹时间或位置维度无效")
            if any(values and len(values) != len(JOINTS) for values in
                   (point.velocities, point.accelerations, point.effort)):
                raise DiagnosticError("轨迹速度、加速度或力矩维度无效")
            if any(not math.isfinite(v) for values in (point.positions, point.velocities, point.accelerations)
                   for v in values):
                raise DiagnosticError("轨迹包含非有限值")
            previous = now
            self.require_valid(dict(zip(joint.joint_names, point.positions)), constraints)
        self.report["trajectory"] = message_to_ordereddict(trajectory)
        self.report["trajectory_sha256"] = fingerprint(self.report["trajectory"])
        self.report["planned_endpoint"] = dict(zip(joint.joint_names, joint.points[-1].positions))
        self.report["validated_waypoints"] = len(joint.points)
        self.report["duration_s"] = previous
        if "pose_target" in self.report:
            q0 = dict(zip(joint.joint_names, joint.points[0].positions))
            self.report["trajectory_start_pose"] = message_to_ordereddict(self.fk(q0))
            pose = message_to_ordereddict(self.fk(self.report["planned_endpoint"]))
            delta, distance, angle = pose_errors(pose, self.report["pose_target"])
            self.report.update(planned_pose=pose, planned_position_error_xyz_m=delta,
                               planned_position_error_m=distance, planned_orientation_error_rad=angle)
            if distance > self.args.position_tolerance or angle > self.args.orientation_tolerance:
                raise DiagnosticError(f"规划终点未满足末端容差：位置 {distance * 1000:.2f} mm，朝向 {angle:.4f} rad")
        self.display.publish(DisplayTrajectory(model_id="onero_a1", trajectory=[trajectory],
                                               trajectory_start=self.robot_state(self.report["start"])))
        print(f"规划与状态检查通过：{len(joint.points)} 点，时长 {previous:.3f}s", flush=True)

    def saved_trial(self, start):
        """Recheck a saved Cartesian line, or plan back to its original start."""
        path = self.args.from_report.expanduser().resolve()
        source = json.loads(path.read_text())
        if (source.get("cartesian_shape") != "line" or not source.get("pose_target")
                or not source.get("trajectory_start_pose") or not source.get("trajectory_sha256")):
            raise DiagnosticError("需要新版保存的笛卡尔直线报告；请先运行 cartesian --cartesian-shape line 仅规划并保存报告")
        if source.get("model_fingerprint") != self.report["model_fingerprint"]:
            raise DiagnosticError("报告的模型或规划限位与当前服务不同，请重新生成测试轨迹")
        old_bridge, new_bridge = source.get("bridge_settings"), self.report["bridge_settings"]
        if (old_bridge is None) != (new_bridge is None):
            raise DiagnosticError("保存轨迹与当前服务的硬件模式不同")
        if old_bridge is not None and any(old_bridge.get(k) != new_bridge.get(k) for k in BRIDGE_IDENTITY):
            raise DiagnosticError("保存轨迹与当前安装方式、负载选项、CAN 或控制频率不同")
        if fingerprint(source.get("trajectory")) != source["trajectory_sha256"]:
            raise DiagnosticError("保存的轨迹内容已改变或损坏，请重新生成报告")
        if source.get("frame") not in ("world", "base_link"):
            raise DiagnosticError("保存轨迹的参考系无效")
        self.args.frame = source["frame"]
        self.report.update(frame=source["frame"], source_report=str(path),
                           source_report_sha256=fingerprint(source),
                           trial_trajectory_sha256=source["trajectory_sha256"])
        trajectory = RobotTrajectory()
        set_message_fields(trajectory, deepcopy(source["trajectory"]))
        joint = trajectory.joint_trajectory
        if (set(joint.joint_names) != set(JOINTS) or len(joint.joint_names) != len(JOINTS)
                or not joint.points or len(joint.points[0].positions) != len(JOINTS)):
            raise DiagnosticError("保存轨迹缺少完整的七关节起点")
        saved_start = dict(zip(joint.joint_names, joint.points[0].positions))
        self.require_valid(saved_start)
        saved_start_pose = message_to_ordereddict(self.fk(saved_start))
        _, frame_distance, frame_angle = pose_errors(saved_start_pose, source["trajectory_start_pose"])
        if frame_distance > 1e-6 or frame_angle > 1e-6:
            raise DiagnosticError("保存轨迹的安装坐标变换与当前不同，请重新生成报告")
        self.report["saved_trial_start"] = saved_start
        self.report["start_pose"] = message_to_ordereddict(self.fk(start))
        self.report["trial_start_difference_rad"] = {n: start[n] - saved_start[n] for n in JOINTS}
        if self.args.case == "prepare_trial":
            self.report["requested_joint_target"] = saved_start
            self.report["pose_target"] = saved_start_pose
            if max(abs(start[n] - saved_start[n]) for n in JOINTS) < 1e-6:
                self.report["already_at_trial_start"] = True
                print("当前位置已经是保存的测试起点，无需运动。", flush=True)
                return
            trajectory = self.plan(start, self.joint_constraints(saved_start, tolerance=0.001))
        else:
            _, distance, angle = pose_errors(self.report["start_pose"], saved_start_pose)
            if (max(abs(start[n] - saved_start[n]) for n in JOINTS) > 0.01
                    or distance > self.args.position_tolerance or angle > self.args.orientation_tolerance):
                raise DiagnosticError("当前位置不符合保存的测试起点；先用 prepare_trial 回到该起点，不能直接重复相对运动")
            for field, attr in (("velocity_scaling", "velocity"), ("acceleration_scaling", "acceleration")):
                value = source.get(field)
                if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 1:
                    raise DiagnosticError(f"报告的 {field} 无效")
                self.report[field] = value
                setattr(self.args, attr, value)
            self.report["pose_target"] = deepcopy(source["pose_target"])
            self.report["cartesian_shape"] = "line"
            # Preserve all joint samples and time_from_start. Start the saved
            # trajectory now instead of reusing an old absolute timestamp.
            trajectory.joint_trajectory.header.stamp.sec = 0
            trajectory.joint_trajectory.header.stamp.nanosec = 0
            trajectory.multi_dof_joint_trajectory.header.stamp.sec = 0
            trajectory.multi_dof_joint_trajectory.header.stamp.nanosec = 0
        self.validate_plan(trajectory)
        if self.args.execute:
            self.execute(trajectory)

    def execute(self, trajectory):
        current = self.state()
        self.require_valid(current)
        self.report["execution_start"] = {n: current[n] for n in JOINTS}
        if "pose_target" in self.report:
            self.report["execution_start_pose"] = message_to_ordereddict(self.fk(current))
        super().execute(trajectory)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        actual = self.state()
        errors = {n: self.report["planned_endpoint"][n] - actual[n] for n in JOINTS}
        self.report.update(actual_endpoint=actual, original_goal_error_rad=errors)
        if "pose_target" in self.report:
            measured = message_to_ordereddict(self.fk(actual))
            delta, distance, angle = pose_errors(measured, self.report["pose_target"])
            self.report.update(actual_pose=measured, fk_position_error_xyz_m=delta,
                               fk_position_error_m=distance, fk_orientation_error_rad=angle)
            self.report["actual_translation_xyz_m"] = {
                axis: measured["position"][axis] - self.report["execution_start_pose"]["position"][axis]
                for axis in "xyz"
            }
            # Controller success only certifies its joint tolerances. A 0.03 rad
            # joint error can exceed an entire requested 1 cm Cartesian move.
            # This is an endpoint acceptance check, not a motion-time guard.
            if distance > self.args.position_tolerance or angle > self.args.orientation_tolerance:
                raise DiagnosticError(
                    "已执行运动且控制器返回成功，但末端到位未通过（依据关节反馈 FK）："
                    f"位置误差 {distance * 1000:.2f} mm，阈值 {self.args.position_tolerance * 1000:.2f} mm；"
                    f"朝向误差 {angle:.4f} rad，阈值 {self.args.orientation_tolerance:.4f} rad。"
                    "请检查跟踪精度，不要连续重复相对运动。")
        maximum = max(abs(v) for v in errors.values())
        if maximum > self.args.goal_tolerance:
            raise DiagnosticError(f"原始目标误差 {maximum:.6f} rad 超过 {self.args.goal_tolerance}")
        if self.args.case == "prepare_trial":
            start_errors = {n: self.report["requested_joint_target"][n] - actual[n] for n in JOINTS}
            self.report["trial_start_error_rad"] = start_errors
            if max(abs(v) for v in start_errors.values()) > 0.01:
                raise DiagnosticError("已执行回测试起点，但关节误差仍超过 0.01 rad；尚不满足复用轨迹的起点要求")
        self.report["executed"] = True
        print(f"执行成功；相对原始终点最大关节误差 {maximum:.6f} rad", flush=True)

def run_diagnostic(options, scenario):
    """Own internal reporting and strict acceptance; examples never call this."""
    output = options.output or Path(".logs/diagnostics") / f"{datetime.now():%Y%m%d-%H%M%S}-{options.case}-{uuid.uuid4().hex[:6]}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise ValueError(f"报告已存在：{output}")
    rclpy.init(args=options.ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    code = 0
    try:
        node = DiagnosticClient(options)
        node.load_model()
        node.report["start"] = {n: q for n, q in node.state().items() if n in JOINTS}
        node.require_valid(node.report["start"])
        scenario(node)
        node.report["status"] = "executed" if node.report.get("executed") else "planning_checks_passed"
    except (Exception, KeyboardInterrupt) as exc:
        code = 1
        if node is not None:
            node.report.update(status="failed", error=str(exc) or "interrupted")
        print(f"诊断失败：{str(exc) or 'interrupted'}", file=sys.stderr)
    finally:
        if node is not None:
            if not node.cleanup():
                code = 1
                node.report["status"] = "failed"
            with output.open("x", encoding="utf-8") as stream:
                json.dump(node.report, stream, ensure_ascii=False, indent=2, allow_nan=False)
                stream.write("\n")
            print(f"结果：{node.report['status']}；报告：{output.resolve()}")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
