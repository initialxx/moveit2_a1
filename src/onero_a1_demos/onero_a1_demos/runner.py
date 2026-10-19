"""A1 versions of the MoveIt planning, Cartesian and Planning Scene examples.

Default: plan and validate only. --execute explicitly submits the validated
trajectory to MoveIt's ExecuteTrajectory action. All targets use Link7 (flange),
not an uncalibrated gripper TCP. No SDK/CAN connection is opened by this program.
"""

import argparse
from copy import deepcopy
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
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject, Constraints, DisplayTrajectory, JointConstraint,
    MoveItErrorCodes, OrientationConstraint, PlanningScene, PositionConstraint, RobotState, RobotTrajectory,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionFK, GetStateValidity
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from onero_a1_demos.trial_records import BRIDGE_IDENTITY, fingerprint

JOINTS = [f"joint{i}-a1_r" for i in range(1, 8)]
TRIAL_CASES = ("prepare_trial", "replay_trial")


class DemoError(RuntimeError):
    pass


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def pose_errors(actual, target):
    """Pose dictionaries in one frame; quaternion sign does not affect angle."""
    delta = {axis: actual["position"][axis] - target["position"][axis] for axis in "xyz"}
    position_error = math.sqrt(sum(value * value for value in delta.values()))
    a = [actual["orientation"][axis] for axis in "xyzw"]
    b = [target["orientation"][axis] for axis in "xyzw"]
    norm = math.sqrt(sum(v * v for v in a) * sum(v * v for v in b))
    if not math.isfinite(norm) or norm < 1e-12 or not math.isfinite(position_error):
        raise DemoError("FK 返回了无效位姿")
    cosine = min(1.0, max(0.0, abs(sum(x * y for x, y in zip(a, b))) / norm))
    return delta, position_error, 2.0 * math.acos(cosine)


class Demo(Node):
    def __init__(self, args):
        super().__init__("onero_a1_demo_" + uuid.uuid4().hex[:8])
        self.args = args
        self.report = {"case": args.case, "execute_requested": args.execute,
                       "frame": args.frame, "tip_link": "Link7",
                       "velocity_scaling": args.velocity, "acceleration_scaling": args.acceleration,
                       "position_tolerance_m": args.position_tolerance,
                       "orientation_tolerance_rad": args.orientation_tolerance}
        self.latest = None
        self.active_goal = None
        self.scene_id = "onero_demo_" + uuid.uuid4().hex
        self.scene_added = False
        self.service_clients = {}
        self.create_subscription(
            JointState, "/joint_states", self.on_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.planner = ActionClient(self, MoveGroup, "/move_action")
        self.execution_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.display = self.create_publisher(DisplayTrajectory, "/display_planned_path", 1)

    def on_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions and math.isfinite(positions[name]) for name in JOINTS):
            self.latest = msg

    def wait(self, future, timeout=15.0):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise DemoError("ROS 请求超时")
        result = future.result()
        if result is None:
            raise DemoError("ROS 请求未返回结果")
        return result

    def call(self, srv, name, request):
        if name not in self.service_clients:
            self.service_clients[name] = self.create_client(srv, name)
        client = self.service_clients[name]
        if not client.wait_for_service(timeout_sec=10.0):
            raise DemoError(f"服务不可用：{name}")
        return self.wait(client.call_async(request))

    def state(self):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest is not None:
                age = self.get_clock().now().nanoseconds * 1e-9 - seconds(self.latest.header.stamp)
                if -0.1 <= age <= 1.0:
                    return dict(zip(self.latest.name, self.latest.position))
        raise DemoError("没有新鲜、完整的 /joint_states")

    @staticmethod
    def robot_state(q):
        state = RobotState()
        # Keep measured gripper/attached-body state from the current scene.
        state.is_diff = True
        state.joint_state = JointState(name=JOINTS, position=[float(q[n]) for n in JOINTS])
        return state

    def load_model(self):
        limit_names = [f"robot_description_planning.joint_limits.{joint}.{field}"
                       for joint in JOINTS for field in
                       ("has_velocity_limits", "max_velocity", "has_acceleration_limits", "max_acceleration")]
        request = GetParameters.Request(names=["robot_description", "robot_description_semantic", *limit_names])
        response = self.call(GetParameters, "/move_group/get_parameters", request)
        if len(response.values) != len(request.names) or any(v.type != 4 for v in response.values[:2]):
            raise DemoError("move_group 未提供 URDF/SRDF")
        self.model_parameter_values = [message_to_ordereddict(v) for v in response.values]
        self.report["model_fingerprint"] = fingerprint(self.model_parameter_values)
        urdf, srdf = [ET.fromstring(v.string_value) for v in response.values[:2]]
        self.bounds = {}
        for joint in urdf.findall("joint"):
            if joint.get("name") in JOINTS:
                limit = joint.find("limit")
                self.bounds[joint.get("name")] = (float(limit.get("lower")), float(limit.get("upper")))
        if set(self.bounds) != set(JOINTS):
            raise DemoError("本示例要求 A1R 七关节模型")
        if urdf.find("link[@name='Link7']") is None:
            raise DemoError("模型缺少法兰 Link7")
        self.named = {p.get("name"): {j.get("name"): float(j.get("value")) for j in p.findall("joint")}
                      for p in srdf.findall("group_state") if p.get("group") == "arm"}
        links = [link.get("name") for link in urdf.findall("link") if link.find("collision") is not None]
        pairs = {tuple(sorted(pair)) for pair in combinations(links, 2)}
        disabled = {tuple(sorted((e.get("link1"), e.get("link2")))) for e in srdf.findall("disable_collisions")}
        self.report["self_collision_pairs"] = {"total": len(pairs), "disabled": len(pairs & disabled),
                                                "checked": len(pairs - disabled)}
        self.report["hardware_plugins"] = [p.text for p in urdf.findall("ros2_control/hardware/plugin")]
        if not pairs - disabled:
            raise DemoError("全部本体自碰撞对被排除，请重建并重启加载更新后的 SRDF")
        self.report["bridge_settings"] = None
        if "joint_state_topic_hardware_interface/JointStateTopicSystem" in self.report["hardware_plugins"]:
            names = [*BRIDGE_IDENTITY, "kp", "kd", "velocity_feedforward",
                     "velocity_feedforward_scale", "velocity_feedforward_timeout"]
            values = self.call(GetParameters, "/onero_topic_bridge/get_parameters",
                               GetParameters.Request(names=names)).values
            if len(values) != len(names) or any(v.type not in (1, 2, 3, 4, 8) for v in values):
                raise DemoError("无法读取话题桥安装、增益和速度前馈配置；请重建并重启服务")
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
                    raise DemoError("无法读取夹爪活动标定")
                for key, value in zip(("gripper_closed_sdk_opening", "gripper_open_sdk_opening"), endpoints):
                    self.report["bridge_settings"][key] = value.double_value

    def check_bounds(self, q):
        for name in JOINTS:
            lo, hi = self.bounds[name]
            if not math.isfinite(q[name]) or not lo - 1e-6 <= q[name] <= hi + 1e-6:
                raise DemoError(f"目标超出关节限位：{name}={q[name]}，允许 [{lo}, {hi}]")

    def startup_snapshot(self):
        if "joint_state_topic_hardware_interface/JointStateTopicSystem" not in self.report["hardware_plugins"]:
            raise DemoError("return_startup 需要启动 SDK 话题桥；纯假硬件模式没有该启动快照")
        response = self.call(GetParameters, "/onero_topic_bridge/get_parameters",
                             GetParameters.Request(names=["startup_pose"]))
        if len(response.values) != 1 or response.values[0].type != 4:
            raise DemoError(
                "当前话题桥没有保存启动姿态（旧服务版本）。无法追溯本次启动位置；"
                "不会使用当前位置、home 或某次运动起点替代。重启会把届时位置记录为新初始位。")
        snapshot = json.loads(response.values[0].string_value)
        if (not isinstance(snapshot, dict) or snapshot.get("schema_version") != 1
                or not isinstance(snapshot.get("id"), str) or not snapshot["id"]
                or snapshot.get("source") not in ("sdk_startup_feedback", "simulated_startup")):
            raise DemoError("启动姿态快照格式无效")
        q = snapshot.get("joint_positions")
        if (not isinstance(q, dict) or set(q) != set(JOINTS)
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in q.values())):
            raise DemoError("启动姿态必须包含 A1R 七个有限关节位置")
        return snapshot

    def validity(self, q, constraints=None):
        request = GetStateValidity.Request(robot_state=self.robot_state(q), group_name="arm")
        if constraints is not None:
            request.constraints = constraints
        return self.call(GetStateValidity, "/check_state_validity", request)

    def require_valid(self, q, constraints=None):
        self.check_bounds(q)
        result = self.validity(q, constraints)
        if not result.valid:
            contacts = [(c.contact_body_1, c.contact_body_2) for c in result.contacts]
            raise DemoError(f"状态碰撞或违反约束：{contacts}")

    def fk(self, q):
        request = GetPositionFK.Request(robot_state=self.robot_state(q), fk_link_names=["Link7"])
        request.header.frame_id = self.args.frame
        response = self.call(GetPositionFK, "/compute_fk", request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS or len(response.pose_stamped) != 1:
            raise DemoError(f"FK 失败：{response.error_code.val}")
        return response.pose_stamped[0].pose

    @staticmethod
    def joint_constraints(q, tolerance=0.005):
        return Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=q[n], tolerance_above=tolerance,
                            tolerance_below=tolerance, weight=1.0) for n in JOINTS])

    def orientation_constraint(self, pose, tolerance):
        constraint = OrientationConstraint(link_name="Link7", orientation=pose.orientation, weight=1.0,
                                           absolute_x_axis_tolerance=tolerance,
                                           absolute_y_axis_tolerance=tolerance,
                                           absolute_z_axis_tolerance=tolerance)
        constraint.header.frame_id = self.args.frame
        constraint.parameterization = OrientationConstraint.ROTATION_VECTOR
        return constraint

    def pose_constraints(self, pose):
        position = PositionConstraint(link_name="Link7", weight=1.0)
        position.header.frame_id = self.args.frame
        position.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])]
        position.constraint_region.primitive_poses = [deepcopy(pose)]
        return Constraints(position_constraints=[position], orientation_constraints=[
            self.orientation_constraint(pose, 0.02)])

    def action(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=10.0):
            raise DemoError("MoveIt action server 未就绪")
        handle = self.wait(client.send_goal_async(goal))
        if not handle.accepted:
            raise DemoError("MoveIt 拒绝了目标")
        self.active_goal = handle
        result = self.wait(handle.get_result_async(), timeout)
        self.active_goal = None
        if client is self.execution_client:
            self.report["execution_action_result"] = {
                "status": result.status, "error_code": result.result.error_code.val,
            }
        if result.status != 4 or result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise DemoError(f"MoveIt action 失败：status={result.status}, code={result.result.error_code.val}")
        return result.result

    def plan(self, start, goal_constraints, path_constraints=None):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = "arm"
        request.pipeline_id = "ompl"
        request.planner_id = "RRTConnect"
        request.allowed_planning_time = self.args.planning_time
        request.num_planning_attempts = 3
        request.start_state = self.robot_state(start)
        request.goal_constraints = [goal_constraints]
        request.max_velocity_scaling_factor = self.args.velocity
        request.max_acceleration_scaling_factor = self.args.acceleration
        if path_constraints is not None:
            request.path_constraints = path_constraints
        # Always inspect and validate a plan before optional execution.
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        result = self.action(self.planner, goal, self.args.planning_time + 15.0)
        return result.planned_trajectory

    def validate_plan(self, trajectory, constraints=None):
        if trajectory.multi_dof_joint_trajectory.points or trajectory.multi_dof_joint_trajectory.joint_names:
            raise DemoError("A1 单臂示例不接受多自由度关节轨迹")
        joint = trajectory.joint_trajectory
        if len(joint.joint_names) != len(JOINTS) or set(joint.joint_names) != set(JOINTS):
            raise DemoError("规划轨迹的关节名不匹配")
        if len(joint.points) < 2 or seconds(joint.points[-1].time_from_start) <= 0:
            raise DemoError("返回轨迹没有有效运动/时间参数；选择不同于起点的目标")
        previous = -1.0
        for point in joint.points:
            now = seconds(point.time_from_start)
            if now <= previous or now < 0 or len(point.positions) != len(JOINTS):
                raise DemoError("轨迹时间或位置维度无效")
            if any(values and len(values) != len(JOINTS) for values in
                   (point.velocities, point.accelerations, point.effort)):
                raise DemoError("轨迹速度、加速度或力矩维度无效")
            if any(not math.isfinite(v) for values in (point.positions, point.velocities, point.accelerations)
                   for v in values):
                raise DemoError("轨迹包含非有限值")
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
                raise DemoError(f"规划终点未满足末端容差：位置 {distance * 1000:.2f} mm，朝向 {angle:.4f} rad")
        self.display.publish(DisplayTrajectory(model_id="onero_a1", trajectory=[trajectory],
                                               trajectory_start=self.robot_state(self.report["start"])))
        print(f"规划与状态检查通过：{len(joint.points)} 点，时长 {previous:.3f}s", flush=True)

    def execute(self, trajectory):
        if self.args.case == "return_startup":
            if self.startup_snapshot()["id"] != self.report["startup_pose"]["id"]:
                raise DemoError("规划期间话题桥已重启，启动姿态已改变，请重新规划")
        current = self.state()
        joint = trajectory.joint_trajectory
        if max(abs(current[n] - p) for n, p in zip(joint.joint_names, joint.points[0].positions)) > 0.01:
            raise DemoError("规划后起始位置已变化超过 0.01 rad，请重新规划")
        self.require_valid(current)
        self.report["execution_start"] = {n: current[n] for n in JOINTS}
        if "pose_target" in self.report:
            self.report["execution_start_pose"] = message_to_ordereddict(self.fk(current))
        goal = ExecuteTrajectory.Goal(trajectory=trajectory, controllers=["arm_controller"])
        self.action(self.execution_client, goal, max(30.0, 2.0 * self.report["duration_s"] + 5.0))
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
                raise DemoError(
                    "已执行运动且控制器返回成功，但末端到位未通过（依据关节反馈 FK）："
                    f"位置误差 {distance * 1000:.2f} mm，阈值 {self.args.position_tolerance * 1000:.2f} mm；"
                    f"朝向误差 {angle:.4f} rad，阈值 {self.args.orientation_tolerance:.4f} rad。"
                    "请检查跟踪精度，不要连续重复相对运动。")
        maximum = max(abs(v) for v in errors.values())
        if maximum > self.args.goal_tolerance:
            raise DemoError(f"原始目标误差 {maximum:.6f} rad 超过 {self.args.goal_tolerance}")
        if self.args.case == "prepare_trial":
            start_errors = {n: self.report["requested_joint_target"][n] - actual[n] for n in JOINTS}
            self.report["trial_start_error_rad"] = start_errors
            if max(abs(v) for v in start_errors.values()) > 0.01:
                raise DemoError("已执行回测试起点，但关节误差仍超过 0.01 rad；尚不满足复用轨迹的起点要求")
        if self.args.case == "return_startup":
            startup_errors = {n: self.report["startup_pose"]["joint_positions"][n] - actual[n] for n in JOINTS}
            self.report["startup_pose_error_rad"] = startup_errors
            startup_maximum = max(abs(v) for v in startup_errors.values())
            if startup_maximum > self.args.goal_tolerance:
                raise DemoError(f"与启动姿态的误差 {startup_maximum:.6f} rad 超过 {self.args.goal_tolerance}")
        self.report["executed"] = True
        print(f"执行成功；相对原始终点最大关节误差 {maximum:.6f} rad", flush=True)

    def collision_demo(self):
        # This deliberately out-of-bounds folded geometry is a read-only test
        # of the collision detector, NEVER a plan or a hardware motion target.
        safe = dict.fromkeys(JOINTS, 0.0)
        folded = {**safe, "joint4-a1_r": 3.14}
        self.require_valid(safe)
        response = self.validity(folded)
        contacts = [(c.contact_body_1, c.contact_body_2) for c in response.contacts]
        body = {"base_link", *[f"Link{i}" for i in range(1, 8)]}
        self_contacts = [p for p in contacts if p[0] in body and p[1] in body]
        self.report["collision_geometry_query"] = {
            "configuration": folded, "out_of_bounds_test_only": True,
            "valid": response.valid, "self_contacts": self_contacts,
        }
        if response.valid or not self_contacts:
            raise DemoError("未检出折叠几何的自碰撞；检查 SRDF 是否仍屏蔽非相邻连杆")
        print(f"自碰撞查询通过：正常姿态有效，折叠几何返回 {self_contacts}", flush=True)

    def apply_sphere(self, pose=None):
        obj = CollisionObject(id=self.scene_id)
        obj.header.frame_id = self.args.frame
        if pose is None:
            obj.operation = CollisionObject.REMOVE
        else:
            obj.operation = CollisionObject.ADD
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE,
                                             dimensions=[self.args.obstacle_radius])]
            obj.primitive_poses = [pose]
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [obj]
        result = self.call(ApplyPlanningScene, "/apply_planning_scene", ApplyPlanningScene.Request(scene=scene))
        if not result.success:
            raise DemoError("Planning Scene 更新失败")
        self.scene_added = pose is not None

    def scene_demo(self, start):
        target = {**start, "joint4-a1_r": start["joint4-a1_r"] + self.args.scene_delta}
        self.require_valid(target)
        baseline = self.plan(start, self.joint_constraints(target))
        points = baseline.joint_trajectory.points
        selected = None
        for fraction in (0.5, 0.3, 0.7):
            if not points:
                break
            point = points[min(len(points) - 1, int((len(points) - 1) * fraction))]
            middle = dict(zip(baseline.joint_trajectory.joint_names, point.positions))
            pose = self.fk(middle)
            self.apply_sphere(pose)
            if (self.validity(start).valid and self.validity(target).valid
                    and not self.validity(middle).valid):
                selected = pose
                break
        if selected is None:
            raise DemoError("当前姿态无法布置同时避开起终点的中途障碍物；调整 --scene-delta 或 --obstacle-radius")
        self.report["obstacle"] = {"id": self.scene_id, "radius": self.args.obstacle_radius,
                                   "pose": message_to_ordereddict(selected),
                                   "baseline_midpoint_blocked": True}
        replanned = self.plan(start, self.joint_constraints(target))
        self.validate_plan(replanned)
        if self.args.execute:
            self.execute(replanned)
        self.apply_sphere()
        self.require_valid(self.state())
        self.report["obstacle_removed"] = True

    def saved_trial(self, start):
        """Recheck a saved Cartesian line, or plan back to its original start."""
        path = self.args.from_report.expanduser().resolve()
        source = json.loads(path.read_text())
        if (source.get("cartesian_shape") != "line" or not source.get("pose_target")
                or not source.get("trajectory_start_pose") or not source.get("trajectory_sha256")):
            raise DemoError("需要新版保存的笛卡尔直线报告；请先运行 cartesian --cartesian-shape line 仅规划并保存报告")
        if source.get("model_fingerprint") != self.report["model_fingerprint"]:
            raise DemoError("报告的模型或规划限位与当前服务不同，请重新生成测试轨迹")
        old_bridge, new_bridge = source.get("bridge_settings"), self.report["bridge_settings"]
        if (old_bridge is None) != (new_bridge is None):
            raise DemoError("保存轨迹与当前服务的硬件模式不同")
        if old_bridge is not None and any(old_bridge.get(k) != new_bridge.get(k) for k in BRIDGE_IDENTITY):
            raise DemoError("保存轨迹与当前安装方式、负载选项、CAN 或控制频率不同")
        if fingerprint(source.get("trajectory")) != source["trajectory_sha256"]:
            raise DemoError("保存的轨迹内容已改变或损坏，请重新生成报告")
        if source.get("frame") not in ("world", "base_link"):
            raise DemoError("保存轨迹的参考系无效")
        self.args.frame = source["frame"]
        self.report.update(frame=source["frame"], source_report=str(path),
                           source_report_sha256=fingerprint(source),
                           trial_trajectory_sha256=source["trajectory_sha256"])
        trajectory = RobotTrajectory()
        set_message_fields(trajectory, deepcopy(source["trajectory"]))
        joint = trajectory.joint_trajectory
        if (set(joint.joint_names) != set(JOINTS) or len(joint.joint_names) != len(JOINTS)
                or not joint.points or len(joint.points[0].positions) != len(JOINTS)):
            raise DemoError("保存轨迹缺少完整的七关节起点")
        saved_start = dict(zip(joint.joint_names, joint.points[0].positions))
        self.require_valid(saved_start)
        saved_start_pose = message_to_ordereddict(self.fk(saved_start))
        _, frame_distance, frame_angle = pose_errors(saved_start_pose, source["trajectory_start_pose"])
        if frame_distance > 1e-6 or frame_angle > 1e-6:
            raise DemoError("保存轨迹的安装坐标变换与当前不同，请重新生成报告")
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
                raise DemoError("当前位置不符合保存的测试起点；先用 prepare_trial 回到该起点，不能直接重复相对运动")
            for field, attr in (("velocity_scaling", "velocity"), ("acceleration_scaling", "acceleration")):
                value = source.get(field)
                if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 1:
                    raise DemoError(f"报告的 {field} 无效")
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

    def run(self):
        self.load_model()
        start = {n: v for n, v in self.state().items() if n in JOINTS}
        self.report["start"] = start
        self.require_valid(start)
        if self.args.case in TRIAL_CASES:
            self.saved_trial(start)
            return
        if self.args.case == "collision":
            self.collision_demo()
            return
        if self.args.case == "scene":
            self.scene_demo(start)
            return
        constraints = None
        if self.args.case in ("joint", "named", "return_startup"):
            if self.args.case == "return_startup":
                snapshot = self.startup_snapshot()
                self.report["startup_pose"] = snapshot
                target = {n: float(snapshot["joint_positions"][n]) for n in JOINTS}
                print(f"返回目标：服务启动快照 {snapshot['id']}，{target}", flush=True)
            elif self.args.case == "named":
                if self.args.target not in self.named:
                    raise DemoError(f"命名姿态不存在：{self.args.target}，可选 {list(self.named)}")
                target = self.named[self.args.target]
            else:
                target = {**start, self.args.joint: start[self.args.joint] + self.args.delta}
            self.require_valid(target)
            self.report["requested_joint_target"] = target
            trajectory = self.plan(start, self.joint_constraints(target))
        else:
            current = self.fk(start)
            self.report["start_pose"] = message_to_ordereddict(current)
            self.report["requested_translation_xyz_m"] = {
                "x": self.args.dx, "y": self.args.dy, "z": self.args.dz,
            }
            target = deepcopy(current)
            target.position.x += self.args.dx
            target.position.y += self.args.dy
            target.position.z += self.args.dz
            self.report["pose_target"] = message_to_ordereddict(target)
            if self.args.case == "cartesian":
                self.report["cartesian_shape"] = self.args.cartesian_shape
                waypoints = [target]
                if self.args.cartesian_shape == "square":
                    size = self.args.square_size
                    for dx, dy in ((size, 0.0), (size, size), (0.0, size), (0.0, 0.0)):
                        corner = deepcopy(target)
                        corner.position.x += dx
                        corner.position.y += dy
                        waypoints.append(corner)
                self.report["cartesian_waypoints"] = [message_to_ordereddict(p) for p in waypoints]
                request = GetCartesianPath.Request(
                    start_state=self.robot_state(start), group_name="arm", link_name="Link7",
                    waypoints=waypoints, max_step=0.003, revolute_jump_threshold=0.2,
                    avoid_collisions=True, max_velocity_scaling_factor=self.args.velocity,
                    max_acceleration_scaling_factor=self.args.acceleration,
                )
                request.header.frame_id = self.args.frame
                response = self.call(GetCartesianPath, "/compute_cartesian_path", request)
                self.report["cartesian_fraction"] = response.fraction
                if response.error_code.val != MoveItErrorCodes.SUCCESS or response.fraction < 1.0 - 1e-6:
                    raise DemoError(f"笛卡尔路径不完整：fraction={response.fraction}，code={response.error_code.val}")
                trajectory = response.solution
            else:
                if self.args.case == "constraints":
                    constraints = Constraints(orientation_constraints=[self.orientation_constraint(current, 0.1)])
                    self.report["path_constraints"] = message_to_ordereddict(constraints)
                trajectory = self.plan(start, self.pose_constraints(target), constraints)
        self.validate_plan(trajectory, constraints)
        if self.args.execute:
            self.execute(trajectory)

    def cleanup(self):
        ok = True
        if self.active_goal is not None:
            try:
                result = self.wait(self.active_goal.cancel_goal_async(), 3.0)
                self.report["cancel_return_code"] = result.return_code
            except Exception as exc:
                ok = False
                self.report["cancel_error"] = str(exc)
                print(f"取消请求失败：{exc}", file=sys.stderr)
        if self.scene_added:
            try:
                self.apply_sphere()
            except Exception as exc:
                ok = False
                self.report["scene_cleanup_error"] = str(exc)
                print(f"清理场景失败（对象 {self.scene_id}）：{exc}", file=sys.stderr)
        return ok


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=["joint", "named", "pose", "cartesian", "constraints", "scene", "collision", "return_startup", *TRIAL_CASES])
    parser.add_argument("--execute", action="store_true", help="显式执行轨迹；默认仅规划，不驱动机械臂")
    parser.add_argument("--joint", choices=JOINTS, default=JOINTS[0])
    parser.add_argument("--delta", type=float, default=0.1, help="关节目标相对当前状态的变化，rad")
    parser.add_argument("--target", default="ready", help="named 示例使用的 SRDF 姿态")
    parser.add_argument("--frame", choices=["base_link", "world"], default=None)
    parser.add_argument("--from-report", type=Path, help="prepare_trial / replay_trial 使用的已保存直线报告")
    parser.add_argument("--dx", type=float, default=0.01)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dz", type=float, default=-0.01)
    parser.add_argument("--cartesian-shape", choices=["line", "square"], default="line")
    parser.add_argument("--square-size", type=float, default=0.01, help="小矩形边长，m")
    parser.add_argument("--velocity", type=float, default=None, help="速度缩放，默认 0.05；replay_trial 使用原轨迹时间")
    parser.add_argument("--acceleration", type=float, default=None, help="加速度缩放，默认 0.05；replay_trial 使用原轨迹时间")
    parser.add_argument("--planning-time", type=float, default=10.0)
    parser.add_argument("--goal-tolerance", type=float, default=0.05, help="执行后与原始终点比较的关节容差，rad")
    parser.add_argument("--position-tolerance", type=float, default=0.005,
                        help="pose/cartesian/constraints 的末端位置验收容差，m；依据关节反馈 FK，默认 5 mm")
    parser.add_argument("--orientation-tolerance", type=float, default=0.05,
                        help="pose/cartesian/constraints 的末端朝向验收容差，rad；执行后检查，不是实时保护")
    parser.add_argument("--scene-delta", type=float, default=0.4, help="场景示例的 joint4 相对变化，rad")
    parser.add_argument("--obstacle-radius", type=float, default=0.01, help="场景球形障碍物半径，m")
    parser.add_argument("--output", type=Path, help="结果 JSON 文件；默认 .logs/demos 下")
    options, ros_args = parser.parse_known_args(args)
    if options.case in TRIAL_CASES:
        if options.from_report is None:
            parser.error("prepare_trial / replay_trial 必须指定 --from-report")
        if options.frame is not None:
            parser.error("prepare_trial / replay_trial 使用报告中的参考系，不接受 --frame")
        if options.case == "replay_trial" and (options.velocity is not None or options.acceleration is not None):
            parser.error("replay_trial 保留原轨迹速度和时间；更改速度需要重新生成报告")
        supplied = list(args) if args is not None else sys.argv[1:]
        target_flags = ("--joint", "--delta", "--target", "--dx", "--dy", "--dz",
                        "--cartesian-shape", "--square-size", "--scene-delta", "--obstacle-radius")
        if any(token == flag or token.startswith(flag + "=") for token in supplied for flag in target_flags):
            parser.error("固定试验使用报告中的目标，不接受改变目标或路径形状的参数")
    elif options.from_report is not None:
        parser.error("--from-report 仅适用于 prepare_trial / replay_trial")
    options.frame = options.frame or "base_link"
    options.velocity = 0.05 if options.velocity is None else options.velocity
    options.acceleration = 0.05 if options.acceleration is None else options.acceleration
    for name in ("velocity", "acceleration"):
        if not math.isfinite(getattr(options, name)) or not 0 < getattr(options, name) <= 1:
            parser.error(f"{name} 必须在 (0,1] 内")
    for name in ("delta", "dx", "dy", "dz", "scene_delta", "planning_time", "goal_tolerance", "position_tolerance", "orientation_tolerance", "obstacle_radius", "square_size"):
        if not math.isfinite(getattr(options, name)):
            parser.error(f"{name} 必须有限")
    if min(options.planning_time, options.goal_tolerance, options.position_tolerance,
           options.orientation_tolerance, options.obstacle_radius, options.square_size) <= 0:
        parser.error("规划时间、容差和障碍物半径必须为正")
    if options.case == "collision" and options.execute:
        parser.error("collision 示例只做只读几何查询，禁止执行反例姿态")
    output = options.output or Path(".logs/demos") / f"{datetime.now():%Y%m%d-%H%M%S}-{options.case}-{uuid.uuid4().hex[:6]}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        parser.error(f"输出文件已存在：{output}")
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Demo(options)
    code = 0
    try:
        node.run()
        node.report["status"] = "executed" if node.report.get("executed") else "planning_checks_passed"
    except (Exception, KeyboardInterrupt) as exc:
        code = 1
        node.report.update(status="failed", error=str(exc) or "interrupted")
        print(f"示例失败：{node.report['error']}", file=sys.stderr)
    finally:
        if not node.cleanup():
            code = 1
            node.report["status"] = "failed"
        output.write_text(json.dumps(node.report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        print(f"结果：{node.report['status']}；报告：{output.resolve()}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
