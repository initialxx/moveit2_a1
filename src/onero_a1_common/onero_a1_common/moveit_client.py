"""Small standard MoveIt ROS client, shared by applications and internal tools.

This module handles ROS requests, fresh state, visualization and execution.
Reporting, pose archives, model fingerprints and precision acceptance belong
to the diagnostic tools, which can extend this client.
"""
from copy import deepcopy
import json
import math
import time
import uuid
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    CollisionObject, Constraints, DisplayTrajectory, JointConstraint,
    MoveItErrorCodes, OrientationConstraint, PlanningScene, PositionConstraint, RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionFK, GetStateValidity
from shape_msgs.msg import SolidPrimitive
from .constants import JOINTS


class MoveItError(RuntimeError):
    pass


def seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


class MoveItClient(Node):
    def __init__(self, args):
        super().__init__("onero_a1_client_" + uuid.uuid4().hex[:8])
        self.args = args
        self.latest = None
        self.active_goal = None
        self.scene_id = "onero_example_" + uuid.uuid4().hex
        self.scene_added = False
        self.service_clients = {}
        self.planned_start_state = None
        self.startup_id = None
        self.create_subscription(
            JointState, "/joint_states", self.on_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.planner = ActionClient(self, MoveGroup, "/move_action")
        self.execution_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.display = self.create_publisher(DisplayTrajectory, "/display_planned_path", 1)

    def on_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(n in positions and math.isfinite(positions[n]) for n in JOINTS):
            self.latest = msg

    def wait(self, future, timeout=15.):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=.05)
        if not future.done():
            raise MoveItError("ROS 请求超时")
        result = future.result()
        if result is None:
            raise MoveItError("ROS 请求未返回结果")
        return result

    def call(self, srv, name, request):
        if name not in self.service_clients:
            self.service_clients[name] = self.create_client(srv, name)
        client = self.service_clients[name]
        if not client.wait_for_service(timeout_sec=10.):
            raise MoveItError(f"服务不可用：{name}")
        return self.wait(client.call_async(request))

    def state(self):
        deadline = time.monotonic()+10.
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if self.latest is not None:
                age = self.get_clock().now().nanoseconds*1e-9-seconds(self.latest.header.stamp)
                if -.1 <= age <= 1.:
                    return dict(zip(self.latest.name, self.latest.position))
        raise MoveItError("没有新鲜、完整的 /joint_states")

    @staticmethod
    def robot_state(q):
        # Preserve the gripper and attached-object state in MoveIt's scene.
        return RobotState(is_diff=True, joint_state=JointState(
            name=list(q), position=[float(v) for v in q.values()]))

    def parameter(self, node, name):
        result = self.call(GetParameters, node+"/get_parameters", GetParameters.Request(names=[name]))
        if len(result.values) != 1 or result.values[0].type != 4:
            raise MoveItError(f"无法读取 {node} 的字符串参数 {name}")
        return result.values[0].string_value

    def named_target(self, name):
        srdf = ET.fromstring(self.parameter("/move_group", "robot_description_semantic"))
        states = {s.get("name"): s for s in srdf.findall("group_state") if s.get("group") == "arm"}
        if name not in states:
            raise MoveItError(f"未知姿态 {name}；可选：{list(states)}")
        return {j.get("name"): float(j.get("value")) for j in states[name].findall("joint")}

    def startup_snapshot(self):
        snapshot = json.loads(self.parameter("/onero_topic_bridge", "startup_pose"))
        if not isinstance(snapshot, dict):
            raise MoveItError("启动姿态格式无效")
        q = snapshot.get("joint_positions")
        if (snapshot.get("schema_version") != 1 or not snapshot.get("id")
                or not isinstance(q, dict) or set(q) != set(JOINTS)
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in q.values())):
            raise MoveItError("启动姿态必须包含七个有限关节角度和有效快照 ID")
        return snapshot

    def fk(self, q):
        request = GetPositionFK.Request(robot_state=self.robot_state(q), fk_link_names=["Link7"])
        request.header.frame_id = self.args.frame
        response = self.call(GetPositionFK, "/compute_fk", request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS or len(response.pose_stamped) != 1:
            raise MoveItError(f"FK 失败：{response.error_code.val}")
        return response.pose_stamped[0].pose

    def validity(self, q, constraints=None, group="arm"):
        request = GetStateValidity.Request(robot_state=self.robot_state(q), group_name=group)
        if constraints is not None:
            request.constraints = constraints
        return self.call(GetStateValidity, "/check_state_validity", request)

    @staticmethod
    def joint_constraints(q, tolerance=.005):
        return Constraints(joint_constraints=[JointConstraint(
            joint_name=n, position=float(v), tolerance_above=tolerance,
            tolerance_below=tolerance, weight=1.) for n, v in q.items()])

    def orientation_constraint(self, pose, tolerance):
        constraint = OrientationConstraint(
            link_name="Link7", orientation=pose.orientation, weight=1.,
            absolute_x_axis_tolerance=tolerance, absolute_y_axis_tolerance=tolerance,
            absolute_z_axis_tolerance=tolerance, parameterization=OrientationConstraint.ROTATION_VECTOR)
        constraint.header.frame_id = self.args.frame
        return constraint

    def pose_constraints(self, pose):
        position = PositionConstraint(link_name="Link7", weight=1.)
        position.header.frame_id = self.args.frame
        position.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.002])]
        position.constraint_region.primitive_poses = [deepcopy(pose)]
        return Constraints(position_constraints=[position], orientation_constraints=[
            self.orientation_constraint(pose, .02)])

    def action(self, client, goal, timeout):
        if not client.wait_for_server(timeout_sec=10.):
            raise MoveItError("MoveIt Action 未就绪")
        handle = self.wait(client.send_goal_async(goal))
        if not handle.accepted:
            raise MoveItError("MoveIt 拒绝了目标")
        self.active_goal = handle
        response = self.wait(handle.get_result_async(), timeout)
        self.active_goal = None
        self.on_action_result(client, response)
        code = response.result.error_code
        if response.status != 4 or code.val != MoveItErrorCodes.SUCCESS:
            phase = "执行" if client is self.execution_client else "规划"
            message = f"；{code.message}" if code.message else ""
            raise MoveItError(f"MoveIt {phase}失败：status={response.status}, code={code.val}{message}")
        return response.result

    def on_action_result(self, client, response):
        """Optional application hook for consuming a completed Action result."""

    def cartesian_path(self, start, waypoints):
        request = GetCartesianPath.Request(
            start_state=self.robot_state(start), group_name="arm", link_name="Link7",
            waypoints=waypoints, max_step=.003, revolute_jump_threshold=.2, avoid_collisions=True,
            max_velocity_scaling_factor=self.args.velocity,
            max_acceleration_scaling_factor=self.args.acceleration)
        request.header.frame_id = self.args.frame
        return self.call(GetCartesianPath, "/compute_cartesian_path", request)

    def plan(self, start, goal_constraints, path_constraints=None, group="arm"):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name, request.pipeline_id, request.planner_id = group, "ompl", "RRTConnect"
        request.start_state = self.robot_state(start)
        request.goal_constraints = [goal_constraints]
        request.allowed_planning_time = self.args.planning_time
        request.num_planning_attempts = 3
        request.max_velocity_scaling_factor = self.args.velocity
        request.max_acceleration_scaling_factor = self.args.acceleration
        if path_constraints is not None:
            request.path_constraints = path_constraints
        goal.planning_options.plan_only = True
        result = self.action(self.planner, goal, self.args.planning_time+15.)
        self.planned_start_state = result.trajectory_start
        return result.planned_trajectory

    def show_plan(self, trajectory):
        points = trajectory.joint_trajectory.points
        if not points:
            raise MoveItError("MoveIt 未返回轨迹")
        self.display.publish(DisplayTrajectory(
            model_id="onero_a1", trajectory=[trajectory],
            trajectory_start=self.planned_start_state or self.robot_state(self.start)))
        print(f"规划成功：{len(points)} 点，时长 {seconds(points[-1].time_from_start):.3f}s")

    def execute(self, trajectory, controller="arm_controller"):
        points = trajectory.joint_trajectory.points
        if not points or seconds(points[-1].time_from_start) <= 0:
            print("目标与当前状态重合，无需执行运动。")
            return
        if self.startup_id is not None and self.startup_snapshot()["id"] != self.startup_id:
            raise MoveItError("规划期间服务已重启，启动姿态发生变化，请重新运行")
        current = self.state()
        names = trajectory.joint_trajectory.joint_names
        if any(n in current and abs(current[n]-v) > .01 for n, v in zip(names, points[0].positions)):
            raise MoveItError("规划后起始状态已变化，请重新规划")
        goal = ExecuteTrajectory.Goal(trajectory=trajectory, controllers=[controller])
        self.action(self.execution_client, goal, max(30., 2.*seconds(points[-1].time_from_start)+5.))
        print("MoveIt 执行成功。")

    def cleanup(self):
        ok = True
        if self.active_goal is not None:
            try:
                self.wait(self.active_goal.cancel_goal_async(), 3.)
            except Exception as exc:
                self.get_logger().error(f"取消请求失败：{exc}")
                ok = False
        if self.scene_added:
            try:
                obj = CollisionObject(id=self.scene_id, operation=CollisionObject.REMOVE)
                obj.header.frame_id = self.args.frame
                scene = PlanningScene(is_diff=True)
                scene.robot_state.is_diff = True
                scene.world.collision_objects = [obj]
                if not self.call(ApplyPlanningScene, "/apply_planning_scene", ApplyPlanningScene.Request(scene=scene)).success:
                    raise MoveItError("MoveIt 拒绝删除临时对象")
                self.scene_added = False
            except Exception as exc:
                self.get_logger().error(f"清理对象 {self.scene_id} 失败：{exc}")
                ok = False
        return ok
