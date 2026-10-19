"""Onero A1 bringup —— 启动 ros2_control 与话题桥适配层。

三种运行模式统一使用 hardware_mode：
    fake（默认）：mock_components 纯 MoveIt 模拟，不启动话题桥。
    sim：启动话题桥，用模拟反馈联调，不连接 CAN。
    real：启动话题桥，通过 SDK 控制真机 CAN。
旧 use_fake_hardware / simulate 命令仍兼容，与显式模式冲突时拒绝启动。

注意：本 launch 只负责【执行层】（ros2_control + 桥接）。
     MoveIt 的 move_group / RViz 由 onero_a1_moveit_config 包启动。

     ros2 launch onero_a1_moveit_config moveit.launch.py hardware_mode:=sim
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from onero_a1_bringup.hardware_mode import configure_hardware_mode, mode_default


def generate_launch_description():
    bringup_share = get_package_share_directory("onero_a1_bringup")
    desc_share = get_package_share_directory("onero_a1_description")

    xacro_file = os.path.join(desc_share, "urdf", "onero_a1.urdf.xacro")
    controllers_file = os.path.join(bringup_share, "config", "ros2_controllers.yaml")

    # ─── 参数 ────────────────────────────────────────────────────────
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    simulate = LaunchConfiguration("simulate")
    joint_commands_topic = LaunchConfiguration("joint_commands_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    args = [
        OpaqueFunction(function=configure_hardware_mode),
        DeclareLaunchArgument("hardware_mode", default_value="fake", choices=["fake", "sim", "real"],
                             description="fake: 纯模拟；sim: 话题桥模拟；real: 真机 CAN"),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="旧命令兼容参数；新命令使用 hardware_mode",
        ),
        DeclareLaunchArgument(
            "simulate",
            default_value="true",
            description="旧命令兼容参数；新命令使用 hardware_mode",
        ),
        DeclareLaunchArgument("joint_commands_topic", default_value="/robot_joint_commands"),
        DeclareLaunchArgument("joint_states_topic", default_value="/robot_joint_states"),
        DeclareLaunchArgument("can_channel", default_value="can0"),
        DeclareLaunchArgument("model", default_value="A1R"),
        DeclareLaunchArgument("kp", default_value=mode_default("kp"),
                             description="默认 50（模拟不使用增益）"),
        DeclareLaunchArgument("kd", default_value="1.0"),
        DeclareLaunchArgument("velocity_feedforward", default_value=mode_default("velocity_feedforward"),
                             description="全部七关节速度前馈；默认 real=true、fake/sim=false；可显式覆盖"),
        DeclareLaunchArgument("velocity_feedforward_scale", default_value="[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]",
                             description="兼容参数，仅接受全 1；使用 velocity_feedforward 统一开关"),
        DeclareLaunchArgument("velocity_feedforward_timeout", default_value="0.1"),
        DeclareLaunchArgument("mounting", default_value="upright"),
        DeclareLaunchArgument("with_gripper", default_value="false"),
        DeclareLaunchArgument("gripper_model", default_value="woan_picker", choices=["none", "woan_picker"]),
        DeclareLaunchArgument("allow_gripper_execution", default_value="true"),
        DeclareLaunchArgument("gripper_closed_sdk_opening", default_value="0.0"),
        DeclareLaunchArgument("gripper_open_sdk_opening", default_value="1.0"),
        DeclareLaunchArgument("gripper_calibration_file", default_value=""),
        DeclareLaunchArgument("gravity_compensation", default_value=mode_default("gravity_compensation"),
                             description="默认 real=true、fake/sim=false；显式传参可覆盖"),
        DeclareLaunchArgument(
            "start_arm_controller", default_value="true",
            description="是否自动启动轨迹控制器；MoveIt 加载环境时延后启动",
        ),
        DeclareLaunchArgument(
            "sdk_python", default_value=EnvironmentVariable(
                "ONERO_SDK_PYTHON", default_value=sys.executable
            ),
            description="SDK 桥接使用的 Python；setup_env.sh 优先选取 .venv-sdk",
        ),
        DeclareLaunchArgument("control_rate", default_value=mode_default("control_rate"),
                             description="话题桥默认 real=100 Hz，sim=200 Hz；fake 不使用"),
        DeclareLaunchArgument(
            "trace_control", default_value="false",
            description="发布 SDK 命令、反馈和 CAN 周期诊断，供 record_tracking.py 记录",
        ),
        DeclareLaunchArgument(
            "publish_world_tf",
            default_value="true",
            description="是否发布 world -> base_link 的静态 TF",
        ),
        DeclareLaunchArgument(
            "watchdog_timeout",
            default_value="1.0",
            description="话题桥失联判定阈值（秒），超时则停用控制器",
        ),
        DeclareLaunchArgument(
            "watchdog_auto_deactivate",
            default_value="true",
            description="失联时是否自动停用 arm_controller",
        ),
    ]

    # ─── 机器人描述 ──────────────────────────────────────────────────
    robot_description = ParameterValue(
        Command(
            [
                "xacro ", xacro_file,
                " use_fake_hardware:=", use_fake_hardware,
                " joint_commands_topic:=", joint_commands_topic,
                " joint_states_topic:=", joint_states_topic,
                " gripper_model:=", LaunchConfiguration("gripper_model"),
            ]
        ),
        value_type=str,
    )

    # ─── TF ──────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description}],
    )

    # MoveIt 的 SRDF virtual_joint 声明 base_link 挂在 world 下，
    # 因此 TF 树里也需要 world -> base_link
    def make_world_tf(context):
        mounting = LaunchConfiguration("mounting").perform(context)
        quaternion = [0.0, 0.0, 0.0, 1.0]
        if mounting != "upright":
            # Reuse SDK mounting definitions; no dynamics dependency here.
            from onero_sdk.mounting import get_mounting_rotation, resolve_mounting
            from scipy.spatial.transform import Rotation
            quaternion = Rotation.from_matrix(
                get_mounting_rotation(resolve_mounting(mounting))
            ).as_quat().tolist()
        return [Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="world_to_base_link", output="log",
            arguments=["--x", "0", "--y", "0", "--z", "0",
                       "--qx", str(quaternion[0]), "--qy", str(quaternion[1]),
                       "--qz", str(quaternion[2]), "--qw", str(quaternion[3]),
                       "--frame-id", "world", "--child-frame-id", "base_link"],
        )]

    world_tf = OpaqueFunction(
        function=make_world_tf,
        condition=IfCondition(LaunchConfiguration("publish_world_tf")),
    )

    # ─── ros2_control ────────────────────────────────────────────────
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_file, {"robot_description": robot_description}],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Real hardware starts position states and commands as NaN. Wait for live
    # bridge state before starting joint_state_broadcaster, so NaNs never reach
    # MoveIt/RSP. Then wait for the broadcaster's finite state before activating
    # JTC; activation must never use 0.0 as a fallback target for a real arm.
    bridge_state_ready = Node(
        package="onero_a1_bringup",
        executable="wait_for_joint_states",
        name="wait_for_bridge_joint_states",
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {
                "topic": joint_states_topic,
                "joints": [
                    "joint1-a1_r", "joint2-a1_r", "joint3-a1_r", "joint4-a1_r",
                    "joint5-a1_r", "joint6-a1_r", "joint7-a1_r",
                ],
            }
        ],
    )

    joint_state_ready = Node(
        package="onero_a1_bringup",
        executable="wait_for_joint_states",
        name="wait_for_valid_joint_states",
        output="screen",
        parameters=[
            {
                "joints": [
                    "joint1-a1_r", "joint2-a1_r", "joint3-a1_r", "joint4-a1_r",
                    "joint5-a1_r", "joint6-a1_r", "joint7-a1_r",
                ],
                "topic": "/joint_states",
                "gripper_model": LaunchConfiguration("gripper_model"),
            }
        ],
    )

    def spawn_broadcaster_after_bridge_state(event, _context):
        if event.returncode == 0:
            return [joint_state_broadcaster_spawner]
        return [LogInfo(msg="未收到真机关节状态；状态广播器和 arm_controller 保持未启动")]

    def activate_controller_after_state(event, context):
        if not IfCondition(LaunchConfiguration("start_arm_controller")).evaluate(context):
            return [LogInfo(msg="当前启动配置禁止自动启动 arm_controller（仅预览或等待环境检查）")]
        if event.returncode == 0:
            return [arm_controller_spawner]
        return [LogInfo(msg="未收到有效关节状态；arm_controller 保持未启动")]

    def spawn_broadcaster_for_fake_hardware(_event, context):
        if LaunchConfiguration("use_fake_hardware").perform(context).lower() == "true":
            return [joint_state_broadcaster_spawner]
        return []

    # ─── 话题桥适配节点（仅话题桥模式启动）──────────────────────────
    bridge_node = Node(
        package="onero_a1_bringup",
        executable="topic_bridge",
        prefix=[LaunchConfiguration("sdk_python")],
        name="onero_topic_bridge",
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {
                "joint_commands_topic": joint_commands_topic,
                "joint_states_topic": joint_states_topic,
                "simulate": ParameterValue(simulate, value_type=bool),
                "trace_control": ParameterValue(
                    LaunchConfiguration("trace_control"), value_type=bool
                ),
                "control_rate": ParameterValue(
                    LaunchConfiguration("control_rate"), value_type=float
                ),
                "can_channel": LaunchConfiguration("can_channel"),
                "model": LaunchConfiguration("model"),
                "mounting": LaunchConfiguration("mounting"),
                "with_gripper": ParameterValue(LaunchConfiguration("with_gripper"), value_type=bool),
                "gripper_model": LaunchConfiguration("gripper_model"),
                "allow_gripper_execution": ParameterValue(LaunchConfiguration("allow_gripper_execution"), value_type=bool),
                "activate_gripper_on_start": ParameterValue(LaunchConfiguration("start_arm_controller"), value_type=bool),
                "gripper_closed_sdk_opening": ParameterValue(LaunchConfiguration("gripper_closed_sdk_opening"), value_type=float),
                "gripper_open_sdk_opening": ParameterValue(LaunchConfiguration("gripper_open_sdk_opening"), value_type=float),
                "gripper_calibration_file": ParameterValue(LaunchConfiguration("gripper_calibration_file"), value_type=str),
                "gravity_compensation": ParameterValue(
                    LaunchConfiguration("gravity_compensation"), value_type=bool
                ),
                "kp": ParameterValue(LaunchConfiguration("kp"), value_type=float),
                "kd": ParameterValue(LaunchConfiguration("kd"), value_type=float),
                "velocity_feedforward": ParameterValue(
                    LaunchConfiguration("velocity_feedforward"), value_type=bool),
                "velocity_feedforward_scale": LaunchConfiguration("velocity_feedforward_scale"),
                "velocity_feedforward_timeout": ParameterValue(
                    LaunchConfiguration("velocity_feedforward_timeout"), value_type=float),
            }
        ],
    )

    def fake_gripper(_event, context):
        if (IfCondition(use_fake_hardware).evaluate(context)
                and LaunchConfiguration("gripper_model").perform(context) == "woan_picker"):
            return [Node(
                package="onero_a1_bringup", executable="woan_gripper_controller",
                name="gripper_controller", output="screen",
                parameters=[{"simulate": True,
                             "allow_execution": ParameterValue(LaunchConfiguration("allow_gripper_execution"), value_type=bool),
                             "activate_on_start": ParameterValue(LaunchConfiguration("start_arm_controller"), value_type=bool),
                             "closed_sdk_opening": ParameterValue(LaunchConfiguration("gripper_closed_sdk_opening"), value_type=float),
                             "open_sdk_opening": ParameterValue(LaunchConfiguration("gripper_open_sdk_opening"), value_type=float),
                             "calibration_file": ParameterValue(LaunchConfiguration("gripper_calibration_file"), value_type=str),
                             "arm_model": LaunchConfiguration("model"),
                             "can_channel": "simulation"}],
            )]
        return []

    # Fake hardware has finite initial values, so its broadcaster can start
    # with controller_manager. Real hardware waits for fresh bridge feedback.
    spawn_after_cm = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=spawn_broadcaster_for_fake_hardware,
        )
    )
    spawn_fake_gripper = RegisterEventHandler(
        OnProcessStart(target_action=ros2_control_node, on_start=fake_gripper))
    spawn_broadcaster_after_bridge = RegisterEventHandler(
        OnProcessExit(
            target_action=bridge_state_ready,
            on_exit=spawn_broadcaster_after_bridge_state,
        )
    )
    spawn_arm_after_valid_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_ready,
            on_exit=activate_controller_after_state,
        )
    )

    # ─── 健康看门狗（仅话题桥模式启动）──────────────────────────────
    # 硬件接口不校验状态时间戳，桥接节点挂掉时它会一直返回最后一帧，
    # MoveIt 会误判为正常。此节点负责失联时停用控制器。
    watchdog_node = Node(
        package="onero_a1_bringup",
        executable="health_watchdog",
        name="onero_bridge_watchdog",
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {
                "joint_states_topic": joint_states_topic,
                "controller_name": "arm_controller",
                "timeout": ParameterValue(
                    LaunchConfiguration("watchdog_timeout"), value_type=float
                ),
                "auto_deactivate": ParameterValue(
                    LaunchConfiguration("watchdog_auto_deactivate"), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(
        args
        + [
            robot_state_publisher,
            world_tf,
            ros2_control_node,
            bridge_node,
            watchdog_node,
            bridge_state_ready,
            joint_state_ready,
            spawn_after_cm,
            spawn_fake_gripper,
            spawn_broadcaster_after_bridge,
            spawn_arm_after_valid_state,
        ]
    )
