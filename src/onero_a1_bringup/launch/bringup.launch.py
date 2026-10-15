"""Onero A1 bringup —— 启动 ros2_control 与话题桥适配层。

三种运行模式（用两个参数组合）:

┌────────────────────────┬───────────┬──────────────┬─────────────────────────┐
│ 命令                    │ 硬件插件   │ 桥接节点      │ 用途                     │
├────────────────────────┼───────────┼──────────────┼─────────────────────────┤
│ (默认)                  │ mock_      │ 不启动        │ 纯 MoveIt 模拟，验证配置  │
│                        │ components │              │                         │
│ use_fake_hardware:=false│ topic 桥   │ simulate=true│ 全链路联调（无真机）      │
│                        │           │              │ ★ 推荐的第一步           │
│ use_fake_hardware:=false│ topic 桥   │ simulate=false│ 真机运行                │
│ simulate:=false        │           │              │                         │
└────────────────────────┴───────────┴──────────────┴─────────────────────────┘

注意：本 launch 只负责【执行层】（ros2_control + 桥接）。
     MoveIt 的 move_group / RViz 由 onero_a1_moveit_config 包启动。

     ros2 launch onero_a1_moveit_config demo.launch.py use_fake_hardware:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="true: mock_components 纯模拟 | false: 话题桥",
        ),
        DeclareLaunchArgument(
            "simulate",
            default_value="true",
            description="话题桥模式下属：true 不连硬件（模拟机械臂）| false 连真机 CAN",
        ),
        DeclareLaunchArgument("joint_commands_topic", default_value="/robot_joint_commands"),
        DeclareLaunchArgument("joint_states_topic", default_value="/robot_joint_states"),
        DeclareLaunchArgument("can_channel", default_value="can0"),
        DeclareLaunchArgument("model", default_value="A1R"),
        DeclareLaunchArgument("kp", default_value="50.0"),
        DeclareLaunchArgument("kd", default_value="1.0"),
        DeclareLaunchArgument("control_rate", default_value="200.0"),
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
    world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_link",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
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

    # ─── 话题桥适配节点（仅话题桥模式启动）──────────────────────────
    bridge_node = Node(
        package="onero_a1_bringup",
        executable="topic_bridge",
        name="onero_topic_bridge",
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {
                "joint_commands_topic": joint_commands_topic,
                "joint_states_topic": joint_states_topic,
                "simulate": ParameterValue(simulate, value_type=bool),
                "control_rate": ParameterValue(
                    LaunchConfiguration("control_rate"), value_type=float
                ),
                "can_channel": LaunchConfiguration("can_channel"),
                "model": LaunchConfiguration("model"),
                "kp": ParameterValue(LaunchConfiguration("kp"), value_type=float),
                "kd": ParameterValue(LaunchConfiguration("kd"), value_type=float),
            }
        ],
    )

    # 等 ros2_control_node 起来再 spawn 控制器，避免竞态
    spawn_after_cm = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner, arm_controller_spawner],
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
            spawn_after_cm,
        ]
    )
