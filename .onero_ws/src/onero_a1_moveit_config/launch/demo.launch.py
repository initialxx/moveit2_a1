"""Onero A1 + MoveIt 2 一键启动。

包含：ros2_control（含话题桥适配层） + move_group + RViz
      + 静态 TF world->base_link

三种模式（见 onero_a1_bringup/launch/bringup.launch.py 的说明）:

    # 1) 纯 MoveIt 模拟（默认）—— 不涉及 armsdk，验证 MoveIt 配置
    ros2 launch onero_a1_moveit_config demo.launch.py

    # 2) 全链路联调（无真机）★ 推荐的第一步
    ros2 launch onero_a1_moveit_config demo.launch.py \
        use_fake_hardware:=false simulate:=true

    # 3) 真机
    ros2 launch onero_a1_moveit_config demo.launch.py \
        use_fake_hardware:=false simulate:=false can_channel:=can0

无头环境（无 X）请加 rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_pkg = get_package_share_directory("onero_a1_moveit_config")
    desc_pkg = get_package_share_directory("onero_a1_description")
    bringup_pkg = get_package_share_directory("onero_a1_bringup")

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    simulate = LaunchConfiguration("simulate")
    use_rviz = LaunchConfiguration("rviz")

    args = [
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument("simulate", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("can_channel", default_value="can0"),
        DeclareLaunchArgument("model", default_value="A1R"),
        DeclareLaunchArgument(
            "rviz_config", default_value="moveit.rviz",
            description="RViz 配置文件名（位于本包 rviz/ 下）",
        ),
    ]

    # ─── MoveIt 参数 ─────────────────────────────────────────────────
    # 用 MoveItConfigsBuilder 拼装，它会自动把各文件放到正确的命名空间下：
    #   kinematics.yaml -> robot_description_kinematics.arm.*
    #   joint_limits.yaml -> robot_description_planning.joint_limits.*
    #   ompl_planning.yaml -> ompl.*
    # 手写这些前缀极易出错，且出错时往往【静默失效】。
    moveit_config = (
        MoveItConfigsBuilder("onero_a1", package_name="onero_a1_moveit_config")
        .robot_description(
            file_path=os.path.join(desc_pkg, "urdf", "onero_a1.urdf.xacro"),
            mappings={
                "use_fake_hardware": use_fake_hardware,
                "joint_commands_topic": "/robot_joint_commands",
                "joint_states_topic": "/robot_joint_states",
            },
        )
        .robot_description_semantic(file_path="config/onero_a1.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    # ─── 执行层（ros2_control + 话题桥）───────────────────────────────
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
            "simulate": simulate,
            "can_channel": LaunchConfiguration("can_channel"),
            "model": LaunchConfiguration("model"),
        }.items(),
    )

    # ─── move_group ──────────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # ─── RViz ────────────────────────────────────────────────────────
    # 注意：不能用 os.path.join 拼 LaunchConfiguration，要用 PathJoinSubstitution
    rviz_config = PathJoinSubstitution(
        [moveit_pkg, "rviz", LaunchConfiguration("rviz_config")]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(args + [bringup, move_group_node, rviz_node])
