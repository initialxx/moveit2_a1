"""Onero A1 独立显示 —— 只看机器人模型，不涉及 MoveIt。

用途：验证 URDF / mesh / TF 是否正确。
     可用 joint_state_publisher_gui 的滑块手动拖动各关节。

用法：
    ros2 launch onero_a1_description display.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("onero_a1_description")
    xacro_file = os.path.join(pkg_share, "urdf", "onero_a1.urdf.xacro")
    rviz_config = os.path.join(pkg_share, "rviz", "display.rviz")

    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " gripper_model:=", LaunchConfiguration("gripper_model")]),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gripper_model", default_value="woan_picker", choices=["none", "woan_picker"]),
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="是否启动 joint_state_publisher_gui（手动拖动关节）",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="是否启动 RViz",
            ),
            # URDF -> TF
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[{"robot_description": robot_description}],
            ),
            # 独立模型显示使用手动关节滑块，不启动控制链路。
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
                condition=IfCondition(use_gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config],
                parameters=[{"robot_description": robot_description}],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
