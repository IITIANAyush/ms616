#!/usr/bin/env python3
"""
display.launch.py

Brings up the ms616 robot model for visualization in RViz2 (Humble).

Starts:
  - robot_state_publisher   (publishes TF from the URDF + joint states)
  - joint_state_publisher_gui (sliders to move left/right wheel joints)
  - rviz2                   (pre-loaded with urdf_view.rviz)

Usage:
  ros2 launch ms616 display.launch.py
  ros2 launch ms616 display.launch.py use_gui:=false   # headless joint publisher
  ros2 launch ms616 display.launch.py use_rviz:=false  # skip RViz
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ms616")

    urdf_file = os.path.join(pkg_share, "urdf", "ms616.urdf")
    default_rviz_config = os.path.join(pkg_share, "rviz", "urdf_view.rviz")

    with open(urdf_file, "r") as f:
        robot_description_content = f.read()

    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    declare_use_gui = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Launch joint_state_publisher_gui (sliders) instead of headless joint_state_publisher",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2 alongside the state publishers",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="Path to the RViz config file",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time},
        ],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(use_gui),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=UnlessCondition(use_gui),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            declare_use_gui,
            declare_use_rviz,
            declare_use_sim_time,
            declare_rviz_config,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            joint_state_publisher_node,
            rviz_node,
        ]
    )
