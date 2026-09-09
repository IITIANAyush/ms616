#!/usr/bin/env python3
"""
explore.launch.py

Brings up SLAM (slam_toolbox), Nav2 navigation stack, and the
frontier_explorer node for fully autonomous exploration/mapping.

Run this AFTER gazebo.launch.py (or after your real robot's driver +
lidar are publishing /scan and TF).

Usage:
  ros2 launch ms616 gazebo.launch.py         # terminal 1
  ros2 launch ms616 explore.launch.py        # terminal 2
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ms616")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "slam.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
        }.items(),
    )

    frontier_explorer_node = Node(
        package="ms616",
        executable="frontier_explorer",
        name="frontier_explorer",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_nav2_params,
            slam_launch,
            nav2_bringup_launch,
            frontier_explorer_node,
        ]
    )
