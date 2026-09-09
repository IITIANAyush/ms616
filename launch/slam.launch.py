#!/usr/bin/env python3
"""
slam.launch.py

Runs slam_toolbox in online-async mode against /scan + TF to build
an occupancy grid map in real time.

Usage:
  ros2 launch ms616 slam.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ms616")
    default_params = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use Gazebo sim clock"
    )
    declare_params = DeclareLaunchArgument(
        "slam_params_file", default_value=default_params,
        description="Path to slam_toolbox params yaml",
    )

    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_params,
            slam_toolbox_node,
        ]
    )
