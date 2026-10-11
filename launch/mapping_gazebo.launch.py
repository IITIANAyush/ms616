#!/usr/bin/env python3
"""Gazebo bringup using a map-building world with no moving-obstacle models."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("ms616")
    gazebo_launch = os.path.join(share, "launch", "gazebo.launch.py")
    mapping_world = os.path.join(share, "worlds", "mapping_world.sdf")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                "world": mapping_world,
                "dynamic_obstacles": "false",
                "rviz": rviz,
                "use_sim_time": use_sim_time,
            }.items(),
        ),
    ])
