#!/usr/bin/env python3
<<<<<<< HEAD
"""Backward-compatible wrapper for dynamic_nav.launch.py."""
=======
"""
controller.launch.py

Brings up the reactive obstacle-avoidance node and the localisation
EKF together. Intended to run ALONGSIDE gazebo.launch.py (which
already provides /scan, /diff_drive_controller/odom, and /cmd_vel via
the sim + ros2_control stack) — this file does not start Gazebo
itself.

Usage:
  ros2 launch ms616 gazebo.launch.py      # terminal 1
  ros2 launch ms616 controller.launch.py  # terminal 2
"""
>>>>>>> origin/main

import os

from ament_index_python.packages import get_package_share_directory
<<<<<<< HEAD
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
=======

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
>>>>>>> origin/main


def generate_launch_description():
    pkg_share = get_package_share_directory("ms616")
<<<<<<< HEAD
    launch_file = PythonLaunchDescriptionSource(
        os.path.join(pkg_share, "launch", "dynamic_nav.launch.py")
    )
    return LaunchDescription([IncludeLaunchDescription(launch_file)])
=======
    ekf_config = os.path.join(pkg_share, "controller", "localisation", "ekf.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use Gazebo sim clock"
    )

    vfh_avoider_node = Node(
        package="ms616",
        executable="vfh_avoider",
        name="vfh_avoider",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        # Tuning overrides can be added here later, e.g.:
        # parameters=[{"use_sim_time": use_sim_time},
        #             {"linear_speed": 0.2}, {"stop_distance": 0.4}],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            vfh_avoider_node,
            ekf_node,
        ]
    )
>>>>>>> origin/main
