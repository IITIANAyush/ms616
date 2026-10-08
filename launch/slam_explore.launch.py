#!/usr/bin/env python3
<<<<<<< HEAD
"""Backward-compatible wrapper for dynamic_nav.launch.py."""
=======
"""
slam_explore_clean.launch.py

Clean SLAM/exploration pipeline:
  wheel encoders -> diff_drive_controller odometry -> Cartographer
  IMU -------------------------------> Cartographer
  LiDAR -----------------------------> Cartographer

No robot_localization EKF is inserted between encoder odometry and
Cartographer. The diff_drive_controller is the single owner of
odom -> base_link for this test.

Run after gazebo.launch.py.
"""
>>>>>>> origin/main

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
<<<<<<< HEAD
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
=======
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
>>>>>>> origin/main


def generate_launch_description():
    pkg_share = get_package_share_directory("ms616")
<<<<<<< HEAD
    launch_file = PythonLaunchDescriptionSource(
        os.path.join(pkg_share, "launch", "dynamic_nav.launch.py")
    )
    return LaunchDescription([IncludeLaunchDescription(launch_file)])
=======

    nav2_params_file = os.path.join(pkg_share, "config", "nav2_params.yaml")
    cartographer_config_dir = os.path.join(pkg_share, "config")
    cartographer_lua_file = "ms616_cartographer.lua"

    use_sim_time = LaunchConfiguration("use_sim_time")
    autonomous_explore = LaunchConfiguration("autonomous_explore")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true"
    )
    declare_autonomous_explore = DeclareLaunchArgument(
        "autonomous_explore", default_value="true"
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory", cartographer_config_dir,
            "-configuration_basename", cartographer_lua_file,
        ],
        remappings=[
            # IMPORTANT: raw wheel-encoder odometry, NOT EKF output.
            ("odom", "/diff_drive_controller/odom"),
            ("scan", "/scan"),
            ("imu", "/imu/data_raw"),
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
    )

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "slam": "true",
        }.items(),
    )

    # Nav2 uses the same raw encoder odometry TF published by
    # diff_drive_controller. No EKF is required for this baseline.
    nav2_launch = GroupAction(
        actions=[
            SetRemap(
                src="/cmd_vel",
                dst="/diff_drive_controller/cmd_vel_unstamped",
            ),
            nav2_include,
        ]
    )

    autonomous_mapper_node = Node(
        package="ms616",
        executable="autonomous_mapper",
        name="autonomous_mapper",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"min_obstacle_clearance": 0.40},
            {"replan_period_sec": 2.0},
        ],
        condition=IfCondition(autonomous_explore),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autonomous_explore,
        cartographer_node,
        cartographer_occupancy_grid_node,
        nav2_launch,
        autonomous_mapper_node,
    ])
>>>>>>> origin/main
