#!/usr/bin/env python3
"""Build a clean occupancy map; no Nav2 and no dynamic-obstacle nodes.

Run this alongside Gazebo loaded with worlds/mapping_world.sdf.
Drive the robot manually, then save /map with nav2_map_server's map_saver_cli.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("ms616")
    config_dir = os.path.join(share, "config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # EKF: wheel-encoder odometry + IMU -> /odometry/filtered and odom->base_link.
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[os.path.join(config_dir, "ekf.yaml"), {"use_sim_time": use_sim_time}],
        ),
        # Cartographer combines laser scans with filtered odometry and IMU for SLAM.
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-configuration_directory", config_dir,
                "-configuration_basename", "ms616_cartographer.lua",
            ],
            remappings=[
                ("odom", "/odometry/filtered"),
                ("scan", "/scan"),
                ("imu", "/imu/data_raw"),
            ],
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="cartographer_occupancy_grid_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
        ),
    ])
