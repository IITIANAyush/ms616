#!/usr/bin/env python3
"""
Main navigation launch for MS616.

Pipeline:
  Gazebo sensors /diff_drive odometry
        -> Cartographer SLAM -> /map + map->odom
        -> Nav2 global planner (SmacPlanner2D)
        -> Nav2 local controller (DWB)
        -> /diff_drive_controller/cmd_vel_unstamped

Dynamic obstacle layer:
  /scan -> dynamic_obstacle_tracker -> /dynamic_obstacles/predicted_points
        -> Nav2 local costmap -> DWB trajectory scoring

This launch intentionally omits Nav2's optional smoother_server,
waypoint_follower and velocity_smoother. The robot is differential-drive,
and this project only needs NavigateToPose + planner/controller + recovery
behaviors for autonomous frontier exploration.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ms616")
    nav2_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    cartographer_config_dir = os.path.join(pkg_share, "config")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autonomous_explore = LaunchConfiguration("autonomous_explore")
    dynamic_avoidance = LaunchConfiguration("dynamic_avoidance")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autonomous_explore", default_value="true"),
        DeclareLaunchArgument(
            "dynamic_avoidance",
            default_value="true",
            description="Enable LiDAR dynamic-obstacle prediction",
        ),

        # ---------------- SLAM ----------------
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-configuration_directory", cartographer_config_dir,
                "-configuration_basename", "ms616_cartographer.lua",
            ],
            remappings=[
                ("odom", "/diff_drive_controller/odom"),
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

        # ---------------- Dynamic obstacle prediction ----------------
        Node(
            package="ms616",
            executable="dynamic_obstacle_tracker",
            name="dynamic_obstacle_tracker",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"scan_topic": "/scan"},
                {"prediction_topic": "/dynamic_obstacles/predicted_points"},
                {"odom_frame": "odom"},
                {"base_frame": "base_link"},
                {"prediction_horizon": 1.10},
                {"min_dynamic_speed": 0.10},
            ],
            condition=IfCondition(dynamic_avoidance),
        ),

        # ---------------- Minimal Nav2 ----------------
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            remappings=[
                ("cmd_vel", "/diff_drive_controller/cmd_vel_unstamped"),
            ],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            remappings=[
                ("cmd_vel", "/diff_drive_controller/cmd_vel_unstamped"),
            ],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": True},
                {
                    "node_names": [
                        "controller_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                    ]
                },
            ],
        ),

        # ---------------- Frontier exploration ----------------
        Node(
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
        ),
    ])
