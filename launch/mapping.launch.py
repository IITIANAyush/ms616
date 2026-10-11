#!/usr/bin/env python3
"""Automated map exploration without moving-obstacle simulation or prediction.

Launch alongside mapping_gazebo.launch.py. Nav2 + autonomous_mapper explore
frontiers automatically. A software keyboard override is available through
cmd_vel_mux; no manual driving is required for normal mapping.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("ms616")
    config_dir = os.path.join(share, "config")
    nav_params = os.path.join(config_dir, "mapping_nav2_params.yaml")
    ekf_params = os.path.join(config_dir, "ekf.yaml")
    sim_time = LaunchConfiguration("use_sim_time")
    auto_explore = LaunchConfiguration("autonomous_explore")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "autonomous_explore", default_value="true",
            description="Automatically explore frontiers while mapping; disable only for debugging.",
        ),
        Node(
            package="robot_localization", executable="ekf_node", name="ekf_filter_node",
            output="screen", parameters=[ekf_params, {"use_sim_time": sim_time}],
        ),
        Node(
            package="cartographer_ros", executable="cartographer_node", name="cartographer_node",
            output="screen", parameters=[{"use_sim_time": sim_time}],
            arguments=["-configuration_directory", config_dir,
                       "-configuration_basename", "ms616_cartographer.lua"],
            remappings=[("odom", "/odometry/filtered"), ("scan", "/scan"), ("imu", "/imu/data_raw")],
        ),
        Node(
            package="cartographer_ros", executable="cartographer_occupancy_grid_node",
            name="cartographer_occupancy_grid_node", output="screen",
            parameters=[{"use_sim_time": sim_time}],
            arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
        ),
        Node(
            package="nav2_controller", executable="controller_server", name="controller_server",
            output="screen", parameters=[nav_params, {"use_sim_time": sim_time}],
            remappings=[("cmd_vel", "/nav_cmd_vel")],
        ),
        Node(
            package="nav2_planner", executable="planner_server", name="planner_server",
            output="screen", parameters=[nav_params, {"use_sim_time": sim_time}],
        ),
        Node(
            package="nav2_behaviors", executable="behavior_server", name="behavior_server",
            output="screen", parameters=[nav_params, {"use_sim_time": sim_time}],
            remappings=[("cmd_vel", "/nav_cmd_vel")],
        ),
        Node(
            package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
            output="screen", parameters=[nav_params, {"use_sim_time": sim_time}],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen",
            parameters=[{"use_sim_time": sim_time, "autostart": True,
                         "node_names": ["controller_server", "planner_server", "behavior_server", "bt_navigator"]}],
        ),
        Node(
            package="ms616", executable="cmd_vel_mux", name="cmd_vel_mux", output="screen",
            parameters=[{"use_sim_time": sim_time}],
        ),
        Node(
            package="ms616", executable="autonomous_mapper", name="autonomous_mapper",
            output="screen", parameters=[{"use_sim_time": sim_time,
                                          "min_obstacle_clearance": 0.40,
                                          "replan_period_sec": 2.0}],
            condition=IfCondition(auto_explore),
        ),
    ])
