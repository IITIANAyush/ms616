#!/usr/bin/env python3
<<<<<<< HEAD
"""Backward-compatible wrapper for dynamic_nav.launch.py."""
=======
"""
slam_explore.launch.py

Brings up the full autonomous mapping stack:
  robot_localization (EKF, fuses wheel odom + IMU)
    -> cartographer_node (SLAM, publishes /map and map->odom TF)
    -> Nav2 bringup (planner/controller/costmaps)
    -> autonomous_mapper (frontier exploration, unchanged from before)

CHANGED FROM PREVIOUS VERSION:
  - slam_toolbox replaced with cartographer_node + occupancy_grid_node
  - robot_localization ekf_node added upstream of Cartographer
  - Nav2 now launched with slam:="true" pointed at Cartographer's /map
    (Cartographer's occupancy_grid_node republishes the same /map
    topic + message type slam_toolbox used, so Nav2's launch args and
    autonomous_mapper.py's /map subscription need NO changes)

Run ALONGSIDE gazebo.launch.py, which must already be running and
providing /scan, /diff_drive_controller/odom, /imu/data_raw, and the
odom -> base_link TF chain (before Cartographer takes over map->odom;
see ekf.yaml / cartographer lua comments for the frame ownership split).

Usage:
  ros2 launch ms616 gazebo.launch.py        # terminal 1
  ros2 launch ms616 slam_explore.launch.py  # terminal 2

To just build the map by driving manually (no autonomous
exploration), launch with autonomous_explore:=false and drive with
teleop_twist_keyboard instead.
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
    ekf_params_file = os.path.join(pkg_share, "config", "ekf.yaml")
    cartographer_config_dir = os.path.join(pkg_share, "config")
    cartographer_lua_file = "ms616_cartographer.lua"

    use_sim_time = LaunchConfiguration("use_sim_time")
    autonomous_explore = LaunchConfiguration("autonomous_explore")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use Gazebo sim clock"
    )
    declare_autonomous_explore = DeclareLaunchArgument(
        "autonomous_explore",
        default_value="true",
        description="If true, run autonomous_mapper to auto-explore via Nav2. "
                     "If false, only SLAM + Nav2 bringup start (drive manually).",
    )

    # --- EKF: fuses /diff_drive_controller/odom + /imu/data_raw ---
    # Publishes /odometry/filtered and the odom -> base_link TF.
    # This MUST be running before Cartographer starts, since
    # Cartographer's use_odometry=true in the lua config expects
    # /odometry/filtered to exist.
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params_file, {"use_sim_time": use_sim_time}],
    )

    # --- Cartographer: SLAM, publishes /map and map -> odom TF ---
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
            ("odom", "/odometry/filtered"),
            ("scan", "/scan"),
            ("imu", "/imu/data_raw"),
        ],
    )

    # cartographer_occupancy_grid_node converts Cartographer's internal
    # submap representation into the standard nav_msgs/OccupancyGrid on
    # /map that Nav2 and autonomous_mapper.py both expect — this is
    # what makes the SLAM-backend swap transparent to your existing code.
    cartographer_occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
    )

    # --- Nav2 bringup: planner, controller, costmaps, BT navigator ---
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            # Cartographer (like slam_toolbox before) already provides
            # map -> odom, so Nav2 should not run AMCL / its own map
            # server here either.
            "slam": "true",
        }.items(),
    )

    # DiffDriveController (ros2_control) subscribes on
    # /diff_drive_controller/cmd_vel_unstamped, not plain /cmd_vel —
    # unchanged from your original setup.
    nav2_launch = GroupAction(
        actions=[
            SetRemap(src="/cmd_vel", dst="/diff_drive_controller/cmd_vel_unstamped"),
            nav2_include,
        ]
    )

    # --- Frontier exploration node (unchanged) ---
    autonomous_mapper_node = Node(
        package="ms616",
        executable="autonomous_mapper",
        name="autonomous_mapper",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(autonomous_explore),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_autonomous_explore,
            ekf_node,
            cartographer_node,
            cartographer_occupancy_grid_node,
            nav2_launch,
            autonomous_mapper_node,
        ]
    )
>>>>>>> origin/main
