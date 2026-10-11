#!/usr/bin/env python3
"""Localize in a saved map and navigate using standard Nav2 components.

EKF owns odom->base_link. AMCL uses /scan + the saved map and owns map->odom.
Current LiDAR scans feed both costmaps for new-obstacle avoidance.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    share = get_package_share_directory("ms616")
    config_dir = os.path.join(share, "config")
    params_file = os.path.join(config_dir, "saved_map_nav2_params.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(config_dir, "ekf.yaml"), {"use_sim_time": use_sim_time}],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
    )
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    localization_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
    )

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")],
    )
    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")],
    )
    navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    navigation_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": [
                "controller_server", "planner_server", "behavior_server", "bt_navigator"
            ],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.expanduser("~/ros2_ws/maps/ms616_map.yaml"),
            description="Absolute path to the saved map YAML",
        ),
        ekf,
        map_server,
        amcl,
        localization_lifecycle,
        GroupAction(actions=[
            SetRemap(src="/cmd_vel", dst="/diff_drive_controller/cmd_vel_unstamped"),
            controller,
            planner,
            behaviors,
            navigator,
            navigation_lifecycle,
        ]),
    ])
