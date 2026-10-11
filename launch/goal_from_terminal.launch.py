#!/usr/bin/env python3
"""Send one goal to already-running Nav2 using x, y and yaw in map frame."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw_deg = LaunchConfiguration("yaw_deg")
    return LaunchDescription([
        DeclareLaunchArgument("x", default_value="0.0", description="Goal x in map frame [m]"),
        DeclareLaunchArgument("y", default_value="0.0", description="Goal y in map frame [m]"),
        DeclareLaunchArgument("yaw_deg", default_value="0.0", description="Goal yaw [degrees]"),
        Node(
            package="ms616",
            executable="terminal_goal",
            name="terminal_goal",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "x": ParameterValue(x, value_type=float),
                "y": ParameterValue(y, value_type=float),
                "yaw_deg": ParameterValue(yaw_deg, value_type=float),
                "frame_id": "map",
            }],
        ),
    ])
