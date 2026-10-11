#!/usr/bin/env python3
"""Run automated straight / curved / rotation tests against Gazebo ground truth.

Start gazebo.launch.py separately with dynamic_obstacles:=false and without
any navigation launch. This launch only adds a ROS<->Ignition ground-truth
bridge and a test node which commands the differential-drive controller.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    ground_truth_topic = LaunchConfiguration("ground_truth_topic")
    bridge_argument = PythonExpression([
        "'", ground_truth_topic, "@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V'"
    ])
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="kinematic_ground_truth_bridge",
        arguments=[bridge_argument],
        remappings=[(ground_truth_topic, "/gazebo_ground_truth/poses")],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    validator = Node(
        package="ms616",
        executable="kinematic_validation_node",
        name="kinematic_validation_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "ground_truth_topic": "/gazebo_ground_truth/poses",
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "wheel_radius": wheel_radius,
            "wheel_separation": wheel_separation,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "ground_truth_topic", default_value="/world/ms616_default/dynamic_pose/info",
            description="Gazebo PosePublisher topic for ground-truth poses",
        ),
        DeclareLaunchArgument("spawn_x", default_value="-0.2", description="Robot spawn x in Gazebo world coordinates"),
        DeclareLaunchArgument("spawn_y", default_value="0.2", description="Robot spawn y in Gazebo world coordinates"),
        DeclareLaunchArgument("wheel_radius", default_value="0.035", description="Match URDF and diff_drive_controller.yaml"),
        DeclareLaunchArgument("wheel_separation", default_value="0.21", description="Match diff_drive_controller.yaml"),
        bridge,
        validator,
    ])
