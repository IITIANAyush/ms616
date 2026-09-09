#!/usr/bin/env python3
"""
gazebo.launch.py

Spawns ms616 in Gazebo (Ignition Fortress, gz-sim 6.18.0), starts
ros2_control controllers (diff_drive_controller + joint_state_broadcaster),
and bridges /cmd_vel, /odom, /scan, /tf between Gazebo and ROS 2.

Usage:
  ros2 launch ms616 gazebo.launch.py
  ros2 launch ms616 gazebo.launch.py world:=<your_world>.sdf

NOTE ON FORTRESS + LIDAR: Ignition Fortress does NOT run the sensors
system plugin (needed for ANY <sensor> tag, including our lidar) unless
the world SDF explicitly loads it:
  <plugin filename="ignition-gazebo-sensors-system"
          name="ignition::gazebo::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
The default_world.sdf below (installed with this package) includes this
plugin plus physics/scene-broadcaster/user-commands, which the stock
ros_gz_sim empty.sdf world does NOT reliably include on Fortress. If you
use a different world, make sure IT also loads this plugin, or /scan
will stay empty with no error message at all.

NOTE ON SERVER/GUI SPLIT: on this machine the EGL/hardware render path
that the Sensors system needs for its off-screen Ogre2 context is
broken (dri2/EGL device selection fails), while the GUI's own on-screen
Ogre2 context works fine on real hardware. Forcing software rendering
(MESA_LOADER_DRIVER_OVERRIDE=llvmpipe + LIBGL_ALWAYS_SOFTWARE=1) globally
fixes the server's sensor init but crashes the GUI (segfault in
QOpenGLContext::doneCurrent via libMinimalScene), because it now also
hits the previously-working hardware GL context.

Fix: run gz-sim server (`-s`) and gz-sim GUI (`-g`) as two separate
processes, and only apply the software-render env vars to the server
process via ExecuteProcess(additional_env=...). The GUI process keeps
the ambient (working) environment untouched.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ms616")

    urdf_file = os.path.join(pkg_share, "urdf", "ms616.urdf")
    default_world = os.path.join(pkg_share, "worlds", "default_world.sdf")

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_world = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Gazebo world file — must load the ignition-gazebo-sensors-system "
                     "plugin or the lidar will silently produce no data on Fortress",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use Gazebo sim clock"
    )

    # NOTE: GZ_SIM_RESOURCE_PATH is set automatically via the
    # <gazebo_ros gazebo_model_path=.../> export in package.xml,
    # applied every time you `source install/setup.bash`.
    # Nothing to do here — no manual env var, no hardcoded path.

    with open(urdf_file, "r") as f:
        robot_description_content = f.read()

    # ign_ros2_control needs a real absolute filesystem path (it does not
    # resolve package:// URIs), so swap in the actual config dir here.
    config_dir = os.path.join(pkg_share, "config")
    robot_description_content = robot_description_content.replace(
        "__MS616_CONFIG_PATH__", config_dir
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time},
        ],
    )

    # Baked-in env vars so the server process doesn't depend on the
    # calling shell having exported these first (they were being lost
    # across terminal sessions, causing "couldn't find shared library"
    # and "Unable to find file with URI [model://...]" errors even
    # after installing the right packages / exporting the right vars
    # manually).
    #
    # - IGN_GAZEBO_SYSTEM_PLUGIN_PATH: where Gazebo looks for
    #   <plugin filename="..."> system plugins like
    #   libign_ros2_control-system.so / libgz_ros2_control-system.so.
    #   ros-humble-gz-ros2-control installs these under
    #   /opt/ros/humble/lib, which is NOT on Gazebo's default plugin
    #   search path.
    # - IGN_GAZEBO_RESOURCE_PATH: where Gazebo resolves model://...
    #   URIs. sdformat rewrites the URDF's package://ms616/meshes/...
    #   into model://ms616/meshes/... during URDF->SDF conversion, and
    #   model:// resolution expects <some_path_entry>/ms616/meshes/...
    #   — i.e. it needs the PARENT of the ms616 share dir on the path,
    #   not the ms616 dir itself.
    existing_plugin_path = os.environ.get("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "")
    existing_resource_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    ros_lib_dir = "/opt/ros/humble/lib"
    ms616_share_parent = os.path.dirname(pkg_share)  # .../install/ms616/share

    gz_server_env = {
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH": os.pathsep.join(
            p for p in [ros_lib_dir, existing_plugin_path] if p
        ),
        "IGN_GAZEBO_RESOURCE_PATH": os.pathsep.join(
            p for p in [ms616_share_parent, existing_resource_path] if p
        ),
        "MESA_LOADER_DRIVER_OVERRIDE": "llvmpipe",
        "LIBGL_ALWAYS_SOFTWARE": "1",
    }

    # --- Gazebo SERVER (headless, software-rendered sensors) ---
    # additional_env is layered on top of the current environment for
    # THIS process only — it does not leak into the GUI process below.
    gz_server = ExecuteProcess(
        cmd=["ign", "gazebo", "-s", "-r", "-v", "4", world],
        additional_env=gz_server_env,
        output="screen",
    )

    # --- Gazebo GUI (hardware-rendered, needs the resource path too
    # so meshes render correctly in the 3D view, but NOT the software
    # render vars — the GUI's own hardware Ogre2 context already works). ---
    gz_gui = ExecuteProcess(
        cmd=["ign", "gazebo", "-g", "-v", "4"],
        additional_env={
            "IGN_GAZEBO_RESOURCE_PATH": gz_server_env["IGN_GAZEBO_RESOURCE_PATH"],
        },
        output="screen",
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "ms616",
            "-z", "0.05",
        ],
        output="screen",
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Start controllers only after the entity has actually spawned
    delayed_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, diff_drive_spawner],
        )
    )

    # ros_gz bridge: sim clock, lidar scan, cmd_vel.
    # Using ignition.msgs.* (Fortress-native) rather than gz.msgs.* —
    # the ros_gz_bridge build paired with Fortress expects this naming.
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/model/ms616/sensor/lidar_2d/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
        ],
        remappings=[
            ("/model/ms616/sensor/lidar_2d/scan", "/scan"),
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_world,
            declare_use_sim_time,
            robot_state_publisher_node,
            gz_server,
            gz_gui,
            spawn_entity,
            delayed_controllers,
            gz_bridge,
            rviz_node,
        ]
    )