#!/usr/bin/env python3
"""Backward-compatible wrapper for dynamic_nav.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory("ms616")
    launch_file = PythonLaunchDescriptionSource(
        os.path.join(pkg_share, "launch", "dynamic_nav.launch.py")
    )
    return LaunchDescription([IncludeLaunchDescription(launch_file)])
