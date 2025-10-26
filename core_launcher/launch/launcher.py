#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    target_packages = [
        "core_perception"
        "core_control"
    ]

    packages_to_launch = []

    for pkg in target_packages:
        launch_file = os.path.join(
            get_package_share_directory(pkg),
            "launch",
            "launch.py"
        )
        packages_to_launch.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file)
            )
        )
    
    return LaunchDescription(packages_to_launch)
