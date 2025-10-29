#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from core.utils.config import Param

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="core_gcs",
            executable="gcs",
            name="gcs",
            output="screen",
        ),

        # Mock publisher data
        # Node(
        #     package="core_gcs",
        #     executable="mock_mission",
        #     name="mock_mission",
        #     output="screen",
        # ),
        # Node(
        #     package="core_gcs",
        #     executable="mock_pixhawk",
        #     name="mock_pixhawk",
        #     output="screen",
        # ),
    ])
