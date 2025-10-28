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
            executable="gcs_socket",
            name="gcs_socket",
            output="screen",
        ),

        # Mock publisher data
        # Node(
        #     package="core_gcs",
        #     executable="talker",
        #     name="talker",
        #     output="screen",
        # ),
        Node(
            package="core_gcs",
            executable="talker2",
            name="talker2",
            output="screen",
        ),
    ])
