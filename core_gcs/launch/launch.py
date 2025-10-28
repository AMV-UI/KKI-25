#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from core.utils.config import Param

def generate_launch_description():
    return LaunchDescription([
        track_arg,
        threshold_arg,
        
        Node(
            package="core_gcs",
            executable="gcs",
            name="camera_controller",
            output="screen",
        ),
    ])
