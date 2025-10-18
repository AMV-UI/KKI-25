#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="core_perception",
            executable="camera_controller",
            name="camera_controller",
            output="screen"
        )
    ])
