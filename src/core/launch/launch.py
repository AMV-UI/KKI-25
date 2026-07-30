#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="core",
            executable="param",
            name="param",        
            output="screen",
        )
    ])