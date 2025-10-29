#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="core_behavior_tree",
            executable="mission",
            name="mission_node",
            output="screen",
            # parameters=[{
            #     'track': LaunchConfiguration('track'),
            # }]
        )
    ])
