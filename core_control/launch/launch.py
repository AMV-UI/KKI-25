#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package="core_control",
        #     executable="microcontroller_asv",
        #     name="microcontroller_asv",
        #     output="screen"
        # ),
        Node(
            package="core_control",
            executable="motor_controller",
            name="motor_controller",
            output="screen"
        ),
        # Node(
        #     package="core_control",
        #     executable="pid_controller",
        #     name="pid_controller",
        #     output="screen"
        # ),
    ])
