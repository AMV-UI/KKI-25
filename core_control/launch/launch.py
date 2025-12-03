#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="core_control",
            executable="pixhawk_controller",
            name="pixhawk_controller",
            output="screen",
        ),
        Node(
            package="core_control",
            executable="esp_controller",
            name="esp_controller",
            output="screen",
        ),
        Node(
            package="core_control",
            executable="pwm_controller",
            name="pwm_controller",
            output="screen",
        ),
        Node(
            package="core_control",
            executable="motor_controller",
            name="motor_controller",
            output="screen",
        ),
        # Node(
        #     package="core_control",
        #     executable="movement_simulator",
        #     name="movement_simulator",
        #     output="screen",
        # ),
        # Node(
        #     package="core_control",
        #     executable="channel_simulator",
        #     name="channel_simulator",
        #     output="screen",
        # ),
        Node(
            package="core_control",
            executable="movement_controller",
            name="movement_controller",
            output="screen",
        ),
        # Node(
        #     package="core_control",
        #     executable="pid_controller",
        #     name="pid_controller",
        #     output="screen",
        # ),
    ])
