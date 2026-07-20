#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_rc_config = LaunchConfiguration("log_rc")
    log_joystick_config = LaunchConfiguration("log_joystick")
    log_manual_control_config = LaunchConfiguration("log_manual_control")
    info_throttle_config = LaunchConfiguration("info_throttle")
    log_rc_arg = DeclareLaunchArgument("log_rc", default_value="true", description="")
    log_manual_control_arg = DeclareLaunchArgument(
        "log_manual_control", default_value="true", description=""
    )
    log_joystick_arg = DeclareLaunchArgument(
        "log_joystick", default_value="true", description=""
    )
    info_throttle_arg = DeclareLaunchArgument(
        "info_throttle", default_value="5000", description=""
    )

    return LaunchDescription(
        [
            log_rc_arg,
            log_manual_control_arg,
            log_joystick_arg,
            info_throttle_arg,
            Node(
                package="core_control",
                executable="px4",
                name="px4",
                output="screen",
                parameters=[
                    {
                        "log_rc": log_rc_config,
                        "log_manual_control": log_manual_control_config,
                        "log_joystick": log_joystick_config,
                        "info_throttle": info_throttle_config,
                    }
                ],
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
            ),
        ],
    )
