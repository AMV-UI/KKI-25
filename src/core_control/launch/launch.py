#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = {
        "log_rc": "true",
        "log_manual_control": "false",
        "log_joystick": "false",
        "log_servo": "true",
        "info_throttle": "0",
        "scale_forward": "1.0",
        "scale_lateral": "1.0",
        "scale_yaw": "1.0",
        "scale_vertical": "1.0",
        "trim_forward": "0.0",
        "trim_lateral": "0.0",
        "trim_yaw": "0.0",
        "trim_vertical": "0.0",
    }

    gcs_param = DeclareLaunchArgument("gcs", default_value="false")

    launch_args = [
        DeclareLaunchArgument(name, default_value=val)
        for name, val in default_params.items()
    ] + [gcs_param]

    node_params = {name: LaunchConfiguration(name) for name in default_params.keys()}

    return LaunchDescription(
        launch_args
        + [
            Node(
                package="core_control",
                executable="px4",
                name="px4",
                output="screen",
                parameters=[node_params],
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
            ),
            Node(
                package="core_gcs",
                executable="gcs",
                name="gcs",
                output="screen",
                parameters=[{}],
                condition=IfCondition(LaunchConfiguration("gcs")),
            ),
        ]
    )
