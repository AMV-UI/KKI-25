#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="core_perception",
                executable="cameras",
                name="cameras",
                output="screen",
                parameters=[{}],
            ),
            Node(
                package="core_gcs",
                executable="gcs_video",
                name="gcs_video",
                output="screen",
                parameters=[{}],
            ),
        ],
    )
