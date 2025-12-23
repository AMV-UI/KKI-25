#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    track_arg = DeclareLaunchArgument(
        'track',  # launch argument name
        default_value='B',
        description='Mission track (A or B)'
    )

    return LaunchDescription([
        track_arg,
        Node(
            package="core_gcs",
            executable="gcs",
            name="gcs",
            output="screen",
            parameters=[{
                '/mission/track': LaunchConfiguration('track'),  # absolute param name
            }]
        ),
        # --- Use on mocking data to gcs
        # Node(
        #     package="core_gcs",
        #     executable="mock_pixhawk",
        #     name="mock_pixhawk",
        #     output="screen",
        # ),
        # Node(
        #     package="core_gcs",
        #     executable="mock_trial",
        #     name="mock_trial",
        #     output="screen",
        # ),
    ])
