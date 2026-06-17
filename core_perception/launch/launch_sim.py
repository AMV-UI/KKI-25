#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from core.utils.config import Param

def generate_launch_description():
    track_arg = DeclareLaunchArgument(
        "track",
        default_value='B',
        description='Mission track (A or B)'
    )
    
    threshold_arg = DeclareLaunchArgument(
        "conf_threshold",
        default_value='0.3',
        description='Detection confidence threshold'
    )

    return LaunchDescription([
        track_arg,
        threshold_arg,
        
        Node(
            package="core_perception",
            executable="camera_controller_sim",
            name="camera_controller_sim",
            output="screen",
            parameters=[{
                "track": LaunchConfiguration("track"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
            }]
        ),
    ])
