#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from core.utils.config import Param

def generate_launch_description():
    track_arg = DeclareLaunchArgument(
        Param.TRACK,
        default_value='A',
        description='Mission track (A or B)'
    )
    
    threshold_arg = DeclareLaunchArgument(
        Param.CONF_THRESHOLD,
        default_value='0.3',
        description='Detection confidence threshold'
    )

    return LaunchDescription([
        track_arg,
        threshold_arg,
        
        Node(
            package="core_perception",
            executable="camera_front",
            name="camera_front",
            output="screen",
            parameters=[{
                Param.TRACK: LaunchConfiguration(Param.TRACK),
                Param.CONF_THRESHOLD: LaunchConfiguration(Param.CONF_THRESHOLD),
            }]
        ),
    ])
