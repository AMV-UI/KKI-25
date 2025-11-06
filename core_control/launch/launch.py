#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments for this package
    track_arg = DeclareLaunchArgument(
        'track',
        default_value='B',
        description='Mission track (A or B)'
    )
    
    motor_speed_arg = DeclareLaunchArgument(
        'motor_speed',
        default_value='1.0',
        description='Motor speed multiplier'
    )
    
    x_speed_arg = DeclareLaunchArgument(
        'x_speed',
        default_value='1.0',
        description='X-axis speed multiplier'
    )

    return LaunchDescription([
        track_arg,
        motor_speed_arg,
        x_speed_arg,
        
        Node(
            package="core_control",
            executable="microcontroller_asv",
            name="microcontroller_asv",
            output="screen",
            # parameters=[{ ##if you want to pass any param just uncomment these lines
            #     'track': LaunchConfiguration('track'),
            # }]
        ),
        Node(
            package="core_control",
            executable="motor_controller",
            name="motor_controller",
            output="screen",
            # parameters=[{ ##if you want to pass any param just uncomment these lines
            #     'track': LaunchConfiguration('track'),
            #     'motor_speed': LaunchConfiguration('motor_speed'),
            #     'x_speed': LaunchConfiguration('x_speed'),
            # }]
        ),
        # Node(
        #     package="core_control",
        #     executable="pid_controller",
        #     name="pid_controller",
        #     output="screen"
        # ),
    ])
