from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    package1_dir = get_package_share_directory('package1')
    package2_dir = get_package_share_directory('package2')
    
    return LaunchDescription([
        # Launch individual nodes directly
        Node(
            package='package1',
            executable='node_executable',
            name='custom_node_name',
            parameters=[{'param1': 'value1'}]
        ),
        
        # Include launch files from other packages
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(package1_dir, 'launch'),
                '/package1_launch.py'
            ])
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(package2_dir, 'launch'),
                '/package2_launch.py'
            ]),
            launch_arguments={
                'arg1': 'value1',
                'arg2': 'value2'
            }.items()
        ),
    ])
