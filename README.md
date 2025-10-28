# core

to change what package to launch or to launch multiple package:

```python
#@core_launcher/launch/launcher.py

#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    target_packages = [
        # "core_perception",
        "core_control" # change these lines, do not forget the comma, uncomment multiple lines to launch specified scripts on their respective package
    ]

    packages_to_launch = []

    for pkg in target_packages:
        launch_file = os.path.join(
            get_package_share_directory(pkg),
            "launch",
            "launch.py"
        )

    packages_to_launch.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file)))
    
    return LaunchDescription(packages_to_launch)

```

to change what scripts to launch on their respective package, using core_control as an example:

```python
#@core_control/launch/launch.py

#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="core_control",
            executable="microcontroller_asv",
            name="microcontroller_asv",
            output="screen"
        ),
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
        # ), #uncomment these node and change it if you ever need one
    ])


```
