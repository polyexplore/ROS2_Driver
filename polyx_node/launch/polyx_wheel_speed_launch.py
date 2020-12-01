from launch import LaunchDescription
from launch_ros.actions import Node

import launch.actions
# import launch_ros.actions ## for launch.actions

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #  parameters
    param_file = os.path.join(get_package_share_directory('polyx_node'), 
        'param',
        'polyx_wheel_speed_params.yaml')

    return LaunchDescription([
        
        # Set env var to print messages to stdout immediately
        # requirement #1
        launch.actions.SetEnvironmentVariable(
            'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        
        # dashing 
        Node(
            package='polyx_node',
            node_namespace='polyx_events', # kick in, override
            node_executable='polyx_node_wheel_speed',
            name='polyx_name', # useless
            output='screen', # requirement #2
            parameters=[param_file]
        )
    ])