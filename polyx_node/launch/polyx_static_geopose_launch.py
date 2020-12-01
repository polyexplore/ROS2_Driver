from launch import LaunchDescription
from launch_ros.actions import Node

import launch.actions

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    param_file = os.path.join(get_package_share_directory('polyx_node'), 
        'param',
        'polyx_static_geopose_params.yaml')
    return LaunchDescription([
        
        launch.actions.SetEnvironmentVariable(
            'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        
        # dashing 
        Node(
            package='polyx_node',
            node_namespace='polyx_events', # this overrides Node constructor's ns 
            node_executable='polyx_node_static_geopose',
            name='', # this will have no effect, overwritten by Node constructor
            output='screen',
            parameters=[param_file]
        )
    ])