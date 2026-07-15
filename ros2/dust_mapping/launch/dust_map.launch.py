from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('dust_mapping')
    params_file = os.path.join(package_share, 'config', 'dust_mapping_params.yaml')

    return LaunchDescription([
        Node(
            package='dust_mapping',
            executable='mqtt_pm_listener',
            name='mqtt_pm_listener',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='dust_mapping',
            executable='dust_mapper',
            name='dust_mapper',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='dust_mapping',
            executable='dust_map_viewer',
            name='dust_map_viewer',
            output='screen',
            parameters=[params_file],
        ),
    ])
