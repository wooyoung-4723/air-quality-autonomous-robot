from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('air_clean_robot')
    params_file = os.path.join(package_share, 'config', 'air_clean_params.yaml')

    return LaunchDescription([
        Node(
            package='air_clean_robot',
            executable='sim_air_quality_publisher',
            name='sim_air_quality_publisher',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='air_clean_robot',
            executable='air_quality_task_manager',
            name='air_quality_task_manager',
            output='screen',
            parameters=[
                params_file,
                {'use_nav2': False},
            ],
        ),
    ])
