"""웹(Spring Boot) ↔ ROS2 통합 launch.

manual_nav_controller       — /air_clean_command 처리기 (Nav2 / 청정기 / ESTOP)
cmd_mqtt_subscriber         — MQTT robot/1/cmd → /air_clean_command 다리
dust_cells_mqtt_publisher   — ROS /dust/cells → MQTT robot/1/dust (히트맵)

세 노드를 함께 띄운다. air_clean_localization.launch.py + dust_mapping
launch 가 이미 실행 중이라고 가정한다.
"""
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
            executable='manual_nav_controller',
            name='manual_nav_controller',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='air_clean_robot',
            executable='cmd_mqtt_subscriber',
            name='cmd_mqtt_subscriber',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='air_clean_robot',
            executable='dust_cells_mqtt_publisher',
            name='dust_cells_mqtt_publisher',
            output='screen',
            parameters=[params_file],
        ),
    ])
