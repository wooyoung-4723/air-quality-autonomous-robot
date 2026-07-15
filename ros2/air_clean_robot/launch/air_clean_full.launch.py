from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('air_clean_robot')
    launch_dir = os.path.join(package_share, 'launch')

    localization_launch = os.path.join(
        launch_dir,
        'air_clean_localization.launch.py'
    )

    astar_launch = os.path.join(
        launch_dir,
        'air_clean_astar.launch.py'
    )

    mqtt_host_arg = DeclareLaunchArgument(
        'mqtt_host',
        default_value='192.168.0.55',
        description='MQTT broker host IP'
    )

    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT broker port'
    )

    mqtt_host = LaunchConfiguration('mqtt_host')
    mqtt_port = LaunchConfiguration('mqtt_port')

    return LaunchDescription([
        mqtt_host_arg,
        mqtt_port_arg,

        # 1. localization 먼저 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch)
        ),

        # 2. localization이 뜰 시간을 조금 준 뒤 A* + MQTT 실행
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(astar_launch),
                    launch_arguments={
                        'mqtt_host': mqtt_host,
                        'mqtt_port': mqtt_port,
                    }.items()
                )
            ]
        ),
    ])
