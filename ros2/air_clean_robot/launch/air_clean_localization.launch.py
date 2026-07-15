from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    turtlebot3_navigation_share = get_package_share_directory('turtlebot3_navigation2')

    default_params = os.path.join(
        turtlebot3_navigation_share,
        'param',
        'humble',
        'waffle_pi.yaml',
    )

    default_rviz = os.path.join(
        turtlebot3_navigation_share,
        'rviz',
        'tb3_navigation2.rviz',
    )

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    auto_initial_pose_node = Node(
        package='air_clean_robot',
        executable='auto_initial_pose',
        name='auto_initial_pose',
        output='screen',
        parameters=[{
            'x': -0.037569766737571876,
            'y': 0.10523828207830883,
            'yaw': -0.007148110090032988,
            'delay_sec': 4.0,
            'publish_count': 5,
            'interval_sec': 0.5,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/home/woo/map_cleaned.yaml',
            description='Full path to the map yaml file.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 localization parameters file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz for initial pose estimation.',
        ),

        localization,
        rviz_node,
        auto_initial_pose_node,
    ])
