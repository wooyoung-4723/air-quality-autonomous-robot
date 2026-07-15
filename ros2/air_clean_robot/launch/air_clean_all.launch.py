from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnShutdown
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

    turtlebot_ip_arg = DeclareLaunchArgument(
        'turtlebot_ip',
        default_value='192.168.0.187',
        description='TurtleBot Raspberry Pi IP address'
    )

    turtlebot_user_arg = DeclareLaunchArgument(
        'turtlebot_user',
        default_value='group4',
        description='TurtleBot Raspberry Pi SSH username'
    )

    ros_domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='199',
        description='ROS_DOMAIN_ID used by PC and TurtleBot'
    )

    turtlebot_model_arg = DeclareLaunchArgument(
        'turtlebot_model',
        default_value='waffle_pi',
        description='TurtleBot3 model'
    )

    lds_model_arg = DeclareLaunchArgument(
        'lds_model',
        default_value='LDS-03',
        description='TurtleBot3 LDS model'
    )

    camera_params_arg = DeclareLaunchArgument(
        'camera_params',
        default_value='/home/group4/cam_cfg/webcam.yaml',
        description='usb_cam parameter file path on TurtleBot'
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

    turtlebot_ip = LaunchConfiguration('turtlebot_ip')
    turtlebot_user = LaunchConfiguration('turtlebot_user')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    turtlebot_model = LaunchConfiguration('turtlebot_model')
    lds_model = LaunchConfiguration('lds_model')
    camera_params = LaunchConfiguration('camera_params')
    mqtt_host = LaunchConfiguration('mqtt_host')
    mqtt_port = LaunchConfiguration('mqtt_port')

    # =====================================================
    # 중요:
    # ssh 뒤의 원격 명령은 반드시 하나의 문자열로 넘겨야 함.
    # 그래야 bash -lc "source ... && ..." 형태가 유지됨.
    # =====================================================

    remote_process_pattern = (
        '[r]os2 launch turtlebot3_bringup robot.launch.py|'
        '[u]sb_cam_node_exe|'
        '[s]ingle_coin_d4_node|'
        '[t]urtlebot3_ros|'
        '[r]obot_state_publisher'
    )

    turtlebot_bringup_remote_cmd = [
        'bash -lc "',
        'source /opt/ros/humble/setup.bash && ',
        'if [ -f ~/turtlebot3_ws/install/setup.bash ]; then ',
        'source ~/turtlebot3_ws/install/setup.bash; ',
        'fi && ',
        'export ROS_DOMAIN_ID=', ros_domain_id, ' && ',
        'export TURTLEBOT3_MODEL=', turtlebot_model, ' && ',
        'export LDS_MODEL=', lds_model, ' && ',
        'cleanup() { ',
        'pkill -INT -f \\"', remote_process_pattern, '\\" || true; ',
        'sleep 1; ',
        'pkill -TERM -f \\"', remote_process_pattern, '\\" || true; ',
        '}; ',
        'trap cleanup INT TERM EXIT; ',
        'ros2 launch turtlebot3_bringup robot.launch.py & ',
        'wait \\$!',
        '"'
    ]

    usb_cam_remote_cmd = [
        'bash -lc "',
        'source /opt/ros/humble/setup.bash && ',
        'if [ -f ~/turtlebot3_ws/install/setup.bash ]; then ',
        'source ~/turtlebot3_ws/install/setup.bash; ',
        'fi && ',
        'if [ -f ~/smart_cart_ws/install/setup.bash ]; then ',
        'source ~/smart_cart_ws/install/setup.bash; ',
        'fi && ',
        'export ROS_DOMAIN_ID=', ros_domain_id, ' && ',
        'cleanup() { ',
        'pkill -INT -f \\"[u]sb_cam_node_exe\\" || true; ',
        'sleep 1; ',
        'pkill -TERM -f \\"[u]sb_cam_node_exe\\" || true; ',
        '}; ',
        'trap cleanup INT TERM EXIT; ',
        'ros2 run usb_cam usb_cam_node_exe ',
        '--ros-args --params-file ', camera_params, ' & ',
        'wait \\$!',
        '"'
    ]

    remote_cleanup_cmd = [
        'bash -lc "',
        'pkill -INT -f \\"', remote_process_pattern, '\\" || true; ',
        'sleep 2; ',
        'pkill -TERM -f \\"', remote_process_pattern, '\\" || true',
        '"'
    ]

    return LaunchDescription([
        turtlebot_ip_arg,
        turtlebot_user_arg,
        ros_domain_id_arg,
        turtlebot_model_arg,
        lds_model_arg,
        camera_params_arg,
        mqtt_host_arg,
        mqtt_port_arg,

        # =====================================================
        # 1. TurtleBot bringup 원격 실행
        # =====================================================
        ExecuteProcess(
            cmd=[
                'ssh',
                '-tt',
                '-o',
                'StrictHostKeyChecking=accept-new',
                [turtlebot_user, '@', turtlebot_ip],
                turtlebot_bringup_remote_cmd,
            ],
            output='screen',
            name='remote_turtlebot_bringup',
            emulate_tty=True,
        ),

        # =====================================================
        # 2. usb_cam 원격 실행
        # =====================================================
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ssh',
                        '-tt',
                        '-o',
                        'StrictHostKeyChecking=accept-new',
                        [turtlebot_user, '@', turtlebot_ip],
                        usb_cam_remote_cmd,
                    ],
                    output='screen',
                    name='remote_usb_cam',
                    emulate_tty=True,
                )
            ]
        ),

        # =====================================================
        # 3. PC localization 실행
        # =====================================================
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(localization_launch)
                )
            ]
        ),

        # =====================================================
        # 4. PC A* + GUI + MQTT 전체 실행
        # =====================================================
        TimerAction(
            period=15.0,
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

        # =====================================================
        # 5. all.launch 종료 시 TurtleBot 원격 프로세스 정리
        # =====================================================
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=[
                            'ssh',
                            '-tt',
                            '-o',
                            'StrictHostKeyChecking=accept-new',
                            [turtlebot_user, '@', turtlebot_ip],
                            remote_cleanup_cmd,
                        ],
                        output='screen',
                        name='remote_turtlebot_cleanup',
                        emulate_tty=True,
                    )
                ]
            )
        ),
    ])
