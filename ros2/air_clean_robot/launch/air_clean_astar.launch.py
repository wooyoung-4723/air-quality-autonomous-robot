from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('air_clean_robot')
    params_file = os.path.join(package_share, 'config', 'air_clean_params.yaml')

    # =========================
    # launch argument
    # 실행할 때 MQTT 브로커 IP를 바꾸고 싶으면:
    # ros2 launch air_clean_robot air_clean_astar.launch.py mqtt_host:=192.168.0.55
    # =========================
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

        # =====================================================
        # 1. A* + Pure Pursuit 주행 컨트롤러
        # /goal_pose, /amcl_pose, /map, /scan 등을 받아
        # /astar_path, /cmd_vel 발행
        # =====================================================
        Node(
            package='air_clean_robot',
            executable='air_clean_pure_controller',
            name='air_clean_pure_controller',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),

        # =====================================================
        # 2. 기존 Python GUI
        # 현재 GUI에서 path가 보이는 부분
        # =====================================================
        Node(
            package='air_clean_robot',
            executable='air_clean_gui',
            name='air_clean_gui',
            output='screen',
            emulate_tty=True,
            parameters=[params_file],
        ),

        # =====================================================
        # 3. MQTT 명령 브리지 + Path 브리지
        #
        # MQTT robot/1/cmd 수신
        #   → /goal_pose
        #   → /air_clean_command
        #   → /cmd_vel
        #
        # ROS2 /astar_path 수신
        #   → MQTT robot/1/path 발행
        # =====================================================
        Node(
            package='air_clean_robot',
            executable='mqtt_cmd_bridge',
            name='mqtt_cmd_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[{
                # =========================
                # MQTT 기본 설정
                # =========================
                'mqtt_host': mqtt_host,
                'mqtt_port': mqtt_port,

                # 웹/Spring Boot → ROS2 명령 수신 토픽
                'mqtt_topic': 'robot/1/cmd',

                # ROS2 /astar_path → 웹 path 표시용 MQTT 발행 토픽
                'mqtt_path_topic': 'robot/1/path',

                # =========================
                # ROS2 토픽 설정
                # =========================
                'goal_topic': '/goal_pose',
                'command_topic': '/air_clean_command',
                'cmd_vel_topic': '/cmd_vel',
                'amcl_pose_topic': '/amcl_pose',
                'path_topic': '/astar_path',

                # =========================
                # Path 전송 설정
                # =========================
                'path_max_points': 120,
                'path_min_publish_interval_sec': 0.2,
                'path_round_digits': 4,

                # =========================
                # 자동 출동 zoneA 좌표
                # =========================
                'zone_a_x': 2.352437292135011,
                'zone_a_y': -2.7041570456321136,
                'zone_a_yaw': -0.3094564395372423,

                # =========================
                # home 좌표
                # =========================
                'home_x': -0.037569766737571876,
                'home_y': 0.10523828207830883,
                'home_yaw': -0.007148110090032988,

                # =========================
                # 충전소 좌표
                # 아직 충전소 좌표 확정 전이므로 home과 동일
                # =========================
                'charge_x': -0.037569766737571876,
                'charge_y': 0.10523828207830883,
                'charge_yaw': -0.007148110090032988,

                # =========================
                # 순찰 노드 좌표
                # dashboard.html / GUI 좌표와 맞춰야 함
                # =========================
                'node1_x': 2.3018,
                'node1_y': -0.0080,
                'node1_yaw': 0.0,

                'node2_x': 2.3524,
                'node2_y': -2.7042,
                'node2_yaw': -0.3094564395372423,

                'node3_x': -0.0914,
                'node3_y': -2.5292,
                'node3_yaw': 0.0,

                # =========================
                # 순찰 도착 판정
                # =========================
                'patrol_arrival_tolerance': 0.25,
                'patrol_min_goal_time': 2.0,
            }],
        ),

        # =====================================================
        # 4. AMCL 현재 위치 MQTT 발행
        #
        # ROS2 /amcl_pose
        #   → MQTT robot/1/pose
        #   → Spring Boot
        #   → home.html 로봇 위치 표시
        # =====================================================
        Node(
            package='air_clean_robot',
            executable='amcl_pose_mqtt_publisher',
            name='amcl_pose_mqtt_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'pose_topic': '/amcl_pose',
                'mqtt_host': mqtt_host,
                'mqtt_port': mqtt_port,
                'mqtt_topic': 'robot/1/pose',
                'publish_interval_sec': 0.5,
            }],
        ),

        # =====================================================
        # 5. TurtleBot 배터리 MQTT 발행
        #
        # ROS2 /battery_state
        #   → MQTT robot/1/battery
        #   → Spring Boot
        #   → home.html 배터리 표시
        # =====================================================
        Node(
            package='air_clean_robot',
            executable='battery_state_mqtt_publisher',
            name='battery_state_mqtt_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'battery_topic': '/battery_state',
                'mqtt_host': mqtt_host,
                'mqtt_port': mqtt_port,
                'mqtt_topic': 'robot/1/battery',
                'publish_interval_sec': 1.0,
            }],
        ),

        # =====================================================
        # 6. 미세먼지 셀/히트맵 MQTT 발행
        #
        # ROS2 dust data
        #   → MQTT robot/1/dust
        #   → Spring Boot
        #   → home.html 히트맵 표시
        # =====================================================
        Node(
            package='air_clean_robot',
            executable='dust_cells_mqtt_publisher',
            name='dust_cells_mqtt_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'mqtt_host': mqtt_host,
                    'mqtt_port': mqtt_port,
                    'mqtt_topic': 'robot/1/dust',
                }
            ],
        ),
    ])
