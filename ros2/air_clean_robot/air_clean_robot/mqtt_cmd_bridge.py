#!/usr/bin/env python3

import json
import math
import threading
import time

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String


class MqttCmdBridge(Node):
    def __init__(self):
        super().__init__('mqtt_cmd_bridge')

        # =========================
        # MQTT 설정
        # =========================
        self.declare_parameter('mqtt_host', '192.168.0.55')
        self.declare_parameter('mqtt_port', 1883)

        # 웹 → ROS 명령 수신 토픽
        self.declare_parameter('mqtt_topic', 'robot/1/cmd')

        # ROS path → 웹 전송 토픽
        self.declare_parameter('mqtt_path_topic', 'robot/1/path')

        # =========================
        # ROS2 토픽 설정
        # =========================
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('command_topic', '/air_clean_command')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')

        # GUI에서 보이는 A* 경로 토픽
        self.declare_parameter('path_topic', '/astar_path')

        # =========================
        # Path 웹 전송 설정
        # =========================
        self.declare_parameter('path_max_points', 120)
        self.declare_parameter('path_min_publish_interval_sec', 0.2)
        self.declare_parameter('path_round_digits', 4)

        # =========================
        # 순찰 도착 판정 설정
        # =========================
        self.declare_parameter('patrol_arrival_tolerance', 0.25)
        self.declare_parameter('patrol_min_goal_time', 2.0)

        # =========================
        # 자동 출동 구역 zoneA 좌표
        # =========================
        self.declare_parameter('zone_a_x', 2.352437292135011)
        self.declare_parameter('zone_a_y', -2.7041570456321136)
        self.declare_parameter('zone_a_yaw', -0.3094564395372423)

        # =========================
        # home 좌표
        # =========================
        self.declare_parameter('home_x', -0.037569766737571876)
        self.declare_parameter('home_y', 0.10523828207830883)
        self.declare_parameter('home_yaw', -0.007148110090032988)

        # =========================
        # 충전소 좌표
        # 아직 충전소 좌표가 확정되지 않았으면 임시로 home과 같은 위치를 사용한다.
        # =========================
        self.declare_parameter('charge_x', -0.037569766737571876)
        self.declare_parameter('charge_y', 0.10523828207830883)
        self.declare_parameter('charge_yaw', -0.007148110090032988)

        # =========================
        # 순찰 노드 좌표
        # dashboard.html의 NODES 좌표와 맞춤
        # =========================
        self.declare_parameter('node1_x', 2.3018)
        self.declare_parameter('node1_y', -0.0080)
        self.declare_parameter('node1_yaw', 0.0)

        self.declare_parameter('node2_x', 2.3524)
        self.declare_parameter('node2_y', -2.7042)
        self.declare_parameter('node2_yaw', -0.3094564395372423)

        self.declare_parameter('node3_x', -0.0914)
        self.declare_parameter('node3_y', -2.5292)
        self.declare_parameter('node3_yaw', 0.0)

        # =========================
        # 파라미터 읽기
        # =========================
        self.mqtt_host = str(self.get_parameter('mqtt_host').value)
        self.mqtt_port = int(self.get_parameter('mqtt_port').value)
        self.mqtt_topic = str(self.get_parameter('mqtt_topic').value)
        self.mqtt_path_topic = str(self.get_parameter('mqtt_path_topic').value)

        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.command_topic = str(self.get_parameter('command_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.amcl_pose_topic = str(self.get_parameter('amcl_pose_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)

        self.path_max_points = int(self.get_parameter('path_max_points').value)
        self.path_min_publish_interval_sec = float(
            self.get_parameter('path_min_publish_interval_sec').value
        )
        self.path_round_digits = int(self.get_parameter('path_round_digits').value)

        self.patrol_arrival_tolerance = float(
            self.get_parameter('patrol_arrival_tolerance').value
        )
        self.patrol_min_goal_time = float(
            self.get_parameter('patrol_min_goal_time').value
        )

        self.zone_a = (
            float(self.get_parameter('zone_a_x').value),
            float(self.get_parameter('zone_a_y').value),
            float(self.get_parameter('zone_a_yaw').value),
        )

        self.home = (
            float(self.get_parameter('home_x').value),
            float(self.get_parameter('home_y').value),
            float(self.get_parameter('home_yaw').value),
        )

        self.charge = (
            float(self.get_parameter('charge_x').value),
            float(self.get_parameter('charge_y').value),
            float(self.get_parameter('charge_yaw').value),
        )

        self.node1 = (
            float(self.get_parameter('node1_x').value),
            float(self.get_parameter('node1_y').value),
            float(self.get_parameter('node1_yaw').value),
        )

        self.node2 = (
            float(self.get_parameter('node2_x').value),
            float(self.get_parameter('node2_y').value),
            float(self.get_parameter('node2_yaw').value),
        )

        self.node3 = (
            float(self.get_parameter('node3_x').value),
            float(self.get_parameter('node3_y').value),
            float(self.get_parameter('node3_yaw').value),
        )

        # =========================
        # 순찰 상태 변수
        # =========================
        self.current_pose = None

        self.patrol_active = False
        self.patrol_index = 0
        self.patrol_current_target = None
        self.last_goal_publish_time = 0.0

        self.patrol_route = [
            {
                'label': 'node1',
                'x': self.node1[0],
                'y': self.node1[1],
                'yaw': self.node1[2],
            },
            {
                'label': 'node2',
                'x': self.node2[0],
                'y': self.node2[1],
                'yaw': self.node2[2],
            },
            {
                'label': 'node3',
                'x': self.node3[0],
                'y': self.node3[1],
                'yaw': self.node3[2],
            },
            {
                'label': 'home',
                'x': self.home[0],
                'y': self.home[1],
                'yaw': self.home[2],
            },
        ]

        # =========================
        # Path 전송 상태 변수
        # =========================
        self.last_path_publish_time = 0.0

        # =========================
        # ROS2 Publisher
        # =========================
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.command_pub = self.create_publisher(String, self.command_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # =========================
        # ROS2 Subscriber
        # =========================
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_pose_topic,
            self.on_amcl_pose,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.on_astar_path,
            10
        )

        # =========================
        # 순찰 상태 확인 Timer
        # =========================
        self.patrol_timer = self.create_timer(0.2, self.check_patrol_progress)

        # =========================
        # MQTT Client
        # =========================
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect

        self.connect_mqtt()

        self.get_logger().info(
            f'MQTT command bridge started. '
            f'mqtt={self.mqtt_host}:{self.mqtt_port}, cmd_topic={self.mqtt_topic}, '
            f'path_mqtt_topic={self.mqtt_path_topic}, '
            f'goal_topic={self.goal_topic}, command_topic={self.command_topic}, '
            f'cmd_vel_topic={self.cmd_vel_topic}, amcl_pose_topic={self.amcl_pose_topic}, '
            f'path_topic={self.path_topic}'
        )

        self.get_logger().info(
            f'순찰 경로: node1 → node2 → node3 → home, '
            f'arrival_tolerance={self.patrol_arrival_tolerance:.2f}m'
        )

    # =========================================================
    # MQTT 연결
    # =========================================================

    def connect_mqtt(self):
        try:
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)

            mqtt_thread = threading.Thread(
                target=self.mqtt_client.loop_forever,
                daemon=True
            )
            mqtt_thread.start()

            self.get_logger().info(
                f'MQTT 연결 시도: {self.mqtt_host}:{self.mqtt_port}'
            )

        except Exception as exc:
            self.get_logger().error(f'MQTT 연결 실패: {exc}')

    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info(f'MQTT 연결 성공. 구독 토픽: {self.mqtt_topic}')
            client.subscribe(self.mqtt_topic, qos=1)

            self.publish_mqtt_json(
                'robot/1/status',
                {
                    'type': 'mqtt_cmd_bridge_status',
                    'status': 'online',
                    'cmd_topic': self.mqtt_topic,
                    'path_topic': self.mqtt_path_topic,
                }
            )
        else:
            self.get_logger().error(f'MQTT 연결 실패 rc={rc}')

    def on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warning(f'MQTT 연결 끊김 rc={rc}')

    def on_mqtt_message(self, client, userdata, message):
        try:
            command = message.payload.decode('utf-8').strip()

            self.get_logger().info(
                f'MQTT 명령 수신: topic={message.topic}, command={command}'
            )

            self.handle_command(command)

        except Exception as exc:
            self.get_logger().error(f'MQTT 명령 처리 실패: {exc}')

    def publish_mqtt_json(self, topic: str, payload: dict, qos: int = 0, retain: bool = False):
        try:
            json_payload = json.dumps(payload, ensure_ascii=False)
            result = self.mqtt_client.publish(topic, json_payload, qos=qos, retain=retain)

            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                self.get_logger().warning(f'MQTT publish 실패: topic={topic}, rc={result.rc}')

        except Exception as exc:
            self.get_logger().error(f'MQTT publish 예외 발생: topic={topic}, error={exc}')

    # =========================================================
    # AMCL 위치 수신
    # =========================================================

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose

        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)

        self.current_pose = {
            'x': x,
            'y': y,
            'yaw': yaw,
        }

    # =========================================================
    # A* Path 수신 → MQTT로 웹에 전송
    # =========================================================

    def on_astar_path(self, msg: Path):
        now = time.monotonic()
        original_count = len(msg.poses)

        if (
            original_count > 0
            and now - self.last_path_publish_time < self.path_min_publish_interval_sec
        ):
            return

        if original_count > 0:
            self.last_path_publish_time = now

        if original_count == 0:
            payload = {
                'type': 'astar_path',
                'frame_id': msg.header.frame_id,
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'point_count': 0,
                'sent_count': 0,
                'current_pose': self.make_current_pose_payload(),
                'path': [],
            }

            self.publish_mqtt_json(self.mqtt_path_topic, payload)
            return

        sampled_poses = self.downsample_poses(msg.poses, self.path_max_points)

        path_points = []

        for pose_stamped in sampled_poses:
            pose = pose_stamped.pose
            position = pose.position
            orientation = pose.orientation

            yaw = self.quaternion_to_yaw(orientation)

            path_points.append(
                {
                    'x': round(float(position.x), self.path_round_digits),
                    'y': round(float(position.y), self.path_round_digits),
                    'yaw': round(float(yaw), self.path_round_digits),
                }
            )

        payload = {
            'type': 'astar_path',
            'frame_id': msg.header.frame_id,
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'point_count': original_count,
            'sent_count': len(path_points),
            'current_pose': self.make_current_pose_payload(),
            'path': path_points,
        }

        self.publish_mqtt_json(self.mqtt_path_topic, payload)

    def make_current_pose_payload(self):
        if self.current_pose is None:
            return None

        return {
            'x': round(float(self.current_pose['x']), self.path_round_digits),
            'y': round(float(self.current_pose['y']), self.path_round_digits),
            'yaw': round(float(self.current_pose['yaw']), self.path_round_digits),
        }

    @staticmethod
    def downsample_poses(poses, max_points: int):
        if max_points <= 0:
            return poses

        total = len(poses)

        if total <= max_points:
            return poses

        step = math.ceil(total / max_points)
        sampled = poses[::step]

        if sampled[-1] != poses[-1]:
            sampled.append(poses[-1])

        return sampled

    # =========================================================
    # 명령 처리
    # =========================================================

    def handle_command(self, command: str):
        if not command:
            return

        raw_command = command.strip()
        upper_command = raw_command.upper()

        # =========================
        # 1. 기본 출동
        # START → zoneA 이동
        # =========================
        if upper_command == 'START':
            self.cancel_patrol('START 명령 수신')
            self.get_logger().info('START 명령 → zoneA 목표 발행')
            self.publish_goal(*self.zone_a, label='zoneA')
            return

        # =========================
        # 2. 구역 이동
        # GO_TO_ZONE:zoneA
        # =========================
        if upper_command.startswith('GO_TO_ZONE:'):
            self.cancel_patrol('GO_TO_ZONE 명령 수신')

            zone = upper_command.split(':', 1)[1].strip()

            if zone in ('ZONEA', 'A'):
                self.get_logger().info('GO_TO_ZONE:zoneA 명령 → zoneA 목표 발행')
                self.publish_goal(*self.zone_a, label='zoneA')
                return

            self.get_logger().warning(f'알 수 없는 zone 명령: {command}')
            return

        # =========================
        # 3. 맵 클릭 좌표 이동
        # GOTO_XY:2.35,-2.70
        # GOTO_XY:2.35,-2.70,-0.30
        # =========================
        if upper_command.startswith('GOTO_XY:'):
            self.cancel_patrol('GOTO_XY 명령 수신')
            self.handle_goto_xy(raw_command)
            return

        # =========================
        # 4. 지정 노드 이동
        # NODE1 / NODE2 / NODE3 / HOME
        # =========================
        if upper_command in ('NODE1', 'N1'):
            self.cancel_patrol('NODE1 명령 수신')
            self.get_logger().info('NODE1 명령 → node1 목표 발행')
            self.publish_goal(*self.node1, label='node1')
            return

        if upper_command in ('NODE2', 'N2'):
            self.cancel_patrol('NODE2 명령 수신')
            self.get_logger().info('NODE2 명령 → node2 목표 발행')
            self.publish_goal(*self.node2, label='node2')
            return

        if upper_command in ('NODE3', 'N3'):
            self.cancel_patrol('NODE3 명령 수신')
            self.get_logger().info('NODE3 명령 → node3 목표 발행')
            self.publish_goal(*self.node3, label='node3')
            return

        if upper_command in ('HOME', 'RETURN', 'BACK'):
            self.cancel_patrol('HOME 명령 수신')
            self.get_logger().info('HOME 명령 → home 목표 발행')
            self.publish_goal(*self.home, label='home')
            return

        # =========================
        # 5. 충전소 이동
        # CHARGE / GO_CHARGE
        # =========================
        if upper_command in ('GO_CHARGE', 'CHARGE'):
            self.cancel_patrol('GO_CHARGE 명령 수신')
            self.get_logger().info('GO_CHARGE 명령 → 충전소 목표 발행')
            self.publish_goal(*self.charge, label='charge')
            return

        # =========================
        # 6. 순찰 시작
        # PATROL → node1 → node2 → node3 → home 자동 이동
        # =========================
        if upper_command in ('PATROL', 'PATROL_START'):
            self.start_patrol()
            return

        # =========================
        # 7. 청정 ON/OFF
        # =========================
        if upper_command in ('CLEAN', 'CLEAN_ON', 'AIR_ON'):
            self.get_logger().info('CLEAN_ON 명령 → /air_clean_command clean_on 발행')
            self.publish_air_clean_command('clean_on')
            return

        if upper_command in ('CLEAN_OFF', 'AIR_OFF'):
            self.get_logger().info('CLEAN_OFF 명령 → /air_clean_command clean_off 발행')
            self.publish_air_clean_command('clean_off')
            return

        # =========================
        # 8. 일반 정지
        # STOP / X / CANCEL
        # =========================
        if upper_command in ('STOP', 'X', 'CANCEL'):
            self.cancel_patrol('STOP 명령 수신')
            self.get_logger().info('STOP 명령 → 주행 취소 + 정지')
            self.publish_air_clean_command('stop')
            self.publish_stop_repeated()
            return

        # =========================
        # 9. 대기 상태
        # =========================
        if upper_command == 'IDLE':
            self.cancel_patrol('IDLE 명령 수신')
            self.get_logger().info('IDLE 명령 → 주행 취소 + 대기')
            self.publish_air_clean_command('idle')
            self.publish_stop_repeated()
            return

        # =========================
        # 10. 비상정지
        # =========================
        if upper_command in ('ESTOP', 'E_STOP', 'EMERGENCY_STOP'):
            self.cancel_patrol('ESTOP 명령 수신')
            self.get_logger().warning('ESTOP 명령 → 비상정지')
            self.publish_air_clean_command('estop')
            self.publish_stop_repeated(repeat=10)
            return

        # =========================
        # 11. 비상정지 해제
        # =========================
        if upper_command in ('RELEASE', 'RELEASE_ESTOP', 'ESTOP_RELEASE'):
            self.get_logger().warning('RELEASE_ESTOP 명령 → 비상정지 해제')
            self.publish_air_clean_command('release_estop')
            return

        self.get_logger().warning(f'지원하지 않는 MQTT 명령: {command}')

    # =========================================================
    # 순찰 기능
    # =========================================================

    def start_patrol(self):
        if self.current_pose is None:
            self.get_logger().warning(
                'PATROL 명령 수신했지만 /amcl_pose 데이터가 아직 없습니다. '
                '그래도 node1 목표를 먼저 발행합니다.'
            )

        self.patrol_active = True
        self.patrol_index = 0
        self.patrol_current_target = None

        self.get_logger().info('PATROL 시작 → node1 → node2 → node3 → home')

        self.publish_next_patrol_goal()

    def publish_next_patrol_goal(self):
        if self.patrol_index >= len(self.patrol_route):
            self.finish_patrol()
            return

        target = self.patrol_route[self.patrol_index]
        self.patrol_current_target = target

        self.publish_goal(
            target['x'],
            target['y'],
            target['yaw'],
            label=f"patrol_{target['label']}"
        )

        self.last_goal_publish_time = time.monotonic()

        self.get_logger().info(
            f"순찰 목표 발행 [{self.patrol_index + 1}/{len(self.patrol_route)}]: "
            f"{target['label']} "
            f"x={target['x']:.3f}, y={target['y']:.3f}, yaw={target['yaw']:.3f}"
        )

    def check_patrol_progress(self):
        if not self.patrol_active:
            return

        if self.current_pose is None:
            return

        if self.patrol_current_target is None:
            return

        elapsed = time.monotonic() - self.last_goal_publish_time

        if elapsed < self.patrol_min_goal_time:
            return

        dx = self.current_pose['x'] - self.patrol_current_target['x']
        dy = self.current_pose['y'] - self.patrol_current_target['y']
        distance = math.sqrt(dx * dx + dy * dy)

        if distance <= self.patrol_arrival_tolerance:
            label = self.patrol_current_target['label']

            self.get_logger().info(
                f"순찰 목표 도착 판정: {label}, "
                f"distance={distance:.3f}m <= {self.patrol_arrival_tolerance:.3f}m"
            )

            self.patrol_index += 1

            if self.patrol_index >= len(self.patrol_route):
                self.finish_patrol()
            else:
                self.publish_next_patrol_goal()

    def finish_patrol(self):
        self.get_logger().info('PATROL 완료 → home 도착, 정지 명령 발행')

        self.patrol_active = False
        self.patrol_index = 0
        self.patrol_current_target = None

        self.publish_air_clean_command('patrol_done')
        self.publish_stop_repeated(repeat=3)

    def cancel_patrol(self, reason: str):
        if not self.patrol_active:
            return

        self.get_logger().info(f'PATROL 취소: {reason}')

        self.patrol_active = False
        self.patrol_index = 0
        self.patrol_current_target = None

    # =========================================================
    # GOTO_XY 처리
    # =========================================================

    def handle_goto_xy(self, command: str):
        try:
            payload = command.split(':', 1)[1].strip()
            parts = [p.strip() for p in payload.split(',')]

            if len(parts) < 2:
                self.get_logger().warning(
                    f'GOTO_XY 형식 오류: {command}, 예: GOTO_XY:2.35,-2.70'
                )
                return

            x = float(parts[0])
            y = float(parts[1])

            if len(parts) >= 3:
                yaw = float(parts[2])
            else:
                yaw = 0.0

            self.get_logger().info(
                f'GOTO_XY 명령 → 목표 발행 x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
            )

            self.publish_goal(x, y, yaw, label='goto_xy')

        except Exception as exc:
            self.get_logger().error(
                f'GOTO_XY 처리 실패: command={command}, error={exc}'
            )

    # =========================================================
    # ROS2 발행 함수
    # =========================================================

    def publish_goal(self, x: float, y: float, yaw: float, label: str):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.yaw_to_quaternion(yaw)

        self.goal_pub.publish(pose)

        self.get_logger().info(
            f'/goal_pose 발행: {label}, x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}'
        )

    def publish_air_clean_command(self, command: str):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)

        self.get_logger().info(f'/air_clean_command 발행: {command}')

    def publish_stop(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('/cmd_vel 0 발행')

    def publish_stop_repeated(self, repeat: int = 5, interval: float = 0.03):
        twist = Twist()

        for _ in range(repeat):
            self.cmd_vel_pub.publish(twist)
            time.sleep(interval)

        self.get_logger().info(f'/cmd_vel 0 반복 발행: {repeat}회')

    # =========================================================
    # 유틸
    # =========================================================

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    @staticmethod
    def quaternion_to_yaw(q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # =========================================================
    # 종료 처리
    # =========================================================

    def destroy_node(self):
        try:
            self.publish_mqtt_json(
                'robot/1/status',
                {
                    'type': 'mqtt_cmd_bridge_status',
                    'status': 'offline',
                }
            )
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttCmdBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
