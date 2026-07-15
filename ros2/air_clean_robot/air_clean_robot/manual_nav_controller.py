#!/usr/bin/env python3

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class ManualNavController(Node):
    """Command-topic controller for manual dirty-zone/home Nav2 goals."""

    def __init__(self):
        super().__init__('manual_nav_controller')

        self.declare_parameter('command_topic', '/air_clean_command')
        self.declare_parameter('use_nav2', True)
        self.declare_parameter('node_1_x', 1.0)
        self.declare_parameter('node_1_y', 0.0)
        self.declare_parameter('node_1_yaw', 0.0)
        self.declare_parameter('node_2_x', 0.0)
        self.declare_parameter('node_2_y', 0.0)
        self.declare_parameter('node_2_yaw', 0.0)
        self.declare_parameter('node_3_x', 0.0)
        self.declare_parameter('node_3_y', 0.0)
        self.declare_parameter('node_3_yaw', 0.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw', 0.0)
        self.declare_parameter('nav2_wait_timeout_sec', 5.0)

        self.command_topic = self.get_parameter('command_topic').value
        self.use_nav2 = bool(self.get_parameter('use_nav2').value)
        self.nodes = {
            'node 1': (
                float(self.get_parameter('node_1_x').value),
                float(self.get_parameter('node_1_y').value),
                float(self.get_parameter('node_1_yaw').value),
            ),
            'node 2': (
                float(self.get_parameter('node_2_x').value),
                float(self.get_parameter('node_2_y').value),
                float(self.get_parameter('node_2_yaw').value),
            ),
            'node 3': (
                float(self.get_parameter('node_3_x').value),
                float(self.get_parameter('node_3_y').value),
                float(self.get_parameter('node_3_yaw').value),
            ),
        }
        self.home_x = float(self.get_parameter('home_x').value)
        self.home_y = float(self.get_parameter('home_y').value)
        self.home_yaw = float(self.get_parameter('home_yaw').value)
        self.nav2_wait_timeout_sec = float(self.get_parameter('nav2_wait_timeout_sec').value)

        self.goal_in_progress = False
        self.current_goal_handle = None
        self.estop_active = False
        self.patrol_queue = []
        self.latest_pose = None  # (x, y, yaw) — auto-yaw 계산용

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cleaner_pub = self.create_publisher(Bool, '/cleaner/command', 10)
        self.subscription = self.create_subscription(
            String,
            self.command_topic,
            self.command_callback,
            10,
        )

        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            pose_qos,
        )

        self.get_logger().info(
            f'Manual nav controller ready. Publish String commands to {self.command_topic}: '
            'node1/2/3, home, start, stop, idle, go_charge, estop, release, patrol, clean'
        )

    def command_callback(self, msg):
        raw = msg.data.strip()
        command = raw.lower()

        if self.estop_active and command not in ('release',):
            self.get_logger().warning(
                f'ESTOP 활성 — "{msg.data}" 무시. 먼저 RELEASE 보내.'
            )
            return

        if command.startswith('goto_xy:'):
            self.handle_goto_xy(raw)
            return

        if command in ('1', 'node1', 'node_1', 'node 1'):
            self.patrol_queue.clear()
            self.send_node_goal('node 1')
        elif command in ('2', 'node2', 'node_2', 'node 2'):
            self.patrol_queue.clear()
            self.send_node_goal('node 2')
        elif command in ('3', 'node3', 'node_3', 'node 3'):
            self.patrol_queue.clear()
            self.send_node_goal('node 3')
        elif command in ('go', 'target', 'dirty', 'dirty_zone', 'start'):
            self.patrol_queue.clear()
            self.send_node_goal('node 1')
        elif command in ('home', 'return', 'back', 'go_charge', 'charge'):
            self.patrol_queue.clear()
            self.send_goal(self.home_x, self.home_y, self.home_yaw, 'home')
        elif command in ('stop', 'idle'):
            self.patrol_queue.clear()
            self.cancel_current_goal('stop/idle')
            self.cleaner_pub.publish(Bool(data=False))
        elif command in ('estop',):
            self.patrol_queue.clear()
            self.cancel_current_goal('estop')
            self.cmd_vel_pub.publish(Twist())
            self.cleaner_pub.publish(Bool(data=False))
            self.estop_active = True
            self.get_logger().error('🚨 ESTOP 활성. RELEASE 명령으로 해제 가능.')
        elif command in ('release',):
            self.estop_active = False
            self.get_logger().info('ESTOP 해제됨')
        elif command in ('patrol',):
            self.start_patrol()
        elif command in ('clean',):
            self.cleaner_pub.publish(Bool(data=True))
            self.get_logger().info('청정기 ON 명령 발행 (/cleaner/command=true)')
        else:
            self.get_logger().warning(
                f'Unknown command "{msg.data}". '
                'Use: node1/2/3, home, start, stop, idle, go_charge, estop, release, patrol, clean.'
            )

    def pose_callback(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.latest_pose = (float(p.position.x), float(p.position.y), yaw)

    def handle_goto_xy(self, raw_text):
        """parse 'GOTO_XY:x,y[,yaw]' (대소문자 무관, 콜론 뒤 인자만 사용)."""
        try:
            body = raw_text.split(':', 1)[1]
            parts = [p.strip() for p in body.split(',')]
            x = float(parts[0])
            y = float(parts[1])
            yaw = float(parts[2]) if len(parts) >= 3 else None
        except (IndexError, ValueError) as exc:
            self.get_logger().warning(f'GOTO_XY 파싱 실패: "{raw_text}" ({exc})')
            return

        if yaw is None:
            yaw = self.compute_auto_yaw(x, y)

        self.patrol_queue.clear()
        self.send_goal(x, y, yaw, f'goto_xy({x:.2f},{y:.2f})')

    def compute_auto_yaw(self, target_x, target_y):
        """출발→도착 벡터의 yaw. amcl_pose 못 받았으면 0 반환."""
        if self.latest_pose is None:
            self.get_logger().warning(
                'amcl_pose 미수신 — auto-yaw 0.0 으로 진행'
            )
            return 0.0
        cx, cy, current_yaw = self.latest_pose
        dx = target_x - cx
        dy = target_y - cy
        if abs(dx) < 1e-3 and abs(dy) < 1e-3:
            return current_yaw
        return math.atan2(dy, dx)

    def cancel_current_goal(self, reason):
        if self.current_goal_handle is None:
            self.get_logger().info(f'cancel({reason}): 진행 중 goal 없음')
            return
        self.get_logger().info(f'goal 취소 요청: {reason}')
        try:
            self.current_goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().warning(f'goal 취소 실패: {exc}')
        self.current_goal_handle = None
        self.goal_in_progress = False

    def start_patrol(self):
        self.patrol_queue = ['node 1', 'node 2', 'node 3', 'home']
        self.get_logger().info('순찰 시작: node 1 → 2 → 3 → home')
        self.advance_patrol()

    def advance_patrol(self):
        if not self.patrol_queue:
            self.get_logger().info('순찰 완료')
            return
        target = self.patrol_queue.pop(0)
        if target == 'home':
            self.send_goal(self.home_x, self.home_y, self.home_yaw, 'home (patrol)')
        else:
            self.send_node_goal(target)

    def send_node_goal(self, label):
        x, y, yaw = self.nodes[label]
        self.send_goal(x, y, yaw, label)

    def send_goal(self, x, y, yaw, label):
        if self.goal_in_progress:
            self.get_logger().warning(f'Navigation goal already in progress. Ignoring {label} command.')
            return

        if not self.use_nav2:
            self.get_logger().info(
                f'use_nav2=false: simulated {label} goal x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
            )
            return

        self.get_logger().info(f'Waiting for Nav2 action server before {label} goal...')
        if not self.nav_client.wait_for_server(timeout_sec=self.nav2_wait_timeout_sec):
            self.get_logger().error(
                'Nav2 NavigateToPose action server is not available. '
                'Start TurtleBot3 Nav2 and localization first.'
            )
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(x, y, yaw)

        self.goal_in_progress = True
        self.get_logger().info(f'Sending {label} goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda send_future: self.goal_response_callback(send_future, label))

    def goal_response_callback(self, future, label):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_in_progress = False
            self.current_goal_handle = None
            self.patrol_queue.clear()
            self.get_logger().error(f'Nav2 rejected {label} goal.')
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info(f'Nav2 accepted {label} goal.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done_future: self.goal_result_callback(done_future, label))

    def goal_result_callback(self, future, label):
        self.goal_in_progress = False
        self.current_goal_handle = None
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'{label} goal succeeded.')
            if self.patrol_queue:
                self.advance_patrol()
        else:
            self.get_logger().error(f'{label} goal failed with status={result.status}.')
            self.patrol_queue.clear()

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.yaw_to_quaternion(yaw)
        return pose

    @staticmethod
    def yaw_to_quaternion(yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = ManualNavController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
