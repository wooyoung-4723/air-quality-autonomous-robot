#!/usr/bin/env python3

import json
import math
import time
from collections import defaultdict, deque
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class State(Enum):
    IDLE = 'IDLE'
    GO_TO_DIRTY_ZONE = 'GO_TO_DIRTY_ZONE'
    CLEANING = 'CLEANING'
    RETURN_HOME = 'RETURN_HOME'
    ERROR = 'ERROR'


class AirQualityTaskManager(Node):
    """FSM that consumes /air_quality JSON and commands Nav2 when needed."""

    def __init__(self):
        super().__init__('air_quality_task_manager')

        self.declare_parameter('air_quality_topic', '/air_quality')
        self.declare_parameter('target_zone', 'A')
        self.declare_parameter('pm25_high_threshold', 35.0)
        self.declare_parameter('pm25_low_threshold', 25.0)
        self.declare_parameter('window_size', 5)
        self.declare_parameter('cleaning_duration_sec', 30.0)
        self.declare_parameter('use_nav2', True)
        self.declare_parameter('dirty_zone_x', 1.0)
        self.declare_parameter('dirty_zone_y', 0.0)
        self.declare_parameter('dirty_zone_yaw', 0.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw', 0.0)
        self.declare_parameter('nav2_wait_timeout_sec', 5.0)

        self.air_quality_topic = self.get_parameter('air_quality_topic').value
        self.target_zone = self.get_parameter('target_zone').value
        self.pm25_high_threshold = float(self.get_parameter('pm25_high_threshold').value)
        self.pm25_low_threshold = float(self.get_parameter('pm25_low_threshold').value)
        self.window_size = int(self.get_parameter('window_size').value)
        self.cleaning_duration_sec = float(self.get_parameter('cleaning_duration_sec').value)
        self.use_nav2 = bool(self.get_parameter('use_nav2').value)
        self.dirty_zone_x = float(self.get_parameter('dirty_zone_x').value)
        self.dirty_zone_y = float(self.get_parameter('dirty_zone_y').value)
        self.dirty_zone_yaw = float(self.get_parameter('dirty_zone_yaw').value)
        self.home_x = float(self.get_parameter('home_x').value)
        self.home_y = float(self.get_parameter('home_y').value)
        self.home_yaw = float(self.get_parameter('home_yaw').value)
        self.nav2_wait_timeout_sec = float(self.get_parameter('nav2_wait_timeout_sec').value)

        self.pm25_windows = defaultdict(lambda: deque(maxlen=self.window_size))
        self.state = State.IDLE
        self.goal_in_progress = False
        self.cleaning_start_time = None

        self.subscription = self.create_subscription(
            String,
            self.air_quality_topic,
            self.air_quality_callback,
            10,
        )
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(1.0, self.fsm_tick)

        self.get_logger().info(
            f'Task manager started: topic={self.air_quality_topic}, target_zone={self.target_zone}, '
            f'high={self.pm25_high_threshold}, low={self.pm25_low_threshold}, '
            f'window_size={self.window_size}, use_nav2={self.use_nav2}'
        )

    def air_quality_callback(self, msg):
        try:
            payload = json.loads(msg.data)
            zone = str(payload['zone'])
            pm25 = float(payload['pm2_5'])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f'Ignoring invalid /air_quality payload: {msg.data} ({exc})')
            return

        self.pm25_windows[zone].append(pm25)
        avg = self.get_pm25_average(zone)
        self.get_logger().info(
            f'Air quality update: zone={zone}, pm2_5={pm25:.1f}, '
            f'rolling_avg={avg:.1f}, samples={len(self.pm25_windows[zone])}'
        )

    def fsm_tick(self):
        if self.state == State.IDLE:
            self.handle_idle()
        elif self.state == State.CLEANING:
            self.handle_cleaning()
        elif self.state == State.ERROR:
            self.get_logger().error('FSM is in ERROR state. No new goals will be sent.')

    def handle_idle(self):
        avg = self.get_pm25_average(self.target_zone)
        if avg is None:
            return

        if avg >= self.pm25_high_threshold:
            self.get_logger().warning(
                f'PM2.5 rolling average {avg:.1f} >= {self.pm25_high_threshold:.1f}. '
                'Dispatching robot to dirty zone.'
            )
            self.transition_to(State.GO_TO_DIRTY_ZONE)
            self.send_navigation_goal(
                self.dirty_zone_x,
                self.dirty_zone_y,
                self.dirty_zone_yaw,
                on_success_state=State.CLEANING,
                on_failure_state=State.ERROR,
                label='dirty zone',
            )

    def handle_cleaning(self):
        if self.cleaning_start_time is None:
            self.cleaning_start_time = time.monotonic()
            self.get_logger().info('CLEANING START')

        elapsed = time.monotonic() - self.cleaning_start_time
        if elapsed < self.cleaning_duration_sec:
            self.get_logger().info(f'CLEANING... {elapsed:.1f}/{self.cleaning_duration_sec:.1f}s')
            return

        self.get_logger().info('Cleaning duration finished. Returning home.')
        self.cleaning_start_time = None
        self.transition_to(State.RETURN_HOME)
        self.send_navigation_goal(
            self.home_x,
            self.home_y,
            self.home_yaw,
            on_success_state=State.IDLE,
            on_failure_state=State.ERROR,
            label='home',
        )

    def send_navigation_goal(self, x, y, yaw, on_success_state, on_failure_state, label):
        if self.goal_in_progress:
            self.get_logger().warning(f'Navigation goal already in progress. Skipping {label} goal.')
            return

        if not self.use_nav2:
            self.get_logger().info(
                f'use_nav2=false: simulated navigation to {label} '
                f'(x={x:.2f}, y={y:.2f}, yaw={yaw:.2f})'
            )
            if on_success_state == State.CLEANING:
                self.cleaning_start_time = None
            self.transition_to(on_success_state)
            return

        self.get_logger().info(f'Waiting for Nav2 action server before sending {label} goal...')
        if not self.nav_client.wait_for_server(timeout_sec=self.nav2_wait_timeout_sec):
            self.get_logger().error(
                'Nav2 NavigateToPose action server is not available. '
                'Start TurtleBot3 Nav2 before running with use_nav2=true.'
            )
            self.transition_to(on_failure_state)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(x, y, yaw)

        self.goal_in_progress = True
        self.get_logger().info(f'Sending Nav2 goal to {label}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda send_future: self.goal_response_callback(
                send_future, on_success_state, on_failure_state, label
            )
        )

    def goal_response_callback(self, future, on_success_state, on_failure_state, label):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_in_progress = False
            self.get_logger().error(f'Nav2 rejected {label} goal.')
            self.transition_to(on_failure_state)
            return

        self.get_logger().info(f'Nav2 accepted {label} goal.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future: self.goal_result_callback(
                done_future, on_success_state, on_failure_state, label
            )
        )

    def goal_result_callback(self, future, on_success_state, on_failure_state, label):
        self.goal_in_progress = False
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Nav2 {label} goal succeeded.')
            if on_success_state == State.CLEANING:
                self.cleaning_start_time = None
            self.transition_to(on_success_state)
        else:
            self.get_logger().error(f'Nav2 {label} goal failed with status={result.status}.')
            self.transition_to(on_failure_state)

    def transition_to(self, new_state):
        if self.state == new_state:
            return
        self.get_logger().info(f'FSM transition: {self.state.value} -> {new_state.value}')
        self.state = new_state

    def get_pm25_average(self, zone):
        values = self.pm25_windows.get(zone)
        if not values:
            return None
        return sum(values) / len(values)

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
    node = AirQualityTaskManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
