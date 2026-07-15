#!/usr/bin/env python3

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


GridPoint = Tuple[int, int]
WorldPoint = Tuple[float, float]


@dataclass(order=True)
class NodeAStar:
    f: float
    h: float
    position: GridPoint = field(compare=False)
    g: float = field(default=0.0, compare=False)
    parent: Optional['NodeAStar'] = field(default=None, compare=False)


def run_astar(
    map_data: np.ndarray,
    start: GridPoint,
    goal: GridPoint,
    width: int,
    height: int,
) -> Optional[List[GridPoint]]:
    if not (0 <= start[0] < height and 0 <= start[1] < width):
        return None

    if not (0 <= goal[0] < height and 0 <= goal[1] < width):
        return None

    if map_data[start[0], start[1]] != 0:
        return None

    if map_data[goal[0], goal[1]] != 0:
        return None

    start_h = math.hypot(start[0] - goal[0], start[1] - goal[1])
    start_node = NodeAStar(
        f=start_h,
        h=start_h,
        position=start,
        g=0.0,
        parent=None,
    )

    open_list: List[NodeAStar] = [start_node]
    best_cost = {start: 0.0}
    visited = set()

    moves = [
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (-1, -1, math.sqrt(2.0)),
    ]

    while open_list:
        current = heapq.heappop(open_list)

        if current.position in visited:
            continue

        visited.add(current.position)

        if current.position == goal:
            path: List[GridPoint] = []
            node: Optional[NodeAStar] = current

            while node is not None:
                path.append(node.position)
                node = node.parent

            return list(reversed(path))

        cy, cx = current.position

        for dy, dx, step_cost in moves:
            ny = cy + dy
            nx = cx + dx

            if not (0 <= ny < height and 0 <= nx < width):
                continue

            if map_data[ny, nx] != 0:
                continue

            # 대각선 이동 시 벽 모서리를 뚫고 지나가는 문제 방지
            if dy != 0 and dx != 0:
                if map_data[cy, nx] != 0 or map_data[ny, cx] != 0:
                    continue

            next_position = (ny, nx)
            new_g = current.g + step_cost

            if new_g >= best_cost.get(next_position, float('inf')):
                continue

            h = math.hypot(ny - goal[0], nx - goal[1])
            best_cost[next_position] = new_g

            heapq.heappush(
                open_list,
                NodeAStar(
                    f=new_g + h,
                    h=h,
                    position=next_position,
                    g=new_g,
                    parent=current,
                ),
            )

    return None


class AirCleanAStarController(Node):
    def __init__(self):
        super().__init__('air_clean_astar_controller')

        self.declare_parameter('command_topic', '/air_clean_command')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('path_topic', '/astar_path')
        self.declare_parameter('shoe_detected_topic', '/shoe_detected')
        self.declare_parameter('scan_topic', '/scan')

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

        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('inflation_radius_cells', 2)
        self.declare_parameter('use_scan_obstacles', True)
        self.declare_parameter('dynamic_obstacle_inflation_cells', 1)
        self.declare_parameter('dynamic_obstacle_max_range', 1.2)
        self.declare_parameter('dynamic_replan_interval_sec', 0.5)
        self.declare_parameter('replan_retry_interval_sec', 1.0)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('heading_gain', 1.5)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('waypoint_tolerance', 0.12)
        self.declare_parameter('rotate_in_place_threshold', 0.35)
        self.declare_parameter('lookahead_points', 4)
        self.declare_parameter('lookahead_distance', 0.20)
        self.declare_parameter('safety_cost_radius_cells', 6)
        self.declare_parameter('safety_cost_weight', 2.0)

        self.command_topic = str(self.get_parameter('command_topic').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.shoe_detected_topic = str(self.get_parameter('shoe_detected_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)

        self.targets = {
            'node 1': (
                float(self.get_parameter('node_1_x').value),
                float(self.get_parameter('node_1_y').value),
            ),
            'node 2': (
                float(self.get_parameter('node_2_x').value),
                float(self.get_parameter('node_2_y').value),
            ),
            'node 3': (
                float(self.get_parameter('node_3_x').value),
                float(self.get_parameter('node_3_y').value),
            ),
            'home': (
                float(self.get_parameter('home_x').value),
                float(self.get_parameter('home_y').value),
            ),
        }

        self.occupancy_threshold = int(self.get_parameter('occupancy_threshold').value)
        self.inflation_radius_cells = int(self.get_parameter('inflation_radius_cells').value)
        self.use_scan_obstacles = bool(self.get_parameter('use_scan_obstacles').value)
        self.dynamic_obstacle_inflation_cells = int(
            self.get_parameter('dynamic_obstacle_inflation_cells').value
        )
        self.dynamic_obstacle_max_range = float(
            self.get_parameter('dynamic_obstacle_max_range').value
        )
        self.dynamic_replan_interval_sec = float(
            self.get_parameter('dynamic_replan_interval_sec').value
        )
        self.replan_retry_interval_sec = float(
            self.get_parameter('replan_retry_interval_sec').value
        )
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.rotate_in_place_threshold = float(
            self.get_parameter('rotate_in_place_threshold').value
        )
        self.lookahead_points = int(self.get_parameter('lookahead_points').value)
        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.safety_cost_radius_cells = int(
            self.get_parameter('safety_cost_radius_cells').value
        )
        self.safety_cost_weight = float(self.get_parameter('safety_cost_weight').value)
        self.control_frequency = float(self.get_parameter('control_frequency').value)

        self.map_data: Optional[np.ndarray] = None
        self.inflated_map: Optional[np.ndarray] = None
        self.safety_cost_map: Optional[np.ndarray] = None
        self.dynamic_obstacle_map: Optional[np.ndarray] = None
        self.map_resolution = 0.0
        self.map_width = 0
        self.map_height = 0
        self.map_origin: WorldPoint = (0.0, 0.0)
        self.map_frame = 'map'

        self.current_pose: Optional[WorldPoint] = None
        self.current_yaw = 0.0

        self.goal_pose: Optional[WorldPoint] = None
        self.goal_label = ''
        self.need_replan = False

        self.path_world: List[WorldPoint] = []
        self.path_grid: List[GridPoint] = []
        self.path_index = 0
        self.last_dynamic_replan_time = 0.0
        self.next_replan_time = 0.0
        self.last_control_log_time = 0.0

        self.shoe_detected = False

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos,
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_callback,
            pose_qos,
        )

        self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10,
        )

        self.create_subscription(
            String,
            self.command_topic,
            self.command_callback,
            10,
        )

        self.create_subscription(
            Bool,
            self.shoe_detected_topic,
            self.shoe_detected_callback,
            10,
        )

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        self.create_timer(1.0 / self.control_frequency, self.control_loop)

        self.get_logger().info(
            f'AirClean A* controller started. '
            f'Command topic={self.command_topic}, '
            f'map={self.map_topic}, pose={self.pose_topic}, '
            f'scan={self.scan_topic}, cmd_vel={self.cmd_vel_topic}, '
            f'path={self.path_topic}'
        )

        self.get_logger().info(
            'Available commands: node1, node2, node3, home, stop'
        )

    def command_callback(self, msg: String):
        command = msg.data.strip().lower()

        if command in ('1', 'node1', 'node_1', 'node 1'):
            self.set_goal_from_target('node 1')

        elif command in ('2', 'node2', 'node_2', 'node 2'):
            self.set_goal_from_target('node 2')

        elif command in ('3', 'node3', 'node_3', 'node 3'):
            self.set_goal_from_target('node 3')

        elif command in ('go', 'target', 'dirty', 'dirty_zone'):
            self.set_goal_from_target('node 1')

        elif command in ('home', 'return', 'back'):
            self.set_goal_from_target('home')

        elif command in ('stop', 'x', 'cancel'):
            self.cancel_navigation()

        else:
            self.get_logger().warning(
                f'Unknown command "{msg.data}". Use one of: node1, node2, node3, home, stop.'
            )

    def set_goal_from_target(self, label: str):
        if label not in self.targets:
            self.get_logger().warning(f'Unknown target label: {label}')
            return

        x, y = self.targets[label]
        self.goal_pose = (x, y)
        self.goal_label = label
        self.need_replan = True
        self.clear_path(keep_goal=True)

        self.get_logger().info(
            f'Received command for {label}. Goal set to x={x:.2f}, y={y:.2f}'
        )

        self.plan_path()

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
        )
        self.goal_label = 'external goal'
        self.need_replan = True
        self.clear_path(keep_goal=True)

        self.get_logger().info(
            f'Received /goal_pose. Goal set to x={self.goal_pose[0]:.2f}, y={self.goal_pose[1]:.2f}'
        )

        self.plan_path()

    def map_callback(self, msg: OccupancyGrid):
        self.map_resolution = float(msg.info.resolution)
        self.map_width = int(msg.info.width)
        self.map_height = int(msg.info.height)
        self.map_origin = (
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
        )
        self.map_frame = msg.header.frame_id or 'map'

        grid = np.array(msg.data, dtype=np.int16).reshape(
            (self.map_height, self.map_width)
        )

        obstacle_mask = np.logical_or(
            grid < 0,
            grid >= self.occupancy_threshold,
        )

        self.map_data = np.where(obstacle_mask, 1, 0).astype(np.uint8)
        self.inflated_map = self.inflate_map(
            self.map_data,
            self.inflation_radius_cells,
        )
        self.safety_cost_map = self.build_safety_cost_map(self.map_data)
        self.dynamic_obstacle_map = np.zeros_like(self.map_data, dtype=np.uint8)

        self.get_logger().info(
            f'Map received: {self.map_width}x{self.map_height}, '
            f'resolution={self.map_resolution:.3f}, '
            f'inflation_radius_cells={self.inflation_radius_cells}'
        )

        if self.goal_pose is not None and self.need_replan:
            self.plan_path()

    def scan_callback(self, msg: LaserScan):
        if not self.use_scan_obstacles:
            return

        if self.map_data is None or self.current_pose is None:
            return

        dynamic_map = np.zeros_like(self.map_data, dtype=np.uint8)
        angle = float(msg.angle_min)
        range_max = float(msg.range_max) if math.isfinite(msg.range_max) else float('inf')
        usable_max_range = min(range_max * 0.95, self.dynamic_obstacle_max_range)

        for distance in msg.ranges:
            if math.isfinite(distance):
                distance = float(distance)

                if msg.range_min <= distance <= usable_max_range:
                    obstacle_angle = self.current_yaw + angle
                    obstacle = (
                        self.current_pose[0] + distance * math.cos(obstacle_angle),
                        self.current_pose[1] + distance * math.sin(obstacle_angle),
                    )
                    cell = self.world_to_grid(obstacle)

                    if cell is not None:
                        dynamic_map[cell[0], cell[1]] = 1

            angle += float(msg.angle_increment)

        self.dynamic_obstacle_map = self.inflate_map(
            dynamic_map,
            self.dynamic_obstacle_inflation_cells,
        )

        if self.goal_pose is not None and self.path_world:
            now = self.get_clock().now().nanoseconds / 1_000_000_000.0

            if now - self.last_dynamic_replan_time >= self.dynamic_replan_interval_sec:
                self.last_dynamic_replan_time = now
                self.need_replan = True

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        if self.goal_pose is not None and self.need_replan:
            self.plan_path()

    def shoe_detected_callback(self, msg: Bool):
        if self.shoe_detected == msg.data:
            return

        self.shoe_detected = bool(msg.data)

        if self.shoe_detected:
            self.publish_stop()
            self.get_logger().warning('Shoe detected. Stopping robot.')
            return

        self.get_logger().info('Shoe cleared. Resuming navigation.')

    def plan_path(self):
        if self.goal_pose is None:
            return

        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if now < self.next_replan_time:
            return

        if self.map_data is None or self.inflated_map is None:
            self.get_logger().warn('Map is not ready yet. Cannot plan path.')
            self.next_replan_time = now + self.replan_retry_interval_sec
            return

        if self.current_pose is None:
            self.get_logger().warn('Robot pose is not ready yet. Cannot plan path.')
            self.next_replan_time = now + self.replan_retry_interval_sec
            return

        start = self.world_to_grid(self.current_pose)
        goal = self.world_to_grid(self.goal_pose)

        if start is None:
            self.get_logger().warn('Start pose is outside the map.')
            self.clear_path(keep_goal=True)
            self.next_replan_time = now + self.replan_retry_interval_sec
            return

        if goal is None:
            self.get_logger().warn('Goal pose is outside the map.')
            self.clear_path(keep_goal=True)
            self.next_replan_time = now + self.replan_retry_interval_sec
            return

        planning_map = self.get_planning_map()

        path_grid = run_astar(
            planning_map,
            start,
            goal,
            self.map_width,
            self.map_height,
        )

        if not path_grid:
            self.get_logger().warn(
                f'A* could not find a path to {self.goal_label}. '
                f'start={start}, goal={goal}'
            )
            self.clear_path(keep_goal=True)
            self.publish_stop()
            self.need_replan = True
            self.next_replan_time = now + self.replan_retry_interval_sec
            return

        self.path_grid = path_grid
        self.path_world = [self.grid_to_world(point) for point in path_grid]
        self.path_index = 0
        self.need_replan = False
        self.next_replan_time = 0.0

        self.publish_path()

        self.get_logger().info(
            f'A* path planned to {self.goal_label}: '
            f'{len(self.path_world)} waypoints'
        )

    def control_loop(self):
        if self.shoe_detected:
            self.publish_stop()
            return

        if self.goal_pose is None:
            return

        if self.current_pose is None:
            return

        if self.need_replan:
            self.plan_path()
            return

        goal_distance = self.distance(self.current_pose, self.goal_pose)
        if goal_distance <= self.goal_tolerance:
            self.publish_stop()
            self.get_logger().info(f'Goal reached: {self.goal_label}')
            self.clear_path(keep_goal=False)
            self.goal_pose = None
            self.goal_label = ''
            return

        if not self.path_world:
            return

        self.advance_path_index()

        if self.path_index >= len(self.path_world):
            self.publish_stop()
            self.need_replan = True
            return

        target = self.select_tracking_target()

        distance_to_target = self.distance(self.current_pose, target)
        heading = math.atan2(
            target[1] - self.current_pose[1],
            target[0] - self.current_pose[0],
        )

        heading_error = self.normalize_angle(heading - self.current_yaw)

        twist = Twist()
        twist.angular.z = self.clamp(
            self.heading_gain * heading_error,
            self.max_angular_speed,
        )

        if abs(heading_error) < self.rotate_in_place_threshold:
            twist.linear.x = min(self.linear_speed, distance_to_target)
        else:
            twist.linear.x = 0.0

        self.cmd_pub.publish(twist)
        self.log_control_state(
            target,
            distance_to_target,
            heading,
            heading_error,
            twist,
            goal_distance,
        )

    def advance_path_index(self):
        while self.path_index < len(self.path_world):
            waypoint = self.path_world[self.path_index]

            if self.distance(self.current_pose, waypoint) > self.waypoint_tolerance:
                break

            self.path_index += 1

    def select_tracking_target(self) -> WorldPoint:
        last_index = len(self.path_world) - 1

        if self.distance(self.current_pose, self.goal_pose) <= self.goal_tolerance:
            return self.goal_pose

        minimum_index = min(
            self.path_index + max(0, self.lookahead_points),
            last_index,
        )

        for index in range(max(0, self.path_index), len(self.path_world)):
            waypoint = self.path_world[index]

            if self.distance(self.current_pose, waypoint) >= self.lookahead_distance:
                return self.path_world[max(index, minimum_index)]

        return self.path_world[last_index]

    def log_control_state(
        self,
        target: WorldPoint,
        distance_to_target: float,
        heading: float,
        heading_error: float,
        twist: Twist,
        goal_distance: float,
    ):
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0

        if now - self.last_control_log_time < 1.0:
            return

        self.last_control_log_time = now
        self.get_logger().info(
            f'control pose=({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}), '
            f'yaw={self.current_yaw:.2f}, path_index={self.path_index}, '
            f'target=({target[0]:.2f}, {target[1]:.2f}), '
            f'distance_to_target={distance_to_target:.2f}, '
            f'heading={heading:.2f}, heading_error={heading_error:.2f}, '
            f'cmd=({twist.linear.x:.2f}, {twist.angular.z:.2f}), '
            f'goal_distance={goal_distance:.2f}'
        )

    def publish_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame

        for x, y in self.path_world:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def cancel_navigation(self):
        self.publish_stop()
        self.clear_path(keep_goal=False)
        self.goal_pose = None
        self.goal_label = ''
        self.need_replan = False
        self.get_logger().info('Navigation canceled by stop command.')

    def clear_path(self, keep_goal: bool = True):
        self.path_world = []
        self.path_grid = []
        self.path_index = 0

        if not keep_goal:
            self.goal_pose = None

        self.publish_path()

    def world_to_grid(self, position: WorldPoint) -> Optional[GridPoint]:
        if self.map_resolution <= 0.0:
            return None

        mx = math.floor((position[0] - self.map_origin[0]) / self.map_resolution)
        my = math.floor((position[1] - self.map_origin[1]) / self.map_resolution)

        if 0 <= my < self.map_height and 0 <= mx < self.map_width:
            return my, mx

        return None

    def get_planning_map(self) -> np.ndarray:
        if self.inflated_map is None:
            raise RuntimeError('Inflated map is not ready')

        if not self.use_scan_obstacles or self.dynamic_obstacle_map is None:
            return self.inflated_map

        return np.maximum(self.inflated_map, self.dynamic_obstacle_map)

    def build_safety_cost_map(self, map_data: np.ndarray) -> np.ndarray:
        # TODO: Use this hook for cost-aware A* so cells near walls receive
        # additional traversal cost instead of only using binary inflation.
        # Intended parameters: safety_cost_radius_cells, safety_cost_weight.
        return np.zeros_like(map_data, dtype=np.float32)

    def grid_to_world(self, cell: GridPoint) -> WorldPoint:
        y, x = cell

        wx = self.map_origin[0] + (x + 0.5) * self.map_resolution
        wy = self.map_origin[1] + (y + 0.5) * self.map_resolution

        return wx, wy

    @staticmethod
    def inflate_map(map_data: np.ndarray, radius: int) -> np.ndarray:
        if radius <= 0:
            return map_data.copy()

        inflated = map_data.copy()
        obstacle_points = np.argwhere(map_data != 0)

        for y, x in obstacle_points:
            y_min = max(0, y - radius)
            y_max = min(map_data.shape[0], y + radius + 1)
            x_min = max(0, x - radius)
            x_max = min(map_data.shape[1], x + radius + 1)

            inflated[y_min:y_max, x_min:x_max] = 1

        return inflated

    @staticmethod
    def distance(a: Sequence[float], b: Sequence[float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    @staticmethod
    def clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))


def main(args=None):
    rclpy.init(args=args)
    node = AirCleanAStarController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
