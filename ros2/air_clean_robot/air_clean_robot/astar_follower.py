import heapq
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from std_msgs.msg import Bool


GridPoint = Tuple[int, int]
WorldPoint = Tuple[float, float]


@dataclass(order=True)
class NodeAStar:
    f: float
    position: GridPoint
    g: float = 0.0
    h: float = 0.0
    parent: Optional['NodeAStar'] = None


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
    if map_data[start[0], start[1]] != 0 or map_data[goal[0], goal[1]] != 0:
        return None

    start_node = NodeAStar(0.0, start, g=0.0, h=0.0, parent=None)
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

            next_position = (ny, nx)
            new_g = current.g + step_cost
            if new_g >= best_cost.get(next_position, float('inf')):
                continue

            h = math.hypot(ny - goal[0], nx - goal[1])
            best_cost[next_position] = new_g
            heapq.heappush(
                open_list,
                NodeAStar(
                    new_g + h,
                    next_position,
                    g=new_g,
                    h=h,
                    parent=current,
                ),
            )

    return None


class AStarFollower(Node):
    def __init__(self):
        super().__init__('astar_follower')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('path_topic', '/astar_path')
        self.declare_parameter('shoe_detected_topic', '/shoe_detected')
        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('inflation_radius_cells', 2)
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('linear_speed', 0.10)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('heading_gain', 1.5)
        self.declare_parameter('goal_tolerance', 0.10)
        self.declare_parameter('waypoint_tolerance', 0.12)
        self.declare_parameter('rotate_in_place_threshold', 0.35)
        self.declare_parameter('lookahead_points', 4)

        self.map_data: Optional[np.ndarray] = None
        self.inflated_map: Optional[np.ndarray] = None
        self.map_resolution = 0.0
        self.map_width = 0
        self.map_height = 0
        self.map_origin: WorldPoint = (0.0, 0.0)
        self.map_frame = 'map'

        self.current_pose: Optional[WorldPoint] = None
        self.current_yaw = 0.0
        self.goal_pose: Optional[WorldPoint] = None
        self.path_world: List[WorldPoint] = []
        self.path_grid: List[GridPoint] = []
        self.path_index = 0
        self.shoe_detected = False

        map_topic = str(self.get_parameter('map_topic').value)
        pose_topic = str(self.get_parameter('pose_topic').value)
        goal_topic = str(self.get_parameter('goal_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        shoe_detected_topic = str(self.get_parameter('shoe_detected_topic').value)

        self.occupancy_threshold = int(self.get_parameter('occupancy_threshold').value)
        self.inflation_radius_cells = int(self.get_parameter('inflation_radius_cells').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.waypoint_tolerance = float(self.get_parameter('waypoint_tolerance').value)
        self.rotate_in_place_threshold = float(
            self.get_parameter('rotate_in_place_threshold').value
        )
        self.lookahead_points = int(self.get_parameter('lookahead_points').value)
        control_frequency = float(self.get_parameter('control_frequency').value)

        self.create_subscription(OccupancyGrid, map_topic, self.map_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, pose_topic, self.pose_callback, 10)
        self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 10)
        self.create_subscription(Bool, shoe_detected_topic, self.shoe_detected_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.create_timer(1.0 / control_frequency, self.control_loop)

        self.get_logger().info('A* follower started')

    def map_callback(self, msg: OccupancyGrid):
        self.map_resolution = float(msg.info.resolution)
        self.map_width = int(msg.info.width)
        self.map_height = int(msg.info.height)
        self.map_origin = (
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
        )
        self.map_frame = msg.header.frame_id or 'map'

        grid = np.array(msg.data, dtype=np.int16).reshape((self.map_height, self.map_width))
        obstacle_mask = np.logical_or(grid < 0, grid >= self.occupancy_threshold)
        self.map_data = np.where(obstacle_mask, 1, 0).astype(np.uint8)
        self.inflated_map = self.inflate_map(self.map_data, self.inflation_radius_cells)

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

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
        )
        self.plan_path()

    def shoe_detected_callback(self, msg: Bool):
        if self.shoe_detected == msg.data:
            return

        self.shoe_detected = msg.data
        if self.shoe_detected:
            self.publish_stop()
            self.get_logger().info('Shoe detected, stopping robot')
            return

        self.get_logger().info('Shoe cleared, resuming navigation')

    def plan_path(self):
        if self.map_data is None or self.inflated_map is None or self.current_pose is None:
            self.get_logger().warn('Map or pose is not ready yet, cannot plan path')
            return
        if self.goal_pose is None:
            return

        start = self.world_to_grid(self.current_pose)
        goal = self.world_to_grid(self.goal_pose)
        if start is None or goal is None:
            self.get_logger().warn('Start or goal is outside the map')
            self.clear_path()
            return

        path_grid = run_astar(
            self.inflated_map,
            start,
            goal,
            self.map_width,
            self.map_height,
        )
        if not path_grid:
            self.get_logger().warn('A* could not find a path')
            self.clear_path()
            self.publish_stop()
            return

        self.path_grid = path_grid
        self.path_world = [self.grid_to_world(point) for point in path_grid]
        self.path_index = 0
        self.publish_path()
        self.get_logger().info(f'Planned path with {len(self.path_world)} waypoints')

    def control_loop(self):
        if self.shoe_detected:
            self.publish_stop()
            return

        if self.current_pose is None or self.goal_pose is None:
            return

        if self.distance(self.current_pose, self.goal_pose) <= self.goal_tolerance:
            self.publish_stop()
            self.clear_path(keep_goal=False)
            self.goal_pose = None
            self.get_logger().info('Goal reached')
            return

        if not self.path_world:
            return

        self.advance_path_index()
        if self.path_index >= len(self.path_world):
            self.publish_stop()
            return

        target = self.path_world[min(self.path_index + self.lookahead_points, len(self.path_world) - 1)]
        distance = self.distance(self.current_pose, target)
        heading = math.atan2(target[1] - self.current_pose[1], target[0] - self.current_pose[0])
        heading_error = self.normalize_angle(heading - self.current_yaw)

        twist = Twist()
        twist.angular.z = self.clamp(self.heading_gain * heading_error, self.max_angular_speed)
        if abs(heading_error) < self.rotate_in_place_threshold:
            twist.linear.x = min(self.linear_speed, distance)
        else:
            twist.linear.x = 0.0

        self.cmd_pub.publish(twist)

    def advance_path_index(self):
        while self.path_index < len(self.path_world):
            waypoint = self.path_world[self.path_index]
            if self.distance(self.current_pose, waypoint) > self.waypoint_tolerance:
                break
            self.path_index += 1

    def publish_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame

        for x, y in self.path_world:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

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
    node = AStarFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()
