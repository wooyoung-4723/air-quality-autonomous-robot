#!/usr/bin/env python3

import csv
import json
import math
import os
from pathlib import Path
from typing import Dict, Optional, Tuple

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


GridKey = Tuple[int, int]


class DustMapper(Node):
    """Fuse robot pose and PM10 data into map-cell averages."""

    def __init__(self):
        super().__init__('dust_mapper')

        self.declare_parameter('pm_topic', '/dust/pm')
        self.declare_parameter('pose_source', 'amcl')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_yaml', '/home/woo/map_cleaned.yaml')
        self.declare_parameter('sampling_period_sec', 1.0)
        self.declare_parameter('cell_size_m', 0.10)
        self.declare_parameter('cells_topic', '/dust/cells')
        self.declare_parameter('csv_path', '/home/woo/group4_ws/dust_logs/dust_samples.csv')

        self.pm_topic = str(self.get_parameter('pm_topic').value)
        self.pose_source = str(self.get_parameter('pose_source').value).lower()
        self.amcl_pose_topic = str(self.get_parameter('amcl_pose_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.map_yaml = str(self.get_parameter('map_yaml').value)
        self.sampling_period_sec = float(self.get_parameter('sampling_period_sec').value)
        self.cell_size_m = float(self.get_parameter('cell_size_m').value)
        self.cells_topic = str(self.get_parameter('cells_topic').value)
        self.csv_path = str(self.get_parameter('csv_path').value)

        self.map_resolution, self.map_origin = self.load_map_metadata(self.map_yaml)
        self.grid_resolution = max(self.map_resolution, self.cell_size_m)

        self.latest_pm: Optional[dict] = None
        self.latest_pose: Optional[Tuple[float, float, float]] = None
        self.cells: Dict[GridKey, dict] = {}

        self.pm_sub = self.create_subscription(String, self.pm_topic, self.pm_callback, 10)

        if self.pose_source == 'odom':
            self.pose_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
            pose_topic = self.odom_topic
        else:
            pose_qos = QoSProfile(depth=10)
            pose_qos.reliability = ReliabilityPolicy.RELIABLE
            pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self.amcl_pose_topic,
                self.amcl_pose_callback,
                pose_qos,
            )
            pose_topic = self.amcl_pose_topic

        self.cells_pub = self.create_publisher(String, self.cells_topic, 10)
        self.prepare_csv()
        self.timer = self.create_timer(self.sampling_period_sec, self.sample)

        self.get_logger().info(
            f'Dust mapper started: pm={self.pm_topic}, pose={pose_topic}, '
            f'grid_resolution={self.grid_resolution:.3f}m, csv={self.csv_path}'
        )

    @staticmethod
    def load_map_metadata(map_yaml: str):
        text = Path(map_yaml).read_text()
        resolution = 0.05
        origin = (0.0, 0.0)

        for line in text.splitlines():
            line = line.strip()
            if line.startswith('resolution:'):
                resolution = float(line.split(':', 1)[1].strip())
            elif line.startswith('origin:'):
                raw = line.split('[', 1)[1].split(']', 1)[0]
                values = [float(value.strip()) for value in raw.split(',')]
                origin = (values[0], values[1])

        return resolution, origin

    def prepare_csv(self):
        path = Path(self.csv_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.exists():
            return

        with path.open('w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'stamp_sec',
                'x',
                'y',
                'yaw',
                'cell_x',
                'cell_y',
                'pm1_0',
                'pm2_5',
                'pm10',
                'avg_pm10',
                'count',
                'mqtt_topic',
            ])

    def pm_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f'Invalid PM ROS JSON: {msg.data}')
            return

        if 'pm10' not in payload:
            self.get_logger().warning(f'PM payload has no pm10 field: {payload}')
            return

        self.latest_pm = payload

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        yaw = self.quaternion_to_yaw(pose.orientation)
        self.latest_pose = (float(pose.position.x), float(pose.position.y), yaw)

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        yaw = self.quaternion_to_yaw(pose.orientation)
        self.latest_pose = (float(pose.position.x), float(pose.position.y), yaw)

    def sample(self):
        if self.latest_pm is None or self.latest_pose is None:
            return

        x, y, yaw = self.latest_pose
        pm10 = float(self.latest_pm.get('pm10', 0.0))
        cell_x, cell_y = self.world_to_cell(x, y)
        key = (cell_x, cell_y)

        entry = self.cells.setdefault(
            key,
            {
                'sum_pm10': 0.0,
                'count': 0,
                'x': self.cell_center_x(cell_x),
                'y': self.cell_center_y(cell_y),
            },
        )
        entry['sum_pm10'] += pm10
        entry['count'] += 1
        avg_pm10 = entry['sum_pm10'] / entry['count']

        self.append_csv(x, y, yaw, cell_x, cell_y, avg_pm10, entry['count'])
        self.publish_cells()

    def append_csv(self, x, y, yaw, cell_x, cell_y, avg_pm10, count):
        stamp = self.get_clock().now().nanoseconds / 1_000_000_000.0
        with open(self.csv_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                f'{stamp:.3f}',
                f'{x:.3f}',
                f'{y:.3f}',
                f'{yaw:.3f}',
                cell_x,
                cell_y,
                self.latest_pm.get('pm1_0', ''),
                self.latest_pm.get('pm2_5', ''),
                self.latest_pm.get('pm10', ''),
                f'{avg_pm10:.3f}',
                count,
                self.latest_pm.get('mqtt_topic', ''),
            ])

    def publish_cells(self):
        cells = []
        for (cell_x, cell_y), entry in self.cells.items():
            avg = entry['sum_pm10'] / max(1, entry['count'])
            cells.append({
                'cell_x': cell_x,
                'cell_y': cell_y,
                'x': entry['x'],
                'y': entry['y'],
                'pm10': avg,
                'count': entry['count'],
            })

        msg = String()
        msg.data = json.dumps({
            'metric': 'pm10',
            'cell_size_m': self.grid_resolution,
            'map_yaml': self.map_yaml,
            'cells': cells,
        })
        self.cells_pub.publish(msg)

    def world_to_cell(self, x: float, y: float):
        cell_x = math.floor((x - self.map_origin[0]) / self.grid_resolution)
        cell_y = math.floor((y - self.map_origin[1]) / self.grid_resolution)
        return cell_x, cell_y

    def cell_center_x(self, cell_x: int):
        return self.map_origin[0] + (cell_x + 0.5) * self.grid_resolution

    def cell_center_y(self, cell_y: int):
        return self.map_origin[1] + (cell_y + 0.5) * self.grid_resolution

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = DustMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
