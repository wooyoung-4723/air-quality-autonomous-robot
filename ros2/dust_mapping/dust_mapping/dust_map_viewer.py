#!/usr/bin/env python3

import json
import math
from pathlib import Path
import tkinter as tk
from tkinter import ttk

from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from PIL import Image, ImageTk
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class DustMapViewer(Node):
    """Tkinter dust heatmap viewer using a static map image plus ROS cell data."""

    PM10_COLOR_STOPS = [
        (0.0, (30, 136, 229)),
        (5.0, (0, 188, 212)),
        (10.0, (67, 160, 71)),
        (15.0, (253, 216, 53)),
        (20.0, (251, 140, 0)),
        (25.0, (239, 83, 80)),
        (30.0, (183, 28, 28)),
    ]

    def __init__(self):
        super().__init__('dust_map_viewer')

        self.declare_parameter('map_yaml', '/home/woo/map_cleaned.yaml')
        self.declare_parameter('cells_topic', '/dust/cells')
        self.declare_parameter('pm_topic', '/dust/pm')
        self.declare_parameter('robot_pose_topic', '/amcl_pose')
        self.declare_parameter('window_title', 'Dust Map Viewer')
        self.declare_parameter('map_max_pixels', 720)
        self.declare_parameter('heatmap_radius_m', 0.35)
        self.declare_parameter('heatmap_alpha', 0.62)
        self.declare_parameter('free_space_threshold', 245)

        self.map_yaml = str(self.get_parameter('map_yaml').value)
        self.cells_topic = str(self.get_parameter('cells_topic').value)
        self.pm_topic = str(self.get_parameter('pm_topic').value)
        self.robot_pose_topic = str(self.get_parameter('robot_pose_topic').value)
        self.window_title = str(self.get_parameter('window_title').value)
        self.map_max_pixels = int(self.get_parameter('map_max_pixels').value)
        self.heatmap_radius_m = float(self.get_parameter('heatmap_radius_m').value)
        self.heatmap_alpha = float(self.get_parameter('heatmap_alpha').value)
        self.free_space_threshold = int(self.get_parameter('free_space_threshold').value)

        self.map_info = self.load_map(self.map_yaml)
        self.cells = []
        self.cell_size_m = 0.10
        self.map_photo = None
        self.heatmap_photo = None
        self.free_space_mask = None
        self.robot_pose = None
        self.latest_pm10 = None

        self.cells_sub = self.create_subscription(
            String,
            self.cells_topic,
            self.cells_callback,
            10,
        )
        self.pm_sub = self.create_subscription(
            String,
            self.pm_topic,
            self.pm_callback,
            10,
        )
        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.robot_pose_topic,
            self.robot_pose_callback,
            pose_qos,
        )

        self.root = tk.Tk()
        self.root.title(self.window_title)
        self.root.geometry('900x760')
        self.root.minsize(760, 620)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.status_var = tk.StringVar(value=f'Waiting for {self.cells_topic}')
        self.pose_status_var = tk.StringVar(value=f'Robot: waiting for {self.robot_pose_topic}')
        self.legend_var = tk.StringVar(value='PM10 heatmap: 0-30 scale, 5-unit steps')

        self.build_widgets()
        self.draw_base_map()

        self.get_logger().info(
            f'Dust map viewer started: map={self.map_yaml}, pm={self.pm_topic}, '
            f'cells={self.cells_topic}, pose={self.robot_pose_topic}'
        )

    def build_widgets(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        frame = ttk.Frame(self.root, padding=12)
        frame.grid(row=0, column=0, sticky='nsew')
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(frame, bg='#bdbdbd', highlightthickness=1, highlightbackground='#777')
        self.canvas.grid(row=0, column=0, sticky='nsew')

        status = ttk.Label(frame, textvariable=self.status_var)
        status.grid(row=1, column=0, sticky='ew', pady=(8, 0))

        pose_status = ttk.Label(frame, textvariable=self.pose_status_var)
        pose_status.grid(row=2, column=0, sticky='ew', pady=(4, 0))

        legend = ttk.Label(frame, textvariable=self.legend_var)
        legend.grid(row=3, column=0, sticky='ew', pady=(4, 0))

    def load_map(self, map_yaml: str):
        yaml_path = Path(map_yaml)
        text = yaml_path.read_text()
        data = {
            'resolution': 0.05,
            'origin': (0.0, 0.0),
            'image': '',
        }

        for line in text.splitlines():
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            if line.startswith('image:'):
                data['image'] = line.split(':', 1)[1].strip()
            elif line.startswith('resolution:'):
                data['resolution'] = float(line.split(':', 1)[1].strip())
            elif line.startswith('origin:'):
                raw = line.split('[', 1)[1].split(']', 1)[0]
                values = [float(value.strip()) for value in raw.split(',')]
                data['origin'] = (values[0], values[1])

        image_path = Path(data['image'])
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path

        image = Image.open(image_path).convert('L')
        width, height = image.size
        stride = max(1, math.ceil(max(width, height) / self.map_max_pixels))
        display_width = math.ceil(width / stride)
        display_height = math.ceil(height / stride)
        scale = max(1, self.map_max_pixels // max(display_width, display_height))

        return {
            'yaml': str(yaml_path),
            'image_path': str(image_path),
            'image': image,
            'width': width,
            'height': height,
            'resolution': data['resolution'],
            'origin': data['origin'],
            'stride': stride,
            'scale': scale,
            'display_width': display_width * scale,
            'display_height': display_height * scale,
        }

    def draw_base_map(self):
        image = self.map_info['image']
        stride = self.map_info['stride']
        scale = self.map_info['scale']

        if stride > 1:
            small = image.resize(
                (math.ceil(image.size[0] / stride), math.ceil(image.size[1] / stride)),
                Image.Resampling.NEAREST,
            )
        else:
            small = image

        rgb = Image.new('RGB', small.size)
        pixels = []
        for value in small.getdata():
            if value < 50:
                pixels.append((32, 32, 32))
            elif value > 245:
                pixels.append((245, 245, 245))
            else:
                pixels.append((155, 155, 155))
        rgb.putdata(pixels)

        display_gray = small
        if scale > 1:
            rgb = rgb.resize((rgb.size[0] * scale, rgb.size[1] * scale), Image.Resampling.NEAREST)
            display_gray = display_gray.resize(rgb.size, Image.Resampling.NEAREST)

        self.map_photo = ImageTk.PhotoImage(rgb)
        self.free_space_mask = np.array(display_gray, dtype=np.uint8) >= self.free_space_threshold
        self.canvas.config(width=rgb.size[0], height=rgb.size[1])
        self.canvas.delete('all')
        self.canvas.create_image(0, 0, anchor='nw', image=self.map_photo, tags=('map',))
        self.draw_heatmap()
        self.draw_robot_pose()

    def cells_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f'Invalid /dust/cells JSON: {msg.data}')
            return

        self.cells = payload.get('cells', [])
        self.cell_size_m = float(payload.get('cell_size_m', self.cell_size_m))
        self.update_dust_status()
        self.draw_heatmap()

    def pm_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f'Invalid {self.pm_topic} JSON: {msg.data}')
            return

        if 'pm10' not in payload:
            return

        try:
            self.latest_pm10 = float(payload['pm10'])
        except (TypeError, ValueError):
            self.get_logger().warning(f'Invalid pm10 value: {payload}')
            return

        self.update_dust_status()

    def robot_pose_callback(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        yaw = self.quaternion_to_yaw(pose.orientation)
        self.robot_pose = (float(pose.position.x), float(pose.position.y), yaw)
        self.pose_status_var.set(
            f'Robot: x={pose.position.x:.2f}, y={pose.position.y:.2f}, yaw={yaw:.2f}'
        )
        self.draw_robot_pose()

    def draw_heatmap(self):
        self.canvas.delete('heatmap')
        self.canvas.delete('legend')
        if self.cells and self.free_space_mask is not None:
            overlay = self.build_heatmap_overlay()
            if overlay is not None:
                self.heatmap_photo = ImageTk.PhotoImage(overlay)
                self.canvas.create_image(0, 0, anchor='nw', image=self.heatmap_photo, tags=('heatmap',))

        self.canvas.tag_raise('robot')
        self.draw_legend()
        self.draw_robot_pose()

    def build_heatmap_overlay(self):
        width = int(self.map_info['display_width'])
        height = int(self.map_info['display_height'])
        if width <= 0 or height <= 0:
            return None

        weighted_pm = np.zeros((height, width), dtype=np.float32)
        weights = np.zeros((height, width), dtype=np.float32)
        radius_px = max(
            6,
            int(self.heatmap_radius_m / self.map_info['resolution'] / self.map_info['stride'] * self.map_info['scale']),
        )
        sigma = max(1.0, radius_px / 2.0)

        for cell in self.cells:
            try:
                x = float(cell['x'])
                y = float(cell['y'])
                pm10 = float(cell['pm10'])
                count = max(1, int(cell.get('count', 1)))
            except (KeyError, TypeError, ValueError):
                continue

            cx, cy = self.world_to_canvas(x, y)
            center_x = int(round(cx))
            center_y = int(round(cy))
            if center_x < -radius_px or center_x >= width + radius_px:
                continue
            if center_y < -radius_px or center_y >= height + radius_px:
                continue

            x0 = max(0, center_x - radius_px)
            x1 = min(width, center_x + radius_px + 1)
            y0 = max(0, center_y - radius_px)
            y1 = min(height, center_y + radius_px + 1)
            if x0 >= x1 or y0 >= y1:
                continue

            ys, xs = np.ogrid[y0:y1, x0:x1]
            dist2 = (xs - center_x) ** 2 + (ys - center_y) ** 2
            kernel = np.exp(-dist2 / (2.0 * sigma * sigma)).astype(np.float32) * count
            weighted_pm[y0:y1, x0:x1] += kernel * pm10
            weights[y0:y1, x0:x1] += kernel

        valid = weights > 1e-4
        if not np.any(valid):
            return None

        pm_grid = np.zeros_like(weighted_pm)
        pm_grid[valid] = weighted_pm[valid] / weights[valid]
        rgba = self.pm_grid_to_rgba(pm_grid)

        alpha = np.zeros_like(weights, dtype=np.float32)
        max_weight = float(np.max(weights))
        if max_weight > 0.0:
            alpha[valid] = np.clip(weights[valid] / max_weight, 0.0, 1.0)
        alpha = np.sqrt(alpha) * np.clip(self.heatmap_alpha, 0.0, 1.0) * 255.0
        alpha[~valid] = 0.0
        alpha[~self.free_space_mask] = 0.0
        rgba[..., 3] = alpha.astype(np.uint8)

        return Image.fromarray(rgba, mode='RGBA')

    def draw_robot_pose(self):
        self.canvas.delete('robot')
        if self.robot_pose is None:
            return

        x, y, yaw = self.robot_pose
        cx, cy = self.world_to_canvas(x, y)
        radius = 8
        self.canvas.create_oval(
            cx - radius,
            cy - radius,
            cx + radius,
            cy + radius,
            fill='#2e7d32',
            outline='white',
            width=2,
            tags=('robot',),
        )
        heading_len = 24
        hx = cx + math.cos(yaw) * heading_len
        hy = cy - math.sin(yaw) * heading_len
        self.canvas.create_line(
            cx,
            cy,
            hx,
            hy,
            fill='#2e7d32',
            width=3,
            arrow=tk.LAST,
            tags=('robot',),
        )
        self.canvas.create_text(
            cx + 10,
            cy - 14,
            text='Robot',
            fill='#1b5e20',
            anchor='w',
            font=('Sans', 10, 'bold'),
            tags=('robot',),
        )

    def draw_legend(self):
        x = 12
        y = 12
        swatch = 18
        gap = 2
        self.canvas.create_text(
            x,
            y,
            text='PM10',
            anchor='w',
            fill='#111111',
            font=('Sans', 10, 'bold'),
            tags=('legend',),
        )
        y += 18
        for index, value in enumerate(range(5, 31, 5)):
            x0 = x + index * (swatch + gap)
            self.canvas.create_rectangle(
                x0,
                y,
                x0 + swatch,
                y + 14,
                fill=self.pm10_to_hex(float(value)),
                outline='white',
                tags=('legend',),
            )
        self.canvas.create_text(x, y + 26, text='0', anchor='w', fill='#111111', tags=('legend',))
        self.canvas.create_text(
            x + 5 * (swatch + gap) + swatch,
            y + 26,
            text='30+',
            anchor='e',
            fill='#111111',
            tags=('legend',),
        )

    def world_to_canvas(self, world_x: float, world_y: float):
        origin_x, origin_y = self.map_info['origin']
        resolution = self.map_info['resolution']
        stride = self.map_info['stride']
        scale = self.map_info['scale']
        display_height = self.map_info['display_height']

        pixel_x = (world_x - origin_x) / resolution
        pixel_y = (world_y - origin_y) / resolution
        canvas_x = pixel_x / stride * scale
        canvas_y = display_height - (pixel_y / stride * scale)
        return canvas_x, canvas_y

    @staticmethod
    def clamp_pm10(pm10: float):
        return max(0.0, min(30.0, pm10))

    @classmethod
    def pm10_to_rgb(cls, pm10: float):
        value = cls.clamp_pm10(pm10)
        stops = cls.PM10_COLOR_STOPS
        if value <= stops[0][0]:
            return stops[0][1]

        for index in range(len(stops) - 1):
            start_value, start_color = stops[index]
            end_value, end_color = stops[index + 1]
            if value <= end_value:
                span = max(1e-6, end_value - start_value)
                ratio = (value - start_value) / span
                return tuple(
                    int(round(start_color[channel] + (end_color[channel] - start_color[channel]) * ratio))
                    for channel in range(3)
                )

        return stops[-1][1]

    @classmethod
    def pm10_to_hex(cls, pm10: float):
        red, green, blue = cls.pm10_to_rgb(pm10)
        return f'#{red:02x}{green:02x}{blue:02x}'

    @classmethod
    def pm_grid_to_rgba(cls, pm_grid):
        rgba = np.zeros((pm_grid.shape[0], pm_grid.shape[1], 4), dtype=np.uint8)
        pm_grid = np.clip(pm_grid, 0.0, 30.0)
        stops = [
            (value, np.array(color, dtype=np.float32))
            for value, color in cls.PM10_COLOR_STOPS
        ]

        for index in range(len(stops) - 1):
            start_value, start_color = stops[index]
            end_value, end_color = stops[index + 1]
            if index == 0:
                mask = pm_grid <= end_value
            else:
                mask = (pm_grid > start_value) & (pm_grid <= end_value)
            ratio = np.clip((pm_grid[mask] - start_value) / max(1e-6, end_value - start_value), 0.0, 1.0)
            rgba[mask, :3] = (
                start_color + (end_color - start_color) * ratio[:, None]
            ).astype(np.uint8)

        high_mask = pm_grid > stops[-1][0]
        rgba[high_mask, :3] = stops[-1][1].astype(np.uint8)
        return rgba

    def update_dust_status(self):
        cell_count = len(self.cells)
        total_count = 0
        weighted_sum = 0.0
        max_pm10 = None

        for cell in self.cells:
            try:
                pm10 = float(cell['pm10'])
                count = max(1, int(cell.get('count', 1)))
            except (KeyError, TypeError, ValueError):
                continue

            total_count += count
            weighted_sum += pm10 * count
            max_pm10 = pm10 if max_pm10 is None else max(max_pm10, pm10)

        avg_pm10 = weighted_sum / total_count if total_count > 0 else None
        self.status_var.set(
            f'PM10 current={self.format_pm(self.latest_pm10)} ({self.pm10_level_text(self.latest_pm10)}), '
            f'cells={cell_count}, avg={self.format_pm(avg_pm10)}, max={self.format_pm(max_pm10)}'
        )

    @staticmethod
    def format_pm(value):
        if value is None:
            return 'n/a'
        return f'{value:.1f}'

    @staticmethod
    def pm10_level_text(value):
        if value is None:
            return 'n/a'
        if value > 30.0:
            return '30+'
        return f'{value:.1f}/30'

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def run(self):
        self.root.after(50, self.spin_once)
        self.root.mainloop()

    def spin_once(self):
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.root.after(50, self.spin_once)

    def close(self):
        self.get_logger().info('Closing dust map viewer')
        self.root.quit()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = DustMapViewer()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
