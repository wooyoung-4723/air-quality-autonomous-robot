#!/usr/bin/env python3

import json
import math
from io import BytesIO
import tkinter as tk
from tkinter import ttk

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from PIL import Image, ImageTk
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class AirCleanGui(Node):
    """Tkinter GUI that publishes manual air-clean robot commands."""

    def __init__(self):
        super().__init__('air_clean_gui')

        self.declare_parameter('command_topic', '/air_clean_command')
        self.declare_parameter('air_quality_topic', '/air_quality')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_pose_topic', '/amcl_pose')
        self.declare_parameter('path_topic', '/astar_path')
        self.declare_parameter('debug_topic', '/air_clean_debug')
        self.declare_parameter('camera_image_topic', '/image_raw/compressed')
        self.declare_parameter('camera_update_period_sec', 0.2)
        self.declare_parameter('window_title', 'Air Clean Robot Control')
        self.declare_parameter('node_1_x', 1.0)
        self.declare_parameter('node_1_y', 0.0)
        self.declare_parameter('node_2_x', 0.0)
        self.declare_parameter('node_2_y', 0.0)
        self.declare_parameter('node_3_x', 0.0)
        self.declare_parameter('node_3_y', 0.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('map_max_pixels', 620)

        self.command_topic = self.get_parameter('command_topic').value
        self.air_quality_topic = self.get_parameter('air_quality_topic').value
        self.map_topic = self.get_parameter('map_topic').value
        self.robot_pose_topic = self.get_parameter('robot_pose_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.debug_topic = self.get_parameter('debug_topic').value
        self.camera_image_topic = self.get_parameter('camera_image_topic').value
        self.camera_update_period_sec = float(
            self.get_parameter('camera_update_period_sec').value
        )
        self.window_title = self.get_parameter('window_title').value
        self.node_markers = [
            ('Node 1', float(self.get_parameter('node_1_x').value), float(self.get_parameter('node_1_y').value)),
            ('Node 2', float(self.get_parameter('node_2_x').value), float(self.get_parameter('node_2_y').value)),
            ('Node 3', float(self.get_parameter('node_3_x').value), float(self.get_parameter('node_3_y').value)),
        ]
        self.home_x = float(self.get_parameter('home_x').value)
        self.home_y = float(self.get_parameter('home_y').value)
        self.map_max_pixels = int(self.get_parameter('map_max_pixels').value)

        self.map_info = None
        self.map_stride = 1
        self.map_scale = 1
        self.map_photo = None
        self.robot_pose = None
        self.path_points = []
        self.camera_photo = None
        self.last_camera_update_time = 0.0
        self.latest_camera_data = None
        self.latest_camera_frame_id = ''
        self.latest_camera_stamp = None
        self.debug_lines = []

        self.command_pub = self.create_publisher(String, self.command_topic, 10)
        self.air_quality_sub = self.create_subscription(
            String,
            self.air_quality_topic,
            self.air_quality_callback,
            10,
        )
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, map_qos)

        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.robot_pose_topic,
            self.robot_pose_callback,
            pose_qos,
        )
        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10,
        )
        self.debug_sub = self.create_subscription(
            String,
            self.debug_topic,
            self.debug_callback,
            10,
        )

        camera_qos = QoSProfile(depth=1)
        camera_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.camera_sub = self.create_subscription(
            CompressedImage,
            self.camera_image_topic,
            self.camera_image_callback,
            camera_qos,
        )

        self.root = tk.Tk()
        self.root.title(self.window_title)
        self.root.geometry('1180x760')
        self.root.minsize(1040, 680)
        self.root.protocol('WM_DELETE_WINDOW', self.close)

        self.status_var = tk.StringVar(value='Ready')
        self.command_var = tk.StringVar(value='Last command: none')
        self.air_quality_var = tk.StringVar(value='Air quality: no data')
        self.map_status_var = tk.StringVar(value='Map: waiting for /map')
        self.pose_status_var = tk.StringVar(value='Robot: waiting for /amcl_pose')
        self.camera_status_var = tk.StringVar(value='Camera: waiting for compressed image')
        self.debug_status_var = tk.StringVar(value='Controller: waiting for debug')

        self.build_widgets()
        self.root.after(
            max(1, int(self.camera_update_period_sec * 1000)),
            self.update_camera_view,
        )
        self.get_logger().info(
            f'GUI ready. Publishing commands to {self.command_topic}, '
            f'listening to {self.air_quality_topic}, {self.map_topic}, '
            f'{self.robot_pose_topic}, {self.path_topic}, {self.debug_topic}, '
            f'{self.camera_image_topic}'
        )

    def build_widgets(self):
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        main = ttk.Frame(self.root, padding=16)
        main.grid(row=0, column=0, sticky='nsew')
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=0)
        main.rowconfigure(1, weight=1)

        title = ttk.Label(main, text='Air Clean Robot', font=('Sans', 18, 'bold'))
        title.grid(row=0, column=0, columnspan=2, sticky='w')

        map_frame = ttk.Frame(main)
        map_frame.grid(row=1, column=0, sticky='nsew', pady=(12, 0), padx=(0, 16))
        map_frame.columnconfigure(0, weight=1)
        map_frame.rowconfigure(0, weight=1)

        self.map_canvas = tk.Canvas(map_frame, bg='#bdbdbd', highlightthickness=1, highlightbackground='#777')
        self.map_canvas.grid(row=0, column=0, sticky='nsew')
        self.map_canvas.config(width=680, height=560)
        self.map_canvas.create_text(
            310,
            260,
            text='Waiting for /map',
            fill='#333333',
            tags=('placeholder',),
        )

        map_status = ttk.Label(map_frame, textvariable=self.map_status_var)
        map_status.grid(row=1, column=0, sticky='ew', pady=(8, 0))

        pose_status = ttk.Label(map_frame, textvariable=self.pose_status_var)
        pose_status.grid(row=2, column=0, sticky='ew', pady=(4, 0))

        side = ttk.Frame(main, width=280)
        side.grid(row=1, column=1, sticky='nsew', pady=(12, 0))
        side.columnconfigure(0, weight=1)

        status = ttk.Label(side, textvariable=self.status_var, font=('Sans', 11), wraplength=260)
        status.grid(row=0, column=0, sticky='ew', pady=(0, 12))

        button_frame = ttk.Frame(side)
        button_frame.grid(row=1, column=0, sticky='ew')
        button_frame.columnconfigure(0, weight=1)

        node1_button = ttk.Button(
            button_frame,
            text='Node 1',
            command=lambda: self.publish_command('node1'),
        )
        node1_button.grid(row=0, column=0, sticky='ew', pady=6, ipady=14)

        node2_button = ttk.Button(
            button_frame,
            text='Node 2',
            command=lambda: self.publish_command('node2'),
        )
        node2_button.grid(row=1, column=0, sticky='ew', pady=6, ipady=14)

        node3_button = ttk.Button(
            button_frame,
            text='Node 3',
            command=lambda: self.publish_command('node3'),
        )
        node3_button.grid(row=2, column=0, sticky='ew', pady=6, ipady=14)

        home_button = ttk.Button(
            button_frame,
            text='Return Home',
            command=lambda: self.publish_command('home'),
        )
        home_button.grid(row=3, column=0, sticky='ew', pady=6, ipady=14)

        stop_button = ttk.Button(
            button_frame,
            text='Stop Robot',
            command=lambda: self.publish_command('stop'),
        )
        stop_button.grid(row=4, column=0, sticky='ew', pady=(16, 6), ipady=14)

        close_button = ttk.Button(
            button_frame,
            text='Close GUI',
            command=self.close,
        )
        close_button.grid(row=5, column=0, sticky='ew', pady=6, ipady=10)

        command_label = ttk.Label(side, textvariable=self.command_var, wraplength=260)
        command_label.grid(row=2, column=0, sticky='ew', pady=(18, 4))

        air_quality_label = ttk.Label(side, textvariable=self.air_quality_var, wraplength=260)
        air_quality_label.grid(row=3, column=0, sticky='ew')

        camera_frame = ttk.Frame(side)
        camera_frame.grid(row=4, column=0, sticky='ew', pady=(18, 0))
        camera_frame.columnconfigure(0, weight=1)

        self.camera_label = tk.Label(
            camera_frame,
            bg='#202020',
            width=260,
            height=180,
            text='Camera',
            fg='#d0d0d0',
        )
        self.camera_label.grid(row=0, column=0, sticky='ew')

        camera_status = ttk.Label(camera_frame, textvariable=self.camera_status_var, wraplength=260)
        camera_status.grid(row=1, column=0, sticky='ew', pady=(6, 0))

        debug_frame = ttk.Frame(side)
        debug_frame.grid(row=5, column=0, sticky='ew', pady=(14, 0))
        debug_frame.columnconfigure(0, weight=1)

        debug_status = ttk.Label(debug_frame, textvariable=self.debug_status_var, wraplength=260)
        debug_status.grid(row=0, column=0, sticky='ew', pady=(0, 4))

        self.debug_text = tk.Text(
            debug_frame,
            height=8,
            width=36,
            wrap='word',
            bg='#111111',
            fg='#e0e0e0',
            insertbackground='#e0e0e0',
            font=('Monospace', 9),
        )
        self.debug_text.grid(row=1, column=0, sticky='ew')
        self.debug_text.configure(state='disabled')

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.command_var.set(f'Last command: {command}')
        self.status_var.set(f'Published "{command}" to {self.command_topic}')
        self.get_logger().info(f'Published command: {command}')

    def air_quality_callback(self, msg):
        try:
            payload = json.loads(msg.data)
            zone = payload.get('zone', '?')
            pm25 = payload.get('pm2_5', '?')
            source = payload.get('source', '?')
            self.air_quality_var.set(f'Air quality: zone={zone}, PM2.5={pm25}, source={source}')
        except (json.JSONDecodeError, TypeError):
            self.air_quality_var.set('Air quality: invalid JSON received')
            self.get_logger().warning(f'Invalid /air_quality payload: {msg.data}')

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_stride = max(1, math.ceil(max(msg.info.width, msg.info.height) / self.map_max_pixels))
        base_width = math.ceil(msg.info.width / self.map_stride)
        base_height = math.ceil(msg.info.height / self.map_stride)
        canvas_width = max(680, self.map_canvas.winfo_width())
        canvas_height = max(560, self.map_canvas.winfo_height())
        self.map_scale = max(1, min(canvas_width // base_width, canvas_height // base_height))
        image_width = base_width * self.map_scale
        image_height = base_height * self.map_scale

        photo = tk.PhotoImage(width=image_width, height=image_height)
        for display_y in range(base_height):
            source_y = msg.info.height - 1 - min(display_y * self.map_stride, msg.info.height - 1)
            row_colors = []
            for display_x in range(base_width):
                source_x = min(display_x * self.map_stride, msg.info.width - 1)
                value = msg.data[source_y * msg.info.width + source_x]
                color = self.occupancy_to_color(value)
                row_colors.extend([color] * self.map_scale)
            row_data = '{' + ' '.join(row_colors) + '}'
            for scale_y in range(self.map_scale):
                photo.put(row_data, to=(0, display_y * self.map_scale + scale_y))

        self.map_photo = photo
        self.map_canvas.delete('all')
        self.map_canvas.create_image(0, 0, anchor='nw', image=self.map_photo, tags=('map',))
        self.map_status_var.set(
            f'Map: {msg.info.width}x{msg.info.height}, resolution={msg.info.resolution:.3f} m/pixel'
        )
        self.draw_overlays()

    def robot_pose_callback(self, msg):
        pose = msg.pose.pose
        yaw = self.quaternion_to_yaw(pose.orientation)
        self.robot_pose = (pose.position.x, pose.position.y, yaw)
        self.pose_status_var.set(
            f'Robot: x={pose.position.x:.2f}, y={pose.position.y:.2f}, yaw={yaw:.2f}'
        )
        self.draw_overlays()

    def path_callback(self, msg):
        self.path_points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        self.draw_overlays()

    def debug_callback(self, msg):
        line = msg.data.strip()
        if not line:
            return

        self.debug_lines.append(line)
        self.debug_lines = self.debug_lines[-8:]
        self.debug_status_var.set('Controller: debug active')

        if hasattr(self, 'debug_text'):
            self.debug_text.configure(state='normal')
            self.debug_text.delete('1.0', tk.END)
            self.debug_text.insert(tk.END, '\n'.join(self.debug_lines))
            self.debug_text.configure(state='disabled')
            self.debug_text.see(tk.END)

    def camera_image_callback(self, msg):
        self.latest_camera_data = bytes(msg.data)
        self.latest_camera_frame_id = msg.header.frame_id or self.camera_image_topic
        self.latest_camera_stamp = (
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
        )

    def update_camera_view(self):
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0

        if now - self.last_camera_update_time < self.camera_update_period_sec:
            self.root.after(
                max(1, int(self.camera_update_period_sec * 1000)),
                self.update_camera_view,
            )
            return

        camera_data = self.latest_camera_data
        camera_frame_id = self.latest_camera_frame_id

        if camera_data is None:
            self.root.after(
                max(1, int(self.camera_update_period_sec * 1000)),
                self.update_camera_view,
            )
            return

        self.last_camera_update_time = now

        try:
            image = Image.open(BytesIO(camera_data)).convert('RGB')
        except Exception as exc:
            self.camera_status_var.set('Camera: invalid compressed image')
            self.get_logger().warning(f'Could not decode compressed image: {exc}')
            self.root.after(
                max(1, int(self.camera_update_period_sec * 1000)),
                self.update_camera_view,
            )
            return

        image.thumbnail((260, 180), Image.Resampling.LANCZOS)
        self.camera_photo = ImageTk.PhotoImage(image)
        self.camera_label.configure(image=self.camera_photo, text='')
        self.camera_status_var.set(
            f'Camera: {camera_frame_id}'
        )
        self.root.after(
            max(1, int(self.camera_update_period_sec * 1000)),
            self.update_camera_view,
        )

    def draw_overlays(self):
        self.map_canvas.delete('overlay')
        if self.map_info is None:
            return

        self.draw_path()

        for label, x, y in self.node_markers:
            self.draw_marker(x, y, '#d32f2f', label)
        self.draw_marker(self.home_x, self.home_y, '#1976d2', 'Home')

        if self.robot_pose is not None:
            x, y, yaw = self.robot_pose
            cx, cy = self.world_to_canvas(x, y)
            radius = 7
            self.map_canvas.create_oval(
                cx - radius,
                cy - radius,
                cx + radius,
                cy + radius,
                fill='#2e7d32',
                outline='white',
                width=2,
                tags=('overlay',),
            )
            heading_len = 22
            hx = cx + math.cos(yaw) * heading_len
            hy = cy - math.sin(yaw) * heading_len
            self.map_canvas.create_line(
                cx,
                cy,
                hx,
                hy,
                fill='#2e7d32',
                width=3,
                arrow=tk.LAST,
                tags=('overlay',),
            )

    def draw_path(self):
        if len(self.path_points) < 2:
            return

        canvas_points = []
        for x, y in self.path_points:
            cx, cy = self.world_to_canvas(x, y)
            canvas_points.extend([cx, cy])

        self.map_canvas.create_line(
            *canvas_points,
            fill='#f9a825',
            width=4,
            tags=('overlay',),
        )

        for x, y in self.path_points[::max(1, len(self.path_points) // 24)]:
            cx, cy = self.world_to_canvas(x, y)
            self.map_canvas.create_oval(
                cx - 2,
                cy - 2,
                cx + 2,
                cy + 2,
                fill='#f57f17',
                outline='',
                tags=('overlay',),
            )

    def draw_marker(self, x, y, color, label):
        cx, cy = self.world_to_canvas(x, y)
        size = 9
        self.map_canvas.create_line(cx - size, cy, cx + size, cy, fill=color, width=3, tags=('overlay',))
        self.map_canvas.create_line(cx, cy - size, cx, cy + size, fill=color, width=3, tags=('overlay',))
        self.map_canvas.create_text(
            cx + 8,
            cy - 12,
            text=label,
            fill=color,
            anchor='w',
            font=('Sans', 10, 'bold'),
            tags=('overlay',),
        )

    def world_to_canvas(self, world_x, world_y):
        origin = self.map_info.origin.position
        resolution = self.map_info.resolution
        pixel_x = (world_x - origin.x) / resolution / self.map_stride * self.map_scale
        pixel_y = (world_y - origin.y) / resolution / self.map_stride * self.map_scale
        canvas_y = math.ceil(self.map_info.height / self.map_stride) * self.map_scale - pixel_y
        return pixel_x, canvas_y

    @staticmethod
    def occupancy_to_color(value):
        if value < 0:
            return '#9e9e9e'
        if value >= 65:
            return '#202020'
        if value <= 20:
            return '#f5f5f5'
        shade = 245 - int((value / 100.0) * 180)
        return f'#{shade:02x}{shade:02x}{shade:02x}'

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
        self.get_logger().info('Closing GUI')
        self.root.quit()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = AirCleanGui()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
