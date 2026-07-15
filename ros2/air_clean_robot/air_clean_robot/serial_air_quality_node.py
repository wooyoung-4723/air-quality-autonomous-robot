#!/usr/bin/env python3

import json
import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
    from serial import SerialException
except ImportError:  # pragma: no cover - handled at runtime on robot
    serial = None
    SerialException = Exception


class SerialAirQualityNode(Node):
    """Read Arduino USB serial PMS values and publish the stable JSON contract."""

    def __init__(self):
        super().__init__('serial_air_quality_node')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('zone_id', 'A')
        self.declare_parameter('publish_topic', '/air_quality')
        self.declare_parameter('warmup_sec', 10.0)
        self.declare_parameter('reconnect_sec', 3.0)

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.zone_id = self.get_parameter('zone_id').value
        self.publish_topic = self.get_parameter('publish_topic').value
        self.warmup_sec = float(self.get_parameter('warmup_sec').value)
        self.reconnect_sec = float(self.get_parameter('reconnect_sec').value)

        self.publisher = self.create_publisher(String, self.publish_topic, 10)
        self.serial_conn = None
        self.partial_values = {}
        self.start_time = time.monotonic()
        self.last_connect_attempt = 0.0

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            f'Starting serial reader: port={self.serial_port}, baudrate={self.baudrate}, '
            f'zone={self.zone_id}, topic={self.publish_topic}, warmup_sec={self.warmup_sec}'
        )

    def connect_serial(self):
        if serial is None:
            self.get_logger().error('pyserial is not installed. Install with: pip install pyserial')
            return

        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_sec:
            return
        self.last_connect_attempt = now

        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baudrate, timeout=0.05)
            self.serial_conn.reset_input_buffer()
            self.get_logger().info(f'Connected to Arduino serial port {self.serial_port}')
        except SerialException as exc:
            self.serial_conn = None
            self.get_logger().error(
                f'Failed to open serial port {self.serial_port}: {exc}. '
                f'Retrying every {self.reconnect_sec:.1f}s.'
            )

    def timer_callback(self):
        if self.serial_conn is None or not self.serial_conn.is_open:
            self.connect_serial()
            return

        try:
            raw = self.serial_conn.readline()
        except SerialException as exc:
            self.get_logger().error(f'Serial read failed: {exc}. Reconnecting...')
            self.close_serial()
            return

        if not raw:
            return

        line = raw.decode('utf-8', errors='ignore').strip()
        if not line:
            return

        values = self.parse_line(line)
        if values == {}:
            return
        if values is None:
            self.get_logger().warning(f'Ignoring unparsable serial line: {line}')
            return

        elapsed = time.monotonic() - self.start_time
        if elapsed < self.warmup_sec:
            self.get_logger().info(
                f'Warmup {elapsed:.1f}/{self.warmup_sec:.1f}s, read but not publishing: {values}'
            )
            return

        payload = {
            'zone': self.zone_id,
            'pm1_0': values['pm1_0'],
            'pm2_5': values['pm2_5'],
            'pm10': values['pm10'],
            'source': 'serial',
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published air quality: {msg.data}')

    def parse_line(self, line):
        named_patterns = {
            'pm1_0': r'PM\s*1(?:\.0)?[^:=]*[:=]\s*([0-9]+(?:\.[0-9]+)?)',
            'pm2_5': r'PM\s*2(?:\.5)?[^:=]*[:=]\s*([0-9]+(?:\.[0-9]+)?)',
            'pm10': r'PM\s*10(?:\.0)?[^:=]*[:=]\s*([0-9]+(?:\.[0-9]+)?)',
        }

        values = {}
        for key, pattern in named_patterns.items():
            match = re.search(pattern, line, re.IGNORECASE)
            if match:
                values[key] = self.as_number(match.group(1))

        if len(values) == 3:
            self.partial_values = {}
            return values

        if values:
            self.partial_values.update(values)
            if len(self.partial_values) == 3:
                complete = dict(self.partial_values)
                self.partial_values = {}
                return complete
            return {}

        csv_match = re.fullmatch(
            r'\s*([0-9]+(?:\.[0-9]+)?)\s*,\s*([0-9]+(?:\.[0-9]+)?)\s*,\s*([0-9]+(?:\.[0-9]+)?)\s*',
            line,
        )
        if csv_match:
            self.partial_values = {}
            return {
                'pm1_0': self.as_number(csv_match.group(1)),
                'pm2_5': self.as_number(csv_match.group(2)),
                'pm10': self.as_number(csv_match.group(3)),
            }

        return None

    @staticmethod
    def as_number(value):
        number = float(value)
        return int(number) if number.is_integer() else number

    def close_serial(self):
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except SerialException:
                pass
        self.serial_conn = None

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialAirQualityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
