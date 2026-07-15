#!/usr/bin/env python3

import json

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState


class BatteryStateMqttPublisher(Node):
    def __init__(self):
        super().__init__('battery_state_mqtt_publisher')

        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('mqtt_host', '192.168.0.55')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', 'robot/1/battery')
        self.declare_parameter('publish_interval_sec', 1.0)

        self.battery_topic = self.get_parameter('battery_topic').value
        self.mqtt_host = self.get_parameter('mqtt_host').value
        self.mqtt_port = int(self.get_parameter('mqtt_port').value)
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        self.publish_interval_sec = float(
            self.get_parameter('publish_interval_sec').value
        )

        self.latest_battery = None

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
        self.mqtt_client.loop_start()

        battery_qos = QoSProfile(depth=10)
        battery_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            BatteryState,
            self.battery_topic,
            self.battery_callback,
            battery_qos
        )

        self.create_timer(self.publish_interval_sec, self.publish_battery)

        self.get_logger().info(
            f'Battery MQTT publisher started. '
            f'battery_topic={self.battery_topic}, '
            f'mqtt={self.mqtt_host}:{self.mqtt_port}, '
            f'mqtt_topic={self.mqtt_topic}'
        )

    def battery_callback(self, msg: BatteryState):
        raw_percentage = float(msg.percentage)

        # TurtleBot3에서는 percentage가 48.88처럼 이미 % 단위로 나오는 경우가 있음.
        # 만약 0.48처럼 0~1 범위로 나오면 100을 곱해서 %로 변환.
        if 0.0 <= raw_percentage <= 1.0:
            percentage = raw_percentage * 100.0
        else:
            percentage = raw_percentage

        percentage = max(0.0, min(100.0, percentage))

        self.latest_battery = {
            'robotId': 1,
            'voltage': float(msg.voltage),
            'percentage': float(percentage),
            'present': bool(msg.present),
            'powerSupplyStatus': int(msg.power_supply_status)
        }

    def publish_battery(self):
        if self.latest_battery is None:
            self.get_logger().warn('Waiting for /battery_state...')
            return

        payload = json.dumps(self.latest_battery)
        self.mqtt_client.publish(self.mqtt_topic, payload)

        self.get_logger().info(
            f'Published battery: '
            f'voltage={self.latest_battery["voltage"]:.2f}V, '
            f'percentage={self.latest_battery["percentage"]:.1f}%'
        )

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryStateMqttPublisher()

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
