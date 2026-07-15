#!/usr/bin/env python3

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimAirQualityPublisher(Node):
    """Publish fake /air_quality JSON for FSM and launch testing."""

    def __init__(self):
        super().__init__('sim_air_quality_publisher')

        self.declare_parameter('zone_id', 'A')
        self.declare_parameter('publish_topic', '/air_quality')
        self.declare_parameter('mode', 'normal')
        self.declare_parameter('publish_period_sec', 1.0)

        self.zone_id = self.get_parameter('zone_id').value
        self.publish_topic = self.get_parameter('publish_topic').value
        self.mode = self.get_parameter('mode').value
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)

        self.publisher = self.create_publisher(String, self.publish_topic, 10)
        self.timer = self.create_timer(self.publish_period_sec, self.timer_callback)
        self.sequence = 0

        self.get_logger().info(
            f'Sim publisher started: zone={self.zone_id}, mode={self.mode}, topic={self.publish_topic}'
        )

    def timer_callback(self):
        self.sequence += 1
        if self.mode == 'dirty':
            pm1_0, pm2_5, pm10 = 18, 45, 70
        else:
            pm1_0, pm2_5, pm10 = 4, 12, 20

        payload = {
            'zone': self.zone_id,
            'pm1_0': pm1_0,
            'pm2_5': pm2_5,
            'pm10': pm10,
            'source': 'sim',
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.publisher.publish(msg)
        self.get_logger().info(f'Published simulated air quality #{self.sequence}: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = SimAirQualityPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
