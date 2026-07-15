#!/usr/bin/env python3

import json
import math

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class AmclPoseMqttPublisher(Node):
    def __init__(self):
        super().__init__('amcl_pose_mqtt_publisher')

        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('mqtt_host', '192.168.0.55')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', 'robot/1/pose')
        self.declare_parameter('publish_interval_sec', 0.5)

        self.pose_topic = self.get_parameter('pose_topic').value
        self.mqtt_host = self.get_parameter('mqtt_host').value
        self.mqtt_port = int(self.get_parameter('mqtt_port').value)
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        self.publish_interval_sec = float(
            self.get_parameter('publish_interval_sec').value
        )

        self.latest_pose = None

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
        self.mqtt_client.loop_start()

        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_callback,
            pose_qos
        )

        self.create_timer(self.publish_interval_sec, self.publish_pose)

        self.get_logger().info(
            f'AMCL pose MQTT publisher started. '
            f'pose_topic={self.pose_topic}, '
            f'mqtt={self.mqtt_host}:{self.mqtt_port}, '
            f'mqtt_topic={self.mqtt_topic}'
        )

    def pose_callback(self, msg):
        pose = msg.pose.pose
        q = pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.latest_pose = {
            'robotId': 1,
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw': float(yaw)
        }

    def publish_pose(self):
        if self.latest_pose is None:
            self.get_logger().warn('Waiting for /amcl_pose...')
            return

        payload = json.dumps(self.latest_pose)
        self.mqtt_client.publish(self.mqtt_topic, payload)

        self.get_logger().info(
            f'Published pose: '
            f'x={self.latest_pose["x"]:.3f}, '
            f'y={self.latest_pose["y"]:.3f}, '
            f'yaw={self.latest_pose["yaw"]:.3f}'
        )

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AmclPoseMqttPublisher()

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
