#!/usr/bin/env python3

"""ROS2 /dust/cells -> MQTT robot/1/dust 다리.

dust_mapping/dust_mapper 가 발행하는 ``/dust/cells`` (std_msgs/String, JSON)
페이로드를 그대로 MQTT 로 forward 한다. Spring Boot(robot-web) 의
MqttRobotDustSubscriber 가 이를 구독해서 웹 캔버스에 PM10 히트맵으로
그린다.

amcl_pose / battery_state 펴블리셔와 동일 패턴(rclpy + paho-mqtt + 주기 publish).
ROS 토픽이 self-throttling 이 약하면 publish_interval_sec 으로 발행률 조절.
"""

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DustCellsMqttPublisher(Node):
    def __init__(self):
        super().__init__('dust_cells_mqtt_publisher')

        self.declare_parameter('cells_topic', '/dust/cells')
        self.declare_parameter('mqtt_host', '192.168.0.55')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', 'robot/1/dust')
        self.declare_parameter('publish_interval_sec', 2.0)
        self.declare_parameter('retain', True)

        self.cells_topic = self.get_parameter('cells_topic').value
        self.mqtt_host = self.get_parameter('mqtt_host').value
        self.mqtt_port = int(self.get_parameter('mqtt_port').value)
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        self.publish_interval_sec = float(
            self.get_parameter('publish_interval_sec').value
        )
        self.retain = bool(self.get_parameter('retain').value)

        self.latest_payload = None
        self.dirty = False

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
        self.mqtt_client.loop_start()

        self.create_subscription(
            String,
            self.cells_topic,
            self.cells_callback,
            10,
        )

        self.create_timer(self.publish_interval_sec, self.publish_dust)

        self.get_logger().info(
            f'Dust cells MQTT publisher started. '
            f'cells_topic={self.cells_topic}, '
            f'mqtt={self.mqtt_host}:{self.mqtt_port}, '
            f'mqtt_topic={self.mqtt_topic}, '
            f'interval={self.publish_interval_sec}s, retain={self.retain}'
        )

    def cells_callback(self, msg: String):
        self.latest_payload = msg.data
        self.dirty = True

    def publish_dust(self):
        if self.latest_payload is None:
            return
        if not self.dirty:
            # 페이로드 변경 없으면 retain 으로 이미 브로커가 보관 중 — skip
            return
        self.mqtt_client.publish(
            self.mqtt_topic,
            self.latest_payload,
            qos=0,
            retain=self.retain,
        )
        self.dirty = False
        self.get_logger().debug(
            f'dust forwarded ({len(self.latest_payload)} bytes)'
        )

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DustCellsMqttPublisher()

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
