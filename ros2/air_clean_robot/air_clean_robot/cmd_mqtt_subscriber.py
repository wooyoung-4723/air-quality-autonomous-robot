#!/usr/bin/env python3

"""MQTT robot/1/cmd → ROS2 /air_clean_command 다리.

Spring Boot(robot-web)가 MQTT topic ``robot/1/cmd`` 로 보내는 평문 명령을
그대로 받아서 ROS2 ``/air_clean_command`` (std_msgs/String) 으로 forward 한다.
manual_nav_controller(또는 동일 토픽을 구독하는 다른 노드)가 이 명령을 처리한다.
"""

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CmdMqttSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_mqtt_subscriber')

        self.declare_parameter('mqtt_host', '192.168.0.55')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_topic', 'robot/1/cmd')
        self.declare_parameter('output_topic', '/air_clean_command')

        self.mqtt_host = self.get_parameter('mqtt_host').value
        self.mqtt_port = int(self.get_parameter('mqtt_port').value)
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        self.output_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(String, self.output_topic, 10)

        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, 60)
        self.mqtt_client.loop_start()

        self.get_logger().info(
            f'Cmd MQTT subscriber started. '
            f'mqtt={self.mqtt_host}:{self.mqtt_port}, '
            f'mqtt_topic={self.mqtt_topic}, '
            f'output_topic={self.output_topic}'
        )

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            client.subscribe(self.mqtt_topic)
            self.get_logger().info(f'MQTT 구독 성공: {self.mqtt_topic}')
        else:
            self.get_logger().error(f'MQTT 연결 실패 rc={rc}')

    def on_message(self, client, userdata, msg):
        try:
            text = msg.payload.decode('utf-8', errors='replace').strip()
        except Exception:
            self.get_logger().warning(f'잘못된 페이로드: {msg.payload!r}')
            return
        if not text:
            return
        out = String()
        out.data = text
        self.pub.publish(out)
        self.get_logger().info(f'MQTT cmd → ROS: "{text}"')

    def destroy_node(self):
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdMqttSubscriber()

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
