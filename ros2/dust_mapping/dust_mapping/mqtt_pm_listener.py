#!/usr/bin/env python3

import json
import shutil
import subprocess
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MqttPmListener(Node):
    """Subscribe to PM sensor MQTT data and republish it as ROS2 JSON."""

    def __init__(self):
        super().__init__('mqtt_pm_listener')

        self.declare_parameter('broker_host', '192.168.0.55')
        self.declare_parameter('broker_port', 1883)
        self.declare_parameter('mqtt_topic', 'sensor/pm/zoneA/#')
        self.declare_parameter('output_topic', '/dust/pm')
        self.declare_parameter('backend', 'auto')

        self.broker_host = str(self.get_parameter('broker_host').value)
        self.broker_port = int(self.get_parameter('broker_port').value)
        self.mqtt_topic = str(self.get_parameter('mqtt_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.backend = str(self.get_parameter('backend').value).lower()

        self.pub = self.create_publisher(String, self.output_topic, 10)
        self.process: Optional[subprocess.Popen] = None
        self.stop_event = threading.Event()
        self.worker = threading.Thread(target=self.run_backend, daemon=True)
        self.worker.start()

        self.get_logger().info(
            f'MQTT PM listener started: {self.broker_host}:{self.broker_port}, '
            f'topic={self.mqtt_topic}, output={self.output_topic}, backend={self.backend}'
        )

    def run_backend(self):
        if self.backend in ('auto', 'paho'):
            try:
                self.run_paho()
                return
            except ImportError:
                if self.backend == 'paho':
                    self.get_logger().error('paho-mqtt is not installed.')
                    return
                self.get_logger().warning('paho-mqtt not installed. Falling back to mosquitto_sub.')
            except Exception as exc:
                if self.backend == 'paho':
                    self.get_logger().error(f'paho MQTT backend failed: {exc}')
                    return
                self.get_logger().warning(f'paho MQTT backend failed: {exc}. Falling back to mosquitto_sub.')

        self.run_mosquitto_sub()

    def run_paho(self):
        import paho.mqtt.client as mqtt

        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                self.get_logger().info('Connected to MQTT broker with paho.')
                client.subscribe(self.mqtt_topic)
            else:
                self.get_logger().error(f'MQTT connection failed, rc={rc}')

        def on_message(client, userdata, msg):
            self.publish_pm_message(msg.topic, msg.payload.decode('utf-8', errors='replace'))

        client = mqtt.Client()
        client.on_connect = on_connect
        client.on_message = on_message
        client.connect(self.broker_host, self.broker_port, keepalive=30)

        while rclpy.ok() and not self.stop_event.is_set():
            client.loop(timeout=0.2)

        client.disconnect()

    def run_mosquitto_sub(self):
        if shutil.which('mosquitto_sub') is None:
            self.get_logger().error(
                'mosquitto_sub was not found. Install mosquitto-clients or python3-paho-mqtt.'
            )
            return

        cmd = [
            'mosquitto_sub',
            '-h',
            self.broker_host,
            '-p',
            str(self.broker_port),
            '-t',
            self.mqtt_topic,
            '-v',
        ]
        self.get_logger().info('Starting mosquitto_sub backend.')

        while rclpy.ok() and not self.stop_event.is_set():
            try:
                self.process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    bufsize=1,
                )

                assert self.process.stdout is not None
                for line in self.process.stdout:
                    if self.stop_event.is_set():
                        break

                    line = line.strip()
                    if not line:
                        continue

                    topic, payload = self.split_mosquitto_line(line)
                    self.publish_pm_message(topic, payload)

                if self.process.poll() is None:
                    self.process.terminate()

                if not self.stop_event.is_set():
                    self.get_logger().warning('mosquitto_sub stopped. Reconnecting in 2 seconds.')
                    time.sleep(2.0)

            except Exception as exc:
                self.get_logger().error(f'mosquitto_sub backend error: {exc}')
                time.sleep(2.0)

    @staticmethod
    def split_mosquitto_line(line: str):
        parts = line.split(' ', 1)
        if len(parts) == 1:
            return '', parts[0]
        return parts[0], parts[1]

    def publish_pm_message(self, topic: str, payload_text: str):
        try:
            payload = json.loads(payload_text)
        except json.JSONDecodeError:
            self.get_logger().warning(f'Invalid MQTT JSON: topic={topic}, payload={payload_text}')
            return

        payload['mqtt_topic'] = topic
        payload['source'] = 'mqtt'
        payload['received_time'] = self.get_clock().now().nanoseconds / 1_000_000_000.0

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)

    def destroy_node(self):
        self.stop_event.set()
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttPmListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
