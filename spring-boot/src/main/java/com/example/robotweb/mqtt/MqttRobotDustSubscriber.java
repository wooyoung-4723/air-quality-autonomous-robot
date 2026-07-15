package com.example.robotweb.mqtt;

import com.example.robotweb.service.RobotDustService;
import jakarta.annotation.PostConstruct;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.springframework.stereotype.Component;

import java.nio.charset.StandardCharsets;
import java.util.UUID;

/**
 * robot/1/dust (retained, ROS2 dust_cells_mqtt_publisher 가 발행)를 구독해서
 * 가장 최근 JSON 페이로드를 RobotDustService 에 캐시한다.
 *
 * MqttRobotPoseSubscriber / MqttRobotBatterySubscriber 와 동일 패턴.
 */
@Component
public class MqttRobotDustSubscriber {

    private static final String BROKER_URL = "tcp://192.168.0.55:1883";
    private static final String TOPIC = "robot/1/dust";

    private final RobotDustService robotDustService;

    private MqttClient client;

    public MqttRobotDustSubscriber(RobotDustService robotDustService) {
        this.robotDustService = robotDustService;
    }

    @PostConstruct
    public void connect() {
        try {
            String clientId = "spring-robot-dust-subscriber-" + UUID.randomUUID();

            client = new MqttClient(BROKER_URL, clientId);

            MqttConnectOptions options = new MqttConnectOptions();
            options.setCleanSession(true);
            options.setAutomaticReconnect(true);

            client.setCallback(new MqttCallback() {
                @Override
                public void connectionLost(Throwable cause) {
                    System.out.println("[MQTT DUST] 연결 끊김: " + cause.getMessage());
                }

                @Override
                public void messageArrived(String topic, MqttMessage message) {
                    String payload = new String(message.getPayload(), StandardCharsets.UTF_8);
                    robotDustService.update(payload);
                }

                @Override
                public void deliveryComplete(IMqttDeliveryToken token) {
                }
            });

            client.connect(options);
            client.subscribe(TOPIC);

            System.out.println("[MQTT DUST] 구독 시작: " + TOPIC);

        } catch (Exception e) {
            System.out.println("[MQTT DUST] 연결 실패: " + e.getMessage());
        }
    }
}
