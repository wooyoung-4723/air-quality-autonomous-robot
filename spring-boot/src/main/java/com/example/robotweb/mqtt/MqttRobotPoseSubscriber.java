package com.example.robotweb.mqtt;

import com.example.robotweb.dto.RobotPoseDto;
import com.example.robotweb.service.RobotPoseService;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import jakarta.annotation.PostConstruct;
import org.eclipse.paho.client.mqttv3.*;
import org.springframework.stereotype.Component;

import java.nio.charset.StandardCharsets;
import java.util.UUID;

@Component
public class MqttRobotPoseSubscriber {

    private static final String BROKER_URL = "tcp://192.168.0.55:1883";
    private static final String TOPIC = "robot/1/pose";

    private final RobotPoseService robotPoseService;
    private final ObjectMapper objectMapper = new ObjectMapper();

    private MqttClient client;

    public MqttRobotPoseSubscriber(RobotPoseService robotPoseService) {
        this.robotPoseService = robotPoseService;
    }

    @PostConstruct
    public void connect() {
        try {
            String clientId = "spring-robot-pose-subscriber-" + UUID.randomUUID();

            client = new MqttClient(BROKER_URL, clientId);

            MqttConnectOptions options = new MqttConnectOptions();
            options.setCleanSession(true);
            options.setAutomaticReconnect(true);

            client.setCallback(new MqttCallback() {
                @Override
                public void connectionLost(Throwable cause) {
                    System.out.println("[MQTT POSE] 연결 끊김: " + cause.getMessage());
                }

                @Override
                public void messageArrived(String topic, MqttMessage message) {
                    handlePoseMessage(message);
                }

                @Override
                public void deliveryComplete(IMqttDeliveryToken token) {
                }
            });

            client.connect(options);
            client.subscribe(TOPIC);

            System.out.println("[MQTT POSE] 구독 시작: " + TOPIC);

        } catch (Exception e) {
            System.out.println("[MQTT POSE] 연결 실패: " + e.getMessage());
        }
    }

    private void handlePoseMessage(MqttMessage message) {
        try {
            String payload = new String(message.getPayload(), StandardCharsets.UTF_8);
            JsonNode json = objectMapper.readTree(payload);

            Long robotId = json.has("robotId") ? json.get("robotId").asLong() : 1L;
            double x = json.get("x").asDouble();
            double y = json.get("y").asDouble();
            double yaw = json.get("yaw").asDouble();

            RobotPoseDto pose = new RobotPoseDto(robotId, x, y, yaw);
            robotPoseService.updatePose(pose);

            System.out.println("[MQTT POSE] x=" + x + ", y=" + y + ", yaw=" + yaw);

        } catch (Exception e) {
            System.out.println("[MQTT POSE] 메시지 처리 실패: " + e.getMessage());
        }
    }
}