package com.example.robotweb.mqtt;

import com.example.robotweb.dto.RobotBatteryDto;
import com.example.robotweb.service.RobotBatteryService;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import jakarta.annotation.PostConstruct;
import org.eclipse.paho.client.mqttv3.*;
import org.springframework.stereotype.Component;

import java.nio.charset.StandardCharsets;
import java.util.UUID;

@Component
public class MqttRobotBatterySubscriber {

    private static final String BROKER_URL = "tcp://192.168.0.55:1883";
    private static final String TOPIC = "robot/1/battery";

    private final RobotBatteryService robotBatteryService;
    private final ObjectMapper objectMapper = new ObjectMapper();

    private MqttClient client;

    public MqttRobotBatterySubscriber(RobotBatteryService robotBatteryService) {
        this.robotBatteryService = robotBatteryService;
    }

    @PostConstruct
    public void connect() {
        try {
            String clientId = "spring-robot-battery-subscriber-" + UUID.randomUUID();

            client = new MqttClient(BROKER_URL, clientId);

            MqttConnectOptions options = new MqttConnectOptions();
            options.setCleanSession(true);
            options.setAutomaticReconnect(true);

            client.setCallback(new MqttCallback() {
                @Override
                public void connectionLost(Throwable cause) {
                    System.out.println("[MQTT BATTERY] 연결 끊김: " + cause.getMessage());
                }

                @Override
                public void messageArrived(String topic, MqttMessage message) {
                    handleBatteryMessage(message);
                }

                @Override
                public void deliveryComplete(IMqttDeliveryToken token) {
                }
            });

            client.connect(options);
            client.subscribe(TOPIC);

            System.out.println("[MQTT BATTERY] 구독 시작: " + TOPIC);

        } catch (Exception e) {
            System.out.println("[MQTT BATTERY] 연결 실패: " + e.getMessage());
        }
    }

    private void handleBatteryMessage(MqttMessage message) {
        try {
            String payload = new String(message.getPayload(), StandardCharsets.UTF_8);
            JsonNode json = objectMapper.readTree(payload);

            Long robotId = json.has("robotId") ? json.get("robotId").asLong() : 1L;
            double voltage = json.get("voltage").asDouble();
            double percentage = json.get("percentage").asDouble();
            boolean present = json.get("present").asBoolean();
            int powerSupplyStatus = json.get("powerSupplyStatus").asInt();

            RobotBatteryDto battery = new RobotBatteryDto(
                    robotId,
                    voltage,
                    percentage,
                    present,
                    powerSupplyStatus
            );

            robotBatteryService.updateBattery(battery);

            System.out.println(
                    "[MQTT BATTERY] voltage=" + voltage +
                            ", percentage=" + percentage +
                            ", present=" + present
            );

        } catch (Exception e) {
            System.out.println("[MQTT BATTERY] 메시지 처리 실패: " + e.getMessage());
        }
    }
}