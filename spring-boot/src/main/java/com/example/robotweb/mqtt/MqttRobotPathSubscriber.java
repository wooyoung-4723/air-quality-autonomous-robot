package com.example.robotweb.mqtt;

import com.example.robotweb.service.RobotPathService;
import jakarta.annotation.PreDestroy;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.ApplicationArguments;
import org.springframework.boot.ApplicationRunner;
import org.springframework.stereotype.Component;

import java.nio.charset.StandardCharsets;

@Component
public class MqttRobotPathSubscriber implements ApplicationRunner {

    @Value("${mqtt.broker-url}")
    private String brokerUrl;

    private static final String PATH_TOPIC = "robot/1/path";

    private final RobotPathService robotPathService;

    private MqttClient mqttClient;

    public MqttRobotPathSubscriber(RobotPathService robotPathService) {
        this.robotPathService = robotPathService;
    }

    @Override
    public void run(ApplicationArguments args) throws Exception {
        String clientId = "robot-web-path-subscriber-" + System.currentTimeMillis();

        mqttClient = new MqttClient(
                brokerUrl,
                clientId,
                new MemoryPersistence()
        );

        MqttConnectOptions options = new MqttConnectOptions();
        options.setAutomaticReconnect(true);
        options.setCleanSession(true);
        options.setConnectionTimeout(10);
        options.setKeepAliveInterval(30);

        mqttClient.setCallback(new MqttCallback() {
            @Override
            public void connectionLost(Throwable cause) {
                System.out.println("[MQTT PATH] 연결 끊김: " + cause.getMessage());
            }

            @Override
            public void messageArrived(String topic, MqttMessage message) {
                String payload = new String(message.getPayload(), StandardCharsets.UTF_8);

                if (PATH_TOPIC.equals(topic)) {
                    robotPathService.updatePath(payload);
                    System.out.println("[MQTT PATH] 수신: " + payload);
                }
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken token) {
            }
        });

        mqttClient.connect(options);
        mqttClient.subscribe(PATH_TOPIC, 0);

        System.out.println("[MQTT PATH] 구독 시작: " + PATH_TOPIC);
    }

    @PreDestroy
    public void close() {
        try {
            if (mqttClient != null) {
                if (mqttClient.isConnected()) {
                    mqttClient.disconnect();
                }
                mqttClient.close();
            }
        } catch (Exception e) {
            System.out.println("[MQTT PATH] 종료 중 오류: " + e.getMessage());
        }
    }
}