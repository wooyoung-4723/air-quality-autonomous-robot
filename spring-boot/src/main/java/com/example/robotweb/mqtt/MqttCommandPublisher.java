package com.example.robotweb.mqtt;

import jakarta.annotation.PreDestroy;
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
public class MqttCommandPublisher implements ApplicationRunner {

    @Value("${mqtt.broker-url}")
    private String brokerUrl;

    @Value("${mqtt.command-topic}")
    private String commandTopic;

    private MqttClient mqttClient;

    @Override
    public void run(ApplicationArguments args) {
        connectIfNeeded();
    }

    public synchronized void publishCommand(String command) {
        try {
            connectIfNeeded();

            MqttMessage message = new MqttMessage(command.getBytes(StandardCharsets.UTF_8));
            message.setQos(1);
            message.setRetained(false);

            mqttClient.publish(commandTopic, message);

            System.out.println("[MQTT-PUB] 명령 발행 완료: topic=" + commandTopic + ", command=" + command);

        } catch (Exception e) {
            System.out.println("[MQTT-PUB] 명령 발행 실패");
            e.printStackTrace();
        }
    }

    private synchronized void connectIfNeeded() {
        try {
            if (mqttClient != null && mqttClient.isConnected()) {
                return;
            }

            String publisherClientId = "robot-web-publisher-" + System.currentTimeMillis();

            mqttClient = new MqttClient(
                    brokerUrl,
                    publisherClientId,
                    new MemoryPersistence()
            );

            MqttConnectOptions options = new MqttConnectOptions();
            options.setCleanSession(true);
            options.setAutomaticReconnect(true);
            options.setConnectionTimeout(10);
            options.setKeepAliveInterval(30);

            mqttClient.connect(options);

            System.out.println("[MQTT-PUB] 연결 성공: " + brokerUrl);
            System.out.println("[MQTT-PUB] 발행 토픽: " + commandTopic);

        } catch (Exception e) {
            System.out.println("[MQTT-PUB] 연결 실패");
            e.printStackTrace();
        }
    }

    @PreDestroy
    public void disconnect() {
        try {
            if (mqttClient != null && mqttClient.isConnected()) {
                mqttClient.disconnect();
                mqttClient.close();
                System.out.println("[MQTT-PUB] 연결 종료");
            }
        } catch (Exception e) {
            System.out.println("[MQTT-PUB] 종료 중 오류");
            e.printStackTrace();
        }
    }
}