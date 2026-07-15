package com.example.robotweb.mqtt;

import com.example.robotweb.dto.RobotBatteryDto;
import com.example.robotweb.entity.PmSensorData;
import com.example.robotweb.entity.RobotStatus;
import com.example.robotweb.repository.PmSensorDataRepository;
import com.example.robotweb.repository.RobotStatusRepository;
import com.example.robotweb.service.RobotBatteryService;
import jakarta.annotation.PreDestroy;
import org.eclipse.paho.client.mqttv3.*;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.boot.ApplicationArguments;
import org.springframework.boot.ApplicationRunner;
import org.springframework.stereotype.Component;

import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.time.LocalDateTime;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

@Component
public class MqttPmSubscriber implements ApplicationRunner {

    private static final double MIN_AUTO_DISPATCH_BATTERY_PERCENT = 20.0;

    private final PmSensorDataRepository pmSensorDataRepository;
    private final RobotStatusRepository robotStatusRepository;
    private final MqttCommandPublisher mqttCommandPublisher;
    private final RobotBatteryService robotBatteryService;

    @Value("${mqtt.broker-url}")
    private String brokerUrl;

    @Value("${mqtt.client-id}")
    private String clientId;

    @Value("${mqtt.topic}")
    private String topicFilter;

    @Value("${mqtt.qos}")
    private int qos;

    @Value("${auto-dispatch.enabled:true}")
    private boolean autoDispatchEnabled;

    @Value("${auto-dispatch.pm25-threshold:30}")
    private int pm25Threshold;

    @Value("${auto-dispatch.pm10-threshold:30}")
    private int pm10Threshold;

    @Value("${auto-dispatch.cooldown-seconds:60}")
    private long cooldownSeconds;

    private MqttClient mqttClient;

    private final Map<String, LocalDateTime> lastDispatchTimeByZone = new ConcurrentHashMap<>();

    public MqttPmSubscriber(PmSensorDataRepository pmSensorDataRepository,
                            RobotStatusRepository robotStatusRepository,
                            MqttCommandPublisher mqttCommandPublisher,
                            RobotBatteryService robotBatteryService) {
        this.pmSensorDataRepository = pmSensorDataRepository;
        this.robotStatusRepository = robotStatusRepository;
        this.mqttCommandPublisher = mqttCommandPublisher;
        this.robotBatteryService = robotBatteryService;
    }

    @Override
    public void run(ApplicationArguments args) {
        System.out.println("[MQTT] Subscriber 시작");

        try {
            String uniqueClientId = clientId + "-" + System.currentTimeMillis();

            mqttClient = new MqttClient(
                    brokerUrl,
                    uniqueClientId,
                    new MemoryPersistence()
            );

            MqttConnectOptions options = new MqttConnectOptions();
            options.setCleanSession(true);
            options.setAutomaticReconnect(true);
            options.setConnectionTimeout(10);
            options.setKeepAliveInterval(30);

            mqttClient.setCallback(new MqttCallback() {
                @Override
                public void connectionLost(Throwable cause) {
                    System.out.println("[MQTT] 연결 끊김: " + cause.getMessage());
                }

                @Override
                public void messageArrived(String topic, MqttMessage message) {
                    handleMessage(topic, message);
                }

                @Override
                public void deliveryComplete(IMqttDeliveryToken token) {
                }
            });

            mqttClient.connect(options);
            mqttClient.subscribe(topicFilter, qos);

            System.out.println("[MQTT] 연결 성공: " + brokerUrl);
            System.out.println("[MQTT] 구독 토픽: " + topicFilter);
            System.out.println("[AUTO] 자동 출동 사용 여부: " + autoDispatchEnabled);
            System.out.println("[AUTO] 기준: PM2.5 >= " + pm25Threshold + ", PM10 >= " + pm10Threshold);
            System.out.println("[AUTO] 쿨타임: " + cooldownSeconds + "초");
            System.out.println("[AUTO] 최소 자동 출동 배터리: " + MIN_AUTO_DISPATCH_BATTERY_PERCENT + "%");

        } catch (Exception e) {
            System.out.println("[MQTT] 연결 실패");
            e.printStackTrace();
        }
    }

    private void handleMessage(String topic, MqttMessage message) {
        try {
            String payload = new String(message.getPayload(), StandardCharsets.UTF_8);

            System.out.println("[MQTT] topic = " + topic);
            System.out.println("[MQTT] payload = " + payload);

            Map<String, String> data = parseSimpleJson(payload);

            String zone = extractZoneFromTopic(topic);

            int pm1_0 = parseInt(data.get("pm1_0"));
            int pm2_5 = parseInt(data.get("pm2_5"));
            int pm10 = parseInt(data.get("pm10"));
            int rssi = parseInt(data.get("rssi"));

            PmSensorData pmSensorData = new PmSensorData(
                    zone,
                    pm1_0,
                    pm2_5,
                    pm10,
                    rssi
            );

            pmSensorDataRepository.save(pmSensorData);

            System.out.println("[MQTT] DB 저장 완료: zone=" + zone
                    + ", pm1_0=" + pm1_0
                    + ", pm2_5=" + pm2_5
                    + ", pm10=" + pm10
                    + ", rssi=" + rssi);

            checkAndDispatchRobot(zone, pm2_5, pm10);

        } catch (Exception e) {
            System.out.println("[MQTT] 메시지 처리 실패");
            e.printStackTrace();
        }
    }

    private void checkAndDispatchRobot(String zone, int pm2_5, int pm10) {
        if (!autoDispatchEnabled) {
            return;
        }

        boolean isBadAir = pm2_5 >= pm25Threshold || pm10 >= pm10Threshold;

        if (!isBadAir) {
            return;
        }

        RobotStatus currentStatus = robotStatusRepository.findTopByOrderByUpdatedAtDesc()
                .orElse(new RobotStatus("TurtleBot3", "대기중", 85, pm2_5));

        RobotBatteryDto battery = robotBatteryService.getLatestBattery();
        int batteryPercent = getBatteryPercentOrDefault(battery, currentStatus.getBattery());

        if (isAutoDispatchBlockedByBattery(battery)) {
            RobotStatus blockedStatus = new RobotStatus(
                    currentStatus.getRobotName(),
                    "배터리 부족 - 자동 출동 보류",
                    batteryPercent,
                    pm2_5
            );

            robotStatusRepository.save(blockedStatus);

            System.out.println("[AUTO] 공기질 기준 초과지만 배터리 부족으로 자동 출동 보류: zone=" + zone
                    + " / battery=" + batteryPercent + "%"
                    + " / PM2.5=" + pm2_5
                    + ", PM10=" + pm10);

            return;
        }

        LocalDateTime now = LocalDateTime.now();
        LocalDateTime lastDispatchTime = lastDispatchTimeByZone.get(zone);

        if (lastDispatchTime != null) {
            long seconds = Duration.between(lastDispatchTime, now).getSeconds();

            if (seconds < cooldownSeconds) {
                System.out.println("[AUTO] 기준 초과지만 쿨타임 중: zone=" + zone
                        + ", 남은 시간=" + (cooldownSeconds - seconds) + "초");
                return;
            }
        }

        String command = "GO_TO_ZONE:" + zone;

        mqttCommandPublisher.publishCommand(command);

        RobotStatus newStatus = new RobotStatus(
                currentStatus.getRobotName(),
                zone + " 공기질 나쁨 / 출동중",
                batteryPercent,
                pm2_5
        );

        robotStatusRepository.save(newStatus);

        lastDispatchTimeByZone.put(zone, now);

        System.out.println("[AUTO] 자동 출동 명령 발행: " + command
                + " / battery=" + batteryPercent + "%"
                + " / PM2.5=" + pm2_5
                + ", PM10=" + pm10);
    }

    private boolean isAutoDispatchBlockedByBattery(RobotBatteryDto battery) {
        if (battery == null) {
            return true;
        }

        if (!battery.isPresent()) {
            return true;
        }

        return battery.getPercentage() < MIN_AUTO_DISPATCH_BATTERY_PERCENT;
    }

    private int getBatteryPercentOrDefault(RobotBatteryDto battery, int defaultValue) {
        if (battery == null) {
            return defaultValue;
        }

        double percentage = battery.getPercentage();

        if (Double.isNaN(percentage) || Double.isInfinite(percentage)) {
            return defaultValue;
        }

        percentage = Math.max(0.0, Math.min(100.0, percentage));

        return (int) Math.round(percentage);
    }

    private String extractZoneFromTopic(String topic) {
        if (topic == null || topic.isBlank()) {
            return "unknown";
        }

        int lastSlashIndex = topic.lastIndexOf("/");

        if (lastSlashIndex == -1 || lastSlashIndex == topic.length() - 1) {
            return "unknown";
        }

        return topic.substring(lastSlashIndex + 1);
    }

    private Map<String, String> parseSimpleJson(String json) {
        Map<String, String> map = new HashMap<>();

        if (json == null || json.isBlank()) {
            return map;
        }

        String cleaned = json.trim();

        if (cleaned.startsWith("{")) {
            cleaned = cleaned.substring(1);
        }

        if (cleaned.endsWith("}")) {
            cleaned = cleaned.substring(0, cleaned.length() - 1);
        }

        String[] pairs = cleaned.split(",");

        for (String pair : pairs) {
            String[] keyValue = pair.split(":", 2);

            if (keyValue.length != 2) {
                continue;
            }

            String key = keyValue[0].trim().replace("\"", "");
            String value = keyValue[1].trim().replace("\"", "");

            map.put(key, value);
        }

        return map;
    }

    private int parseInt(String value) {
        if (value == null || value.isBlank()) {
            return 0;
        }

        return Integer.parseInt(value.trim());
    }

    @PreDestroy
    public void disconnect() {
        try {
            if (mqttClient != null && mqttClient.isConnected()) {
                mqttClient.disconnect();
                mqttClient.close();
                System.out.println("[MQTT] 연결 종료");
            }
        } catch (Exception e) {
            System.out.println("[MQTT] 종료 중 오류");
            e.printStackTrace();
        }
    }
}