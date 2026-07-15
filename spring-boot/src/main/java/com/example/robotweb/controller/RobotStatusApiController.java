package com.example.robotweb.controller;

import com.example.robotweb.dto.RobotBatteryDto;
import com.example.robotweb.dto.RobotPoseDto;
import com.example.robotweb.entity.RobotStatus;
import com.example.robotweb.mqtt.MqttCommandPublisher;
import com.example.robotweb.repository.RobotStatusRepository;
import com.example.robotweb.service.RobotBatteryService;
import com.example.robotweb.service.RobotDustService;
import com.example.robotweb.service.RobotPoseService;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.Locale;
import java.util.Map;

@RestController
@RequestMapping("/api/robot")
public class RobotStatusApiController {

    private static final double MIN_START_BATTERY_PERCENT = 20.0;

    private final RobotStatusRepository robotStatusRepository;
    private final MqttCommandPublisher mqttCommandPublisher;
    private final RobotPoseService robotPoseService;
    private final RobotBatteryService robotBatteryService;
    private final RobotDustService robotDustService;

    public RobotStatusApiController(RobotStatusRepository robotStatusRepository,
                                    MqttCommandPublisher mqttCommandPublisher,
                                    RobotPoseService robotPoseService,
                                    RobotBatteryService robotBatteryService,
                                    RobotDustService robotDustService) {
        this.robotStatusRepository = robotStatusRepository;
        this.mqttCommandPublisher = mqttCommandPublisher;
        this.robotPoseService = robotPoseService;
        this.robotBatteryService = robotBatteryService;
        this.robotDustService = robotDustService;
    }

    @GetMapping("/status")
    public RobotStatus getLatestStatus() {
        return robotStatusRepository.findTopByOrderByUpdatedAtDesc()
                .orElse(new RobotStatus("TurtleBot3", "대기중", 85, 12));
    }

    @GetMapping("/pose")
    public RobotPoseDto getLatestPose() {
        return robotPoseService.getLatestPose();
    }

    @GetMapping("/battery")
    public RobotBatteryDto getLatestBattery() {
        return robotBatteryService.getLatestBattery();
    }

    @GetMapping(value = "/dust", produces = MediaType.APPLICATION_JSON_VALUE)
    public ResponseEntity<String> getLatestDust() {
        String payload = robotDustService.getLatestPayload();
        return ResponseEntity.ok(payload != null ? payload : "null");
    }

    @PostMapping("/goto")
    public Map<String, Object> gotoXy(@RequestBody Map<String, Object> body) {
        if (body == null || body.get("x") == null || body.get("y") == null) {
            return Map.of("ok", false, "error", "x and y required");
        }

        double x = ((Number) body.get("x")).doubleValue();
        double y = ((Number) body.get("y")).doubleValue();
        Object yawObj = body.get("yaw");

        RobotBatteryDto battery = robotBatteryService.getLatestBattery();

        if (isStartBlockedByBattery(battery)) {
            return Map.of(
                    "ok", false,
                    "error", "battery low",
                    "percentage", battery == null ? 0 : battery.getPercentage()
            );
        }

        String cmd;

        if (yawObj == null) {
            cmd = String.format(Locale.US, "GOTO_XY:%.4f,%.4f", x, y);
        } else {
            double yaw = ((Number) yawObj).doubleValue();
            cmd = String.format(Locale.US, "GOTO_XY:%.4f,%.4f,%.4f", x, y, yaw);
        }

        mqttCommandPublisher.publishCommand(cmd);

        RobotStatus current = robotStatusRepository.findTopByOrderByUpdatedAtDesc()
                .orElse(new RobotStatus("TurtleBot3", "대기중", 85, 12));

        int currentBatteryPercent = getBatteryPercentOrDefault(
                battery,
                current.getBattery()
        );

        RobotStatus saved = robotStatusRepository.save(new RobotStatus(
                current.getRobotName(),
                String.format(Locale.US, "이동중 (%.2f, %.2f)", x, y),
                currentBatteryPercent,
                current.getPm25()
        ));

        return Map.of("ok", true, "sent", cmd, "robotStatus", saved);
    }

    @PostMapping("/command/{command}")
    public RobotStatus updateStatus(@PathVariable String command) {
        RobotStatus currentStatus = robotStatusRepository.findTopByOrderByUpdatedAtDesc()
                .orElse(new RobotStatus("TurtleBot3", "대기중", 85, 12));

        RobotBatteryDto battery = robotBatteryService.getLatestBattery();

        String normalizedCommand = command == null
                ? ""
                : command.trim().toLowerCase();

        int currentBatteryPercent = getBatteryPercentOrDefault(
                battery,
                currentStatus.getBattery()
        );

        /*
         * 배터리 부족 상태에서는 이동 시작 / 순찰만 차단한다.
         * STOP / IDLE / GO_CHARGE / ESTOP / RELEASE / CLEAN / CLEAN_OFF 는 안전상 허용한다.
         */
        if ("start".equals(normalizedCommand) || "patrol".equals(normalizedCommand)) {
            if (isStartBlockedByBattery(battery)) {
                RobotStatus blockedStatus = new RobotStatus(
                        currentStatus.getRobotName(),
                        "배터리 부족 - 출동 불가",
                        currentBatteryPercent,
                        currentStatus.getPm25()
                );

                return robotStatusRepository.save(blockedStatus);
            }
        }

        /*
         * 중요:
         * 로봇이 이동 중이면 CLEAN 명령을 바로 보내지 않는다.
         * 클리닝은 원칙적으로 ROS2 컨트롤러가 목적지 도착을 판단한 뒤 보내는 것이 맞다.
         * 이 clean 버튼은 수동 테스트용으로만 사용한다.
         */
        if ("clean".equals(normalizedCommand)) {
            if (isRobotMoving(currentStatus.getStatus())) {
                RobotStatus blockedStatus = new RobotStatus(
                        currentStatus.getRobotName(),
                        "이동 중 - 도착 후 청정 가능",
                        currentBatteryPercent,
                        currentStatus.getPm25()
                );

                return robotStatusRepository.save(blockedStatus);
            }
        }

        String newStatus;
        String mqttCommand;

        switch (normalizedCommand) {
            case "start":
                newStatus = "이동중";
                mqttCommand = "START";
                break;

            case "stop":
                newStatus = "정지";
                mqttCommand = "STOP";
                break;

            case "charge":
                newStatus = "충전소 이동중";
                mqttCommand = "GO_CHARGE";
                break;

            case "idle":
                newStatus = "대기중";
                mqttCommand = "IDLE";
                break;

            case "estop":
                newStatus = "비상정지";
                mqttCommand = "ESTOP";
                break;

            case "release":
                newStatus = "대기중";
                mqttCommand = "RELEASE";
                break;

            case "patrol":
                newStatus = "순찰중";
                mqttCommand = "PATROL";
                break;

            case "clean":
                newStatus = "청정중";
                mqttCommand = "CLEAN";
                break;

            case "clean_off":
            case "cleanoff":
                newStatus = "청정 정지";
                mqttCommand = "CLEAN_OFF";
                break;

            default:
                RobotStatus unknownStatus = new RobotStatus(
                        currentStatus.getRobotName(),
                        "알 수 없는 명령",
                        currentBatteryPercent,
                        currentStatus.getPm25()
                );

                return robotStatusRepository.save(unknownStatus);
        }

        RobotStatus newRobotStatus = new RobotStatus(
                currentStatus.getRobotName(),
                newStatus,
                currentBatteryPercent,
                currentStatus.getPm25()
        );

        RobotStatus savedStatus = robotStatusRepository.save(newRobotStatus);

        mqttCommandPublisher.publishCommand(mqttCommand);

        return savedStatus;
    }

    private boolean isRobotMoving(String status) {
        if (status == null || status.isBlank()) {
            return false;
        }

        return status.contains("이동중")
                || status.contains("순찰중")
                || status.contains("충전소 이동중");
    }

    private boolean isStartBlockedByBattery(RobotBatteryDto battery) {
        if (battery == null) {
            return true;
        }

        if (!battery.isPresent()) {
            return true;
        }

        return battery.getPercentage() < MIN_START_BATTERY_PERCENT;
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
}