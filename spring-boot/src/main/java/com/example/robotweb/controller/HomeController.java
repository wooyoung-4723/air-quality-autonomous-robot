package com.example.robotweb.controller;

import com.example.robotweb.entity.PmSensorData;
import com.example.robotweb.entity.RobotStatus;
import com.example.robotweb.repository.PmSensorDataRepository;
import com.example.robotweb.repository.RobotStatusRepository;
import org.springframework.stereotype.Controller;
import org.springframework.ui.Model;
import org.springframework.web.bind.annotation.GetMapping;

@Controller
public class HomeController {

    private final RobotStatusRepository robotStatusRepository;
    private final PmSensorDataRepository pmSensorDataRepository;

    public HomeController(RobotStatusRepository robotStatusRepository,
                          PmSensorDataRepository pmSensorDataRepository) {
        this.robotStatusRepository = robotStatusRepository;
        this.pmSensorDataRepository = pmSensorDataRepository;
    }

    @GetMapping("/")
    public String home(Model model) {
        RobotStatus robotStatus = robotStatusRepository.findTopByOrderByUpdatedAtDesc()
                .orElse(new RobotStatus("TurtleBot3", "대기중", 85, 12));

        PmSensorData pmSensorData = pmSensorDataRepository.findTopByOrderByCreatedAtDesc()
                .orElse(new PmSensorData("zoneA", 0, 0, 0, 0));

        model.addAttribute("robotStatus", robotStatus);
        model.addAttribute("pmSensorData", pmSensorData);

        return "home";
    }

    @GetMapping("/dashboard/3d")
    public String dashboard3d(Model model) {
        // rosbridge_websocket 주소. 다른 호스트면 -Drosbridge.host=... 로 override.
        model.addAttribute("rosbridgeUrl", "ws://" + System.getProperty("rosbridge.host", "localhost") + ":9090");
        return "dashboard3d";
    }
}