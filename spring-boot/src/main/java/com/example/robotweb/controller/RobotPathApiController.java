package com.example.robotweb.controller;

import com.example.robotweb.service.RobotPathService;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.RestController;

/**
 * home.html에서 fetch('/api/robot/path')로 호출하는 API
 */
@RestController
public class RobotPathApiController {

    private final RobotPathService robotPathService;

    public RobotPathApiController(RobotPathService robotPathService) {
        this.robotPathService = robotPathService;
    }

    @GetMapping(
            value = "/api/robot/path",
            produces = MediaType.APPLICATION_JSON_VALUE
    )
    public ResponseEntity<String> getRobotPath() {
        String latestPathJson = robotPathService.getLatestPathJson();

        if (latestPathJson == null || latestPathJson.isBlank()) {
            return ResponseEntity.ok("null");
        }

        return ResponseEntity.ok(latestPathJson);
    }
}