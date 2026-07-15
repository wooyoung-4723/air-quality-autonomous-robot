package com.example.robotweb.service;

import com.example.robotweb.dto.RobotPoseDto;
import org.springframework.stereotype.Service;

@Service
public class RobotPoseService {

    private RobotPoseDto latestPose = new RobotPoseDto(
            1L,
            -0.037569766737571876,
            0.10523828207830883,
            -0.007148110090032988
    );

    public synchronized void updatePose(RobotPoseDto pose) {
        this.latestPose = pose;
    }

    public synchronized RobotPoseDto getLatestPose() {
        return latestPose;
    }
}