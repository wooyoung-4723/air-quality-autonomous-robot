package com.example.robotweb.service;

import com.example.robotweb.dto.RobotBatteryDto;
import org.springframework.stereotype.Service;

@Service
public class RobotBatteryService {

    private RobotBatteryDto latestBattery =
            new RobotBatteryDto(1L, 11.38, 48.88, true, 0);

    public synchronized void updateBattery(RobotBatteryDto battery) {
        this.latestBattery = battery;
    }

    public synchronized RobotBatteryDto getLatestBattery() {
        return latestBattery;
    }
}