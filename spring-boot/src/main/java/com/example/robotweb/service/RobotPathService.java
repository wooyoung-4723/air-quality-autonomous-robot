package com.example.robotweb.service;

import org.springframework.stereotype.Service;

@Service
public class RobotPathService {

    private String latestPathJson;

    public synchronized void updatePath(String pathJson) {
        this.latestPathJson = pathJson;
    }

    public synchronized String getLatestPathJson() {
        return latestPathJson;
    }
}