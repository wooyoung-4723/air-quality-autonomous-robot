package com.example.robotweb.entity;

import jakarta.persistence.*;
import java.time.LocalDateTime;

@Entity
public class RobotStatus {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    private String robotName;

    private String status;

    private Integer battery;

    private Integer pm25;

    private LocalDateTime updatedAt;

    public RobotStatus() {
    }

    public RobotStatus(String robotName, String status, Integer battery, Integer pm25) {
        this.robotName = robotName;
        this.status = status;
        this.battery = battery;
        this.pm25 = pm25;
        this.updatedAt = LocalDateTime.now();
    }

    @PrePersist
    @PreUpdate
    public void updateTime() {
        this.updatedAt = LocalDateTime.now();
    }

    public Long getId() {
        return id;
    }

    public String getRobotName() {
        return robotName;
    }

    public void setRobotName(String robotName) {
        this.robotName = robotName;
    }

    public String getStatus() {
        return status;
    }

    public void setStatus(String status) {
        this.status = status;
    }

    public Integer getBattery() {
        return battery;
    }

    public void setBattery(Integer battery) {
        this.battery = battery;
    }

    public Integer getPm25() {
        return pm25;
    }

    public void setPm25(Integer pm25) {
        this.pm25 = pm25;
    }

    public LocalDateTime getUpdatedAt() {
        return updatedAt;
    }
}