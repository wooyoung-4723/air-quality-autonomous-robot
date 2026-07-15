package com.example.robotweb.entity;

import jakarta.persistence.*;
import java.time.LocalDateTime;

@Entity
public class PmSensorData {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    private String zone;

    private Integer pm1_0;

    private Integer pm2_5;

    private Integer pm10;

    private Integer rssi;

    private LocalDateTime createdAt;

    public PmSensorData() {
    }

    public PmSensorData(String zone, Integer pm1_0, Integer pm2_5, Integer pm10, Integer rssi) {
        this.zone = zone;
        this.pm1_0 = pm1_0;
        this.pm2_5 = pm2_5;
        this.pm10 = pm10;
        this.rssi = rssi;
        this.createdAt = LocalDateTime.now();
    }

    @PrePersist
    public void prePersist() {
        this.createdAt = LocalDateTime.now();
    }

    public Long getId() {
        return id;
    }

    public String getZone() {
        return zone;
    }

    public Integer getPm1_0() {
        return pm1_0;
    }

    public Integer getPm2_5() {
        return pm2_5;
    }

    public Integer getPm10() {
        return pm10;
    }

    public Integer getRssi() {
        return rssi;
    }

    public LocalDateTime getCreatedAt() {
        return createdAt;
    }

    public void setZone(String zone) {
        this.zone = zone;
    }

    public void setPm1_0(Integer pm1_0) {
        this.pm1_0 = pm1_0;
    }

    public void setPm2_5(Integer pm2_5) {
        this.pm2_5 = pm2_5;
    }

    public void setPm10(Integer pm10) {
        this.pm10 = pm10;
    }

    public void setRssi(Integer rssi) {
        this.rssi = rssi;
    }
}