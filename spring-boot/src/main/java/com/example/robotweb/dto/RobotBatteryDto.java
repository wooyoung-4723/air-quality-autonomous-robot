package com.example.robotweb.dto;

public class RobotBatteryDto {
    private Long robotId;
    private double voltage;
    private double percentage;
    private boolean present;
    private int powerSupplyStatus;

    public RobotBatteryDto() {
    }

    public RobotBatteryDto(Long robotId, double voltage, double percentage, boolean present, int powerSupplyStatus) {
        this.robotId = robotId;
        this.voltage = voltage;
        this.percentage = percentage;
        this.present = present;
        this.powerSupplyStatus = powerSupplyStatus;
    }

    public Long getRobotId() {
        return robotId;
    }

    public void setRobotId(Long robotId) {
        this.robotId = robotId;
    }

    public double getVoltage() {
        return voltage;
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
    }

    public double getPercentage() {
        return percentage;
    }

    public void setPercentage(double percentage) {
        this.percentage = percentage;
    }

    public boolean isPresent() {
        return present;
    }

    public void setPresent(boolean present) {
        this.present = present;
    }

    public int getPowerSupplyStatus() {
        return powerSupplyStatus;
    }

    public void setPowerSupplyStatus(int powerSupplyStatus) {
        this.powerSupplyStatus = powerSupplyStatus;
    }
}