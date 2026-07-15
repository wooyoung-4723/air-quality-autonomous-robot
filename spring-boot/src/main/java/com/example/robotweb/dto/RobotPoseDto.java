package com.example.robotweb.dto;

public class RobotPoseDto {
    private Long robotId;
    private double x;
    private double y;
    private double yaw;

    public RobotPoseDto() {
    }

    public RobotPoseDto(Long robotId, double x, double y, double yaw) {
        this.robotId = robotId;
        this.x = x;
        this.y = y;
        this.yaw = yaw;
    }

    public Long getRobotId() {
        return robotId;
    }

    public void setRobotId(Long robotId) {
        this.robotId = robotId;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getYaw() {
        return yaw;
    }

    public void setYaw(double yaw) {
        this.yaw = yaw;
    }
}