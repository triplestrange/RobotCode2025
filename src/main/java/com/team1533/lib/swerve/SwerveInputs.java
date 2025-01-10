package com.team1533.lib.swerve;

public class SwerveInputs {
    private double controllerX = 0;
    private double controllerY = 0;
    private double controllerOmega = 0;
    private boolean robotRelative = false;

    public SwerveInputs(double x, double y, double omega, boolean robotRelative) {
        this.controllerX = x;
        this.controllerY = y;
        this.controllerOmega = omega;
        this.robotRelative = robotRelative;

    }

    public SwerveInputs(double x, double y, double omega) {
        this.controllerX = x;
        this.controllerY = y;
        this.controllerOmega = omega;
        this.robotRelative = false;

    }

    public SwerveInputs(double x, double y) {
        this.controllerX = x;
        this.controllerY = y;
        this.controllerOmega = 0;
        this.robotRelative = false;

    }

    public SwerveInputs() {
        this.controllerX = 0;
        this.controllerY = 0;
        this.controllerOmega = 0;
        this.robotRelative = false;

    }

    public double getX() {
        return controllerX;
    }

    public double getY() {
        return controllerY;
    }

    public double getOmega() {
        return controllerOmega;
    }

    public boolean getRobotRelative() {
        return robotRelative;
    }

    public void updateInputs(double x, double y, double omega, boolean robotRelative) {
        this.controllerX = x;
        this.controllerY = y;
        this.controllerOmega = omega;
        this.robotRelative = robotRelative;
    }

    public void updateInputs(double x, double y, double omega) {
        this.controllerX = x;
        this.controllerY = y;
        this.controllerOmega = omega;
    }

    public void updateInputs(double x, double y) {
        this.controllerX = x;
        this.controllerY = y;

    }

    public void updateXInput(double x) {
        this.controllerX = x;
    }

    public void updateYInput(double y) {
        this.controllerY = y;

    }
}
