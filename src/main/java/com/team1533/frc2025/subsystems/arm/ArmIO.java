package com.team1533.frc2025.subsystems.arm;

import com.team1533.frc2025.Constants.Gains;

public interface ArmIO {

    class ArmIOInputs {
        public boolean leaderConnected = false;
        public double leaderVelocityRotPerSec = 0.0;
        public double leaderAppliedVolts = 0.0;
        public double leaderCurrentAmps = 0.0;
        public double leaderStatorAmps = 0.0;
        public double leaderTempCelc = 0.0;

        public boolean followerConnected = false;
        public double followerVelocityRotPerSec = 0.0;
        public double followerAppliedVolts = 0.0;
        public double followerCurrentAmps = 0.0;
        public double followerStatorAmps = 0.0;
        public double followerTempCelc = 0.0;

        public double leaderRotPosition = 0.0;
        public double followerRotPosition = 0.0;

        public double secondsSinceReset = 0.0;

        public double elevatorPosMeters = 0.0;
        public double elevatorVelMetersPerSecond = 0.0;
        public double elevatorAccelMetersPerSecondPerSecond = 0.0;

    }

    default void updateInputs(ArmIOInputs inputs) {
    }

    default void runVolts(double volts) {
    }

    default void setDutyCycleOut(double output) {

    }

    default void setPositionSetpoint(double positionMeters) {
    }

    default void setPositionSetpoint(double positionMeters, double metersPerSec) {
    }

    default void setCurrentSetpoint(double amps) {
    }

    default void setBrakeMode(boolean enabled) {
    }

    default void setPID(Gains gains) {
    }

    default void stop() {
    }

}
