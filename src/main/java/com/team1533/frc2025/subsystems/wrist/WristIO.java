package com.team1533.frc2025.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

import com.team1533.frc2025.Constants.Gains;

public interface WristIO {
    @AutoLog
    class WristIOInputs {
        public boolean leaderConnected = false;
        public double leaderVelocityRotPerSec = 0.0;
        public double leaderAppliedVolts = 0.0;
        public double leaderCurrentAmps = 0.0;
        public double leaderStatorAmps = 0.0;
        public double leaderTempCelc = 0.0;

        public boolean absoluteEncoderConnected = true;
        public double absoluteEncoderPositionRots = 0.0;
        public double relativeEncoderPositionRots = 0.0;

        public double leaderRotPosition = 0.0;

        public double wristVelMetersPerSecond = 0.0;
        public double wristAccelMetersPerSecondPerSecond = 0.0;

    }

    default void updateInputs(WristIOInputs inputs) {
    }

    default void runVolts(double volts) {
    }

    default void setDutyCycleOut(double output) {

    }

    default void setPositionSetpoint(double positionRotation) {
    }

    default void setPositionSetpoint(double positionRotation, double rotationsPerSec) {
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
