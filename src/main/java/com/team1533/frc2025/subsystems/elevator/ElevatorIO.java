package com.team1533.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSecond = 0.0;
        public double appliedVoltage = 0.0;
        public double[] currentAmps = new double[]{};
        void updateInputs(ElevatorIOInputs inputs) {}
        void setVoltage(double volts) {}
        void setPosition(double positionMeters) {}
    }
}