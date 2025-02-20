package com.team1533.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Velocity;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {

        public boolean leftConnected = false;
        public double leftVelocityRadPerSec = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        public double leftStatorAmps = 0.0;

        public boolean rightConnected = false;
        public double rightVelocityRadPerSec = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
        public double rightStatorAmps = 0.0;

        public double elevatorPosMeters = 0.0;
        public double secondsSinceReset = 0.0;
    }

    default void updateInputs(ElevatorIOInputs inputs) {
    }

    default void setOpenLoopDutyCycle(double dutyCycle) {
    }

    default void setPositionSetpoint(double positionMeters) {
    }

    default void setPositionSetpoint(double positionMeters, double feedForward) {
    }
}