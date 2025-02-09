package com.team1533.frc2025.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIO.ElevatorIOInputs inputs = new ElevatorIO.ElevatorIOInputs();
    private double targetPosition = 0.0;

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void setTargetPosition(double position) {
        this.targetPosition = position;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        // Proportional control to adjust voltage
        double error = targetPosition - inputs.positionMeters;
        double outputVoltage = error * 5.0; // p-control (replace with a PID controller??)
        io.setVoltage(outputVoltage);

        // AdvantageKit shouldautomaticaly log the values in inputs
        Logger.recordOutput("Elevator/Position", inputs.positionMeters);
        Logger.recordOutput("Elevator/TargetPosition", targetPosition);
        Logger.recordOutput("Elevator/AppliedVoltage", inputs.appliedVoltage);
        Logger.recordOutput("Elevator/Velocity", inputs.velocityMetersPerSecond);
        Logger.recordOutput("Elevator/CurrentAmps", inputs.currentAmps[0]);
    }
}
