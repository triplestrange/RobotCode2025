package com.team1533.frc2025.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.team1533.frc2025.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import com.team1533.lib.time.RobotTime;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private double elevatorSetpointMeters = 0.0;

    public ElevatorSubsystem(final ElevatorIO io) {
        this.io = io;
    }

    public void setTeleopDefaultCommand() {
        this.setDefaultCommand(run(() -> {
            setPositionSetpointImpl(elevatorSetpointMeters, 0.0);
        }).withName("Hood Maintain Setpoint (default)"));
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        io.updateInputs(inputs);

        Logger.recordOutput("Elevator/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    }

    public Command moveElevatorSetpoint(DoubleSupplier metersFromBottom) {
        return runOnce(() -> {
            elevatorSetpointMeters = metersFromBottom.getAsDouble();
        });
    }

    public Command runDutyCycle(DoubleSupplier percentOutput) {
        return startEnd(
                () -> setDutyCycleOut(percentOutput.getAsDouble()),
                () -> setDutyCycleOut(0.0)).withName("Elevator DutyCycleControl");
    }

    private void setPositionSetpointImpl(double metersFromBottom, double metersPerSec) {
        Logger.recordOutput("Elevator/API/setPositionSetpoint/radiansFromCenter", metersFromBottom);
        Logger.recordOutput("Elevator/API/setPositionSetpoint/metersPerSec", metersPerSec);
        io.setPositionSetpoint(metersFromBottom, metersPerSec);
    }

    private void setDutyCycleOut(double percentOutput) {
        io.setDutyCycleOut(percentOutput);
    }

    public Command positionSetpointCommand(DoubleSupplier metersFromBottom, DoubleSupplier metersPerSec) {
        return run(() -> {
            double setpoint = metersFromBottom.getAsDouble();
            setPositionSetpointImpl(setpoint, metersPerSec.getAsDouble());
            elevatorSetpointMeters = setpoint;
        }).withName("Elevator positionSetpointCommand");
    }

    public double getSetpoint() {
        return elevatorSetpointMeters;
    }

    public double getCurrentPosition() {
        return inputs.elevatorPosMeters;
    }

    public Command waitForPosition(DoubleSupplier metersFromBottom, double toleranceMeters) {
        return new WaitUntilCommand(() -> {
            return Math.abs(getCurrentPosition() - metersFromBottom.getAsDouble()) < toleranceMeters;
        }).withName("Hood wait for position");
    }

    public double getCurrentPositionRotations() {
        return inputs.leaderRotPosition;
    }

    // TODO: implememt
    public void resetZeroPoint() {

    }
}
