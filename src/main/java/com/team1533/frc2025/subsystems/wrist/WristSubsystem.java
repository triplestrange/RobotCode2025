package com.team1533.frc2025.subsystems.wrist;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.team1533.lib.time.RobotTime;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class WristSubsystem extends SubsystemBase {

    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private double wristSetpointRotations = 0.0;

    public WristSubsystem(final WristIO io) {
        this.io = io;
    }

    public void setTeleopDefaultCommand() {
        this.setDefaultCommand(run(() -> {
            setPositionSetpointImpl(wristSetpointRotations, 0.0);
        }).withName("Wrist Maintain Setpoint (default)"));
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
        io.updateInputs(inputs);

        Logger.recordOutput("Wrist/latencyPeriodicSec", RobotTime.getTimestampSeconds()
                - timestamp);
    }

    public Command moveWristSetpoint(DoubleSupplier rotationsFromHorizontal) {
        return runOnce(() -> {
            wristSetpointRotations = rotationsFromHorizontal.getAsDouble();
        });
    }

    public Command runDutyCycle(DoubleSupplier percentOutput) {
        return startEnd(
                () -> setDutyCycleOut(percentOutput.getAsDouble()),
                () -> setDutyCycleOut(0.0)).withName("Wrist DutyCycleControl");
    }

    private void setPositionSetpointImpl(double rotationsFromHorizontal, double rotationsPerSec) {
        Logger.recordOutput("Wrist/API/setPositionSetpoint/rotationsFromHorizontal",
                rotationsFromHorizontal);
        Logger.recordOutput("Wrist/API/setPositionSetpoint/rotationsPerSec",
                rotationsPerSec);
        io.setPositionSetpoint(rotationsFromHorizontal, rotationsPerSec);
    }

    private void setDutyCycleOut(double percentOutput) {
        io.setDutyCycleOut(percentOutput);
    }

    public Command positionSetpointCommand(DoubleSupplier rotationsFromHorizontal, DoubleSupplier rotationsPerSec) {
        return run(() -> {
            double setpoint = rotationsFromHorizontal.getAsDouble();
            setPositionSetpointImpl(setpoint, rotationsPerSec.getAsDouble());
            wristSetpointRotations = setpoint;
        }).withName("Wrist positionSetpointCommand");
    }

    public double getSetpoint() {
        return wristSetpointRotations;
    }

    public double getCurrentPosition() {
        return inputs.absoluteEncoderPositionRots;
    }

    // public Command waitForPosition(DoubleSupplier rotationsFromHorizontal,
    // double
    // toleranceRotations) {
    // return new WaitUntilCommand(() -> {
    // return Math.abs(getCurrentPosition() -
    // rotationsFromHorizontal.getAsDouble())
    // < toleranceMeters;
    // }).withName("Wrist wait for position");
    // }

    public double getCurrentPositionRotations() {
        return inputs.leaderRotPosition;
    }

    // TODO: implememt
    public void resetZeroPoint() {

    }

}
