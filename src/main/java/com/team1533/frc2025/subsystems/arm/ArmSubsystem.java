package com.team1533.frc2025.subsystems.arm;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.team1533.lib.time.RobotTime;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ArmSubsystem extends SubsystemBase {

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private double armSetpointRotations = 0.0;

    public ArmSubsystem(final ArmIO io) {
        this.io = io;
    }

    public void setTeleopDefaultCommand() {
        this.setDefaultCommand(run(() -> {
            setPositionSetpointImpl(armSetpointRotations, 0.0);
        }).withName("Arm Maintain Setpoint (default)"));
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        io.updateInputs(inputs);

        Logger.recordOutput("Arm/latencyPeriodicSec", RobotTime.getTimestampSeconds()
                - timestamp);
    }

    public Command moveArmSetpoint(DoubleSupplier rotationsFromHorizontal) {
        return runOnce(() -> {
            armSetpointRotations = rotationsFromHorizontal.getAsDouble();
        });
    }

    public Command runDutyCycle(DoubleSupplier percentOutput) {
        return startEnd(
                () -> setDutyCycleOut(percentOutput.getAsDouble()),
                () -> setDutyCycleOut(0.0)).withName("Arm DutyCycleControl");
    }

    public Command manualDutyCycle(DoubleSupplier percentOutput) {
        return run(
                () -> setDutyCycleOut(percentOutput.getAsDouble()));    
            }

    private void setPositionSetpointImpl(double rotationsFromHorizontal, double rotationsPerSec) {
        Logger.recordOutput("Arm/API/setPositionSetpoint/rotationsFromHorizontal",
                rotationsFromHorizontal);
        Logger.recordOutput("Arm/API/setPositionSetpoint/rotationsPerSec",
                rotationsPerSec);
        io.setPositionSetpoint(rotationsFromHorizontal, rotationsPerSec);
    }

    private void setMotionMagicSetpointImpl(double rotationsFromHorizontal)   {
        Logger.recordOutput("Arm/API/setPositionSetpoint/rotationsFromHorizontal", rotationsFromHorizontal);
        io.setMotionMagicSetpoint(rotationsFromHorizontal);    
    }

    private void setDutyCycleOut(double percentOutput) {
        io.setDutyCycleOut(percentOutput);
    }

    public Command positionSetpointCommand(DoubleSupplier rotationsFromHorizontal, DoubleSupplier rotationsPerSec) {
        return run(() -> {
            double setpoint = rotationsFromHorizontal.getAsDouble();
            setPositionSetpointImpl(setpoint, rotationsPerSec.getAsDouble());
            armSetpointRotations = setpoint;
        }).withName("Arm positionSetpointCommand");
    }

    public Command motionMagicPositionCommand(DoubleSupplier rotationsFromHorizontal) {
        return run(() -> {
            double setpoint = rotationsFromHorizontal.getAsDouble();
            setMotionMagicSetpointImpl(setpoint);
            armSetpointRotations = setpoint;
        }).withName("Arm Motion Magic Setpoint Command");
    } 

    public double getSetpoint() {
        return armSetpointRotations;
    }

    public double getCurrentPosition() {
        return inputs.absoluteEncoderPositionRots;
    }

    public Command waitForPosition(DoubleSupplier rotationsFromHorizontal, double toleranceRotations) {
        return new WaitUntilCommand(() -> {
            return Math.abs(getCurrentPosition() - rotationsFromHorizontal.getAsDouble()) < toleranceRotations;
        }).withName("Arm wait for position");
    }

    public Command waitForSetpoint(double toleranceRotations) {
        return waitForPosition(this::getSetpoint, toleranceRotations);
    }

    public double getCurrentPositionRotations() {
        return inputs.leaderRotPosition;
    }
}
