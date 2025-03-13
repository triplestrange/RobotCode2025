package com.team1533.frc2025.subsystems.wrist;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team1533.frc2025.RobotState;
import com.team1533.lib.loops.IStatusSignalLoop;
import com.team1533.lib.time.RobotTime;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class WristSubsystem extends SubsystemBase implements IStatusSignalLoop {

    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private volatile FastWristIOInputsAutoLogged fastInputs = new FastWristIOInputsAutoLogged();
    private final FastWristIOInputsAutoLogged cachedFastInputs = new FastWristIOInputsAutoLogged();
    private final RobotState state;

    private double wristSetpointRotations = 0.0;

    public WristSubsystem(final WristIO io) {
        this.io = io;
        setTeleopDefaultCommand();
        this.state = RobotState.getInstance();
    }

    @Override
    public List<BaseStatusSignal> getStatusSignals() {
        return io.getStatusSignals();
    }

    @Override
    public void onLoop() {
        io.updateFastInputs(fastInputs);
        double timestamp = RobotTime.getTimestampSeconds();
        state.addWristUpdate(timestamp, fastInputs.FusedCANcoderPositionRots);
    }

    public void setTeleopDefaultCommand() {
        this.setDefaultCommand(holdSetpointCommand()
                .withName("Wrist Maintain Setpoint (default)"));
    }

    public Command holdSetpointCommand() {
        return run(() -> {
            setMotionMagicSetpointImpl(wristSetpointRotations);
        }).withName("Wrist Maintain Setpoint");
    }

    public Command setSetpointHere() {
        return runOnce(
                () -> {
                    wristSetpointRotations = getCurrentPosition();
                }).withName("Wrist Set Setpoint Here");
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
        io.updateInputs(inputs);

        if (DriverStation.isDisabled()) {
            wristSetpointRotations = getCurrentPosition();
        }
        cachedFastInputs.FusedCANcoderPositionRots = getCurrentPosition();
        Logger.processInputs("Elevator/fastInputs", cachedFastInputs);
        Logger.recordOutput("Wrist/latencyPeriodicSec", RobotTime.getTimestampSeconds()
                - timestamp);
    }

    public Command moveWristSetpoint(DoubleSupplier rotationsFromHorizontal) {
        return runOnce(() -> {
            wristSetpointRotations = rotationsFromHorizontal.getAsDouble();
        });
    }

    public Command runDutyCycle(DoubleSupplier percentOutput) {
        return runEnd(
                () -> setDutyCycleOut(percentOutput.getAsDouble()),
                () -> setDutyCycleOut(0.0)).withName("Wrist DutyCycleControl");
    }

    public Command manualDutyCycle(DoubleSupplier percentOutput) {
        return run(
                () -> setDutyCycleOut(percentOutput.getAsDouble()));
    }

    private void setPositionSetpointImpl(double rotationsFromHorizontal, double rotationsPerSec) {
        Logger.recordOutput("Wrist/API/setPositionSetpoint/rotationsFromHorizontal",
                rotationsFromHorizontal);
        Logger.recordOutput("Wrist/API/setPositionSetpoint/rotationsPerSec",
                rotationsPerSec);
        io.setPositionSetpoint(rotationsFromHorizontal, rotationsPerSec);
    }

    private void setMotionMagicSetpointImpl(double rotationsFromHorizontal) {
        Logger.recordOutput("Wrist/API/setPositionSetpoint/rotationsFromHorizontal", rotationsFromHorizontal);
        io.setMotionMagicSetpoint(rotationsFromHorizontal);
    }

    private void setDutyCycleOut(double percentOutput) {
        io.setDutyCycleOut(percentOutput);
        wristSetpointRotations = getCurrentPosition();
    }

    public Command positionSetpointCommand(DoubleSupplier rotationsFromHorizontal, DoubleSupplier rotationsPerSec) {
        return run(() -> {
            double setpoint = rotationsFromHorizontal.getAsDouble();
            setPositionSetpointImpl(setpoint, rotationsPerSec.getAsDouble());
            wristSetpointRotations = setpoint;
        }).withName("Wrist positionSetpointCommand");
    }

    public Command motionMagicPositionCommand(DoubleSupplier rotationsFromHorizontal) {
        return run(() -> {
            double setpoint = rotationsFromHorizontal.getAsDouble();
            setMotionMagicSetpointImpl(setpoint);
            wristSetpointRotations = setpoint;
        }).withName("Wrist Motion Magic Setpoint Command");
    }

    public double getSetpoint() {
        return wristSetpointRotations;
    }

    public double getCurrentPosition() {
        return state.getLatestWristPositionRadians();
    }

    public Command waitForPosition(DoubleSupplier rotationsFromHorizontal, double toleranceRotations) {
        return new WaitUntilCommand(() -> {
            return Math.abs(getCurrentPosition() - rotationsFromHorizontal.getAsDouble()) < toleranceRotations;
        }).withName("Wrist wait for position");
    }

    public Command waitForSetpoint(double toleranceRotations) {
        return waitForPosition(this::getSetpoint, toleranceRotations);
    }

    public BooleanSupplier atSetpoint(double toleranceRotations) {
        return () -> MathUtil.isNear(getCurrentPosition(), getSetpoint(), toleranceRotations);
    }

    public double getCurrentPositionRotations() {
        return inputs.leaderRotPosition;
    }

    // TODO: implememt
    public void resetZeroPoint() {

    }

}
