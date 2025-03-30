// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team1533.frc2025.RobotState;
import com.team1533.lib.loops.IStatusSignalLoop;
import com.team1533.lib.time.RobotTime;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase implements IStatusSignalLoop {

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private volatile FastArmIOInputsAutoLogged fastInputs = new FastArmIOInputsAutoLogged();

  private final FastArmIOInputsAutoLogged cachedFastInputs = new FastArmIOInputsAutoLogged();

  private final RobotState state;
  @Getter private double armSetpointRotations = 0.0;

  public ArmSubsystem(final ArmIO io) {
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
    state.addArmUpdate(timestamp, fastInputs.FusedCANcoderPositionRots);
  }

  public void setTeleopDefaultCommand() {
    this.setDefaultCommand(holdSetpointCommand().withName("Arm Maintain Setpoint (default)"));
  }

  public Command holdSetpointCommand() {
    return run(() -> {
          setMotionMagicSetpointImpl(armSetpointRotations);
        })
        .withName("Arm Maintain Setpoint");
  }

  public Command setSetpointHere() {
    return runOnce(
            () -> {
              armSetpointRotations = getCurrentPosition();
            })
        .withName("Arm Set Setpoint Here");
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    cachedFastInputs.FusedCANcoderPositionRots = getCurrentPosition();

    if (DriverStation.isDisabled()) {
      armSetpointRotations = getCurrentPosition();
    }
    Logger.processInputs("Arm/fastInputs", cachedFastInputs);

    Logger.recordOutput("Arm/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
  }

  public Command moveArmSetpoint(DoubleSupplier rotationsFromHorizontal) {
    return runOnce(
        () -> {
          armSetpointRotations = rotationsFromHorizontal.getAsDouble();
        });
  }

  public Command runDutyCycle(DoubleSupplier percentOutput) {
    return runEnd(() -> setDutyCycleOut(percentOutput.getAsDouble()), () -> setDutyCycleOut(0.0))
        .withName("Arm DutyCycleControl");
  }

  public Command manualDutyCycle(DoubleSupplier percentOutput) {
    return run(() -> setDutyCycleOut(percentOutput.getAsDouble()));
  }

  private void setPositionSetpointImpl(double rotationsFromHorizontal, double rotationsPerSec) {
    Logger.recordOutput(
        "Arm/API/setPositionSetpoint/rotationsFromHorizontal", rotationsFromHorizontal);
    Logger.recordOutput("Arm/API/setPositionSetpoint/rotationsPerSec", rotationsPerSec);
    io.setPositionSetpoint(rotationsFromHorizontal, rotationsPerSec);
  }

  private void setMotionMagicSetpointImpl(double rotationsFromHorizontal) {
    Logger.recordOutput(
        "Arm/API/setPositionSetpoint/rotationsFromHorizontal", rotationsFromHorizontal);
    io.setMotionMagicSetpoint(rotationsFromHorizontal);
  }

  private void setDutyCycleOut(double percentOutput) {
    io.setDutyCycleOut(percentOutput);
    armSetpointRotations = getCurrentPosition();
  }

  public Command positionSetpointCommand(
      DoubleSupplier rotationsFromHorizontal, DoubleSupplier rotationsPerSec) {
    return run(() -> {
          double setpoint = rotationsFromHorizontal.getAsDouble();
          setPositionSetpointImpl(setpoint, rotationsPerSec.getAsDouble());
          armSetpointRotations = setpoint;
        })
        .withName("Arm positionSetpointCommand");
  }

  public Command motionMagicPositionCommand(DoubleSupplier rotationsFromHorizontal) {
    return run(() -> {
          double setpoint = rotationsFromHorizontal.getAsDouble();
          setMotionMagicSetpointImpl(setpoint);
          armSetpointRotations = setpoint;
        })
        .withName("Arm Motion Magic Setpoint Command");
  }

  public double getCurrentPosition() {
    return state.getLatestArmPositionRotations();
  }

  public Command waitForPosition(
      DoubleSupplier rotationsFromHorizontal, double toleranceRotations) {
    return new WaitUntilCommand(
            () -> {
              return Math.abs(getCurrentPosition() - rotationsFromHorizontal.getAsDouble())
                  < toleranceRotations;
            })
        .withName("Arm wait for position");
  }

  public Command waitForSetpoint(double toleranceRotations) {
    return waitForPosition(this::getArmSetpointRotations, toleranceRotations);
  }

  public BooleanSupplier atSetpoint(double toleranceRotations) {
    return () ->
        MathUtil.isNear(getCurrentPosition(), getArmSetpointRotations(), toleranceRotations);
  }

  public double getCurrentPositionRotations() {
    return inputs.leaderRotPosition;
  }
}
