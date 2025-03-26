// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.elevator;

import com.team1533.lib.time.RobotTime;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private double elevatorSetpointMeters = 0.0;

  public ElevatorSubsystem(final ElevatorIO io) {
    this.io = io;
    setTeleopDefaultCommand();
  }

  public void setTeleopDefaultCommand() {
    this.setDefaultCommand(holdSetpointCommand().withName("Elevator Maintain Setpoint (default)"));
  }

  public Command holdSetpointCommand() {
    return run(() -> {
          setMotionMagicSetpointImpl(elevatorSetpointMeters);
        })
        .withName("Elevator Maintain Setpoint");
  }

  public Command setSetpointHere() {
    return runOnce(
            () -> {
              elevatorSetpointMeters = getCurrentPosition();
            })
        .withName("Elevator Set Setpoint Here");
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    io.updateInputs(inputs);

    if (DriverStation.isDisabled()) {
      elevatorSetpointMeters = getCurrentPosition();
    }

    Logger.recordOutput("Elevator/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
  }

  public Command moveElevatorSetpoint(DoubleSupplier metersFromBottom) {
    return runOnce(
        () -> {
          elevatorSetpointMeters = metersFromBottom.getAsDouble();
        });
  }

  public Command runDutyCycle(DoubleSupplier percentOutput) {
    return runEnd(() -> setDutyCycleOut(percentOutput.getAsDouble()), () -> setDutyCycleOut(0.0))
        .withName("Elevator DutyCycleControl");
  }

  private void setPositionSetpointImpl(double metersFromBottom, double metersPerSec) {
    Logger.recordOutput("Elevator/API/setPositionSetpoint/metersFromBottom", metersFromBottom);
    Logger.recordOutput("Elevator/API/setPositionSetpoint/metersPerSec", metersPerSec);
    io.setPositionSetpoint(metersFromBottom, metersPerSec);
  }

  private void setMotionMagicSetpointImpl(double metersFromBottom) {
    Logger.recordOutput("Elevator/API/setPositionSetpoint/metersFromBottom", metersFromBottom);
    io.setMotionMagicSetpoint(metersFromBottom);
  }

  private void setDutyCycleOut(double percentOutput) {
    io.setDutyCycleOut(percentOutput);
    elevatorSetpointMeters = getCurrentPosition();
  }

  public Command positionSetpointCommand(
      DoubleSupplier metersFromBottom, DoubleSupplier metersPerSec) {
    return run(() -> {
          double setpoint = metersFromBottom.getAsDouble();
          setPositionSetpointImpl(setpoint, metersPerSec.getAsDouble());
          elevatorSetpointMeters = setpoint;
        })
        .withName("Elevator positionSetpointCommand");
  }

  public Command motionMagicPositionCommand(DoubleSupplier metersFromBottom) {
    return run(() -> {
          double setpoint = metersFromBottom.getAsDouble();
          setMotionMagicSetpointImpl(setpoint);
          elevatorSetpointMeters = setpoint;
        })
        .withName("Elevator Motion Magic Setpoint Command");
  }

  public double getSetpoint() {
    return elevatorSetpointMeters;
  }

  public double getCurrentPosition() {
    return inputs.elevatorPosMeters;
  }

  public Command waitForPosition(DoubleSupplier metersFromBottom, double toleranceMeters) {
    return new WaitUntilCommand(
            () -> {
              return Math.abs(getCurrentPosition() - metersFromBottom.getAsDouble())
                  < toleranceMeters;
            })
        .withName("Elevator wait for position");
  }

  public Command waitForSetpoint(double toleranceMeters) {
    return waitForPosition(this::getSetpoint, toleranceMeters);
  }

  public BooleanSupplier atSetpoint(double toleranceMeters) {
    return () -> MathUtil.isNear(getCurrentPosition(), getSetpoint(), toleranceMeters);
  }

  public double getCurrentPositionRotations() {
    return inputs.leaderRotPosition;
  }

  public Command resetZeroPoint() {

    return runEnd(
            (() -> io.setDutyCycleOutIgnoreLimits()),
            () -> {
              io.zero();
              elevatorSetpointMeters = 0;
            })
        .until(
            () ->
                (inputs.leaderStatorAmps > ElevatorConstants.blockedCurrent
                    && MathUtil.isNear(0, inputs.leaderVelocityRotPerSec, 0.1)))
        .withName("Elevator Zero Command");
  }
}
