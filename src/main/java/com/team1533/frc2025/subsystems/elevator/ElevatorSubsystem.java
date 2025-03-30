// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team1533.frc2025.RobotState;
import com.team1533.lib.loops.IStatusSignalLoop;
import com.team1533.lib.time.RobotTime;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase implements IStatusSignalLoop {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private volatile FastElevatorIOInputsAutoLogged fastInputs = new FastElevatorIOInputsAutoLogged();
  private final FastElevatorIOInputsAutoLogged cachedFastInputs =
      new FastElevatorIOInputsAutoLogged();

  private final RobotState state;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  public double currentFilterValue = 0.0;
  @Getter private boolean zerod = false;
  @Getter private double elevatorSetpointMeters = 0.0;

  public ElevatorSubsystem(final ElevatorIO io) {
    this.io = io;
    this.state = RobotState.getInstance();
    setTeleopDefaultCommand();
  }

  @Override
  public List<BaseStatusSignal> getStatusSignals() {
    return io.getStatusSignals();
  }

  @Override
  public void onLoop() {
    io.updateFastInputs(fastInputs);
    double timestamp = RobotTime.getTimestampSeconds();
    state.addElevUpdate(timestamp, fastInputs.elevatorPosMeters);
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
    currentFilterValue = currentFilter.calculate(inputs.leaderStatorAmps);
    Logger.recordOutput("Elevator/Filtered Current", currentFilterValue);

    if (DriverStation.isDisabled()) {
      elevatorSetpointMeters = getCurrentPosition();
    }
    cachedFastInputs.elevatorPosMeters = getCurrentPosition();
    Logger.processInputs("Elevator/fastInputs", cachedFastInputs);
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

  public double getCurrentPosition() {
    return state.getLatestElevPositionMeters();
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
    return waitForPosition(this::getElevatorSetpointMeters, toleranceMeters);
  }

  public BooleanSupplier atSetpoint(double toleranceMeters) {
    return () ->
        MathUtil.isNear(getCurrentPosition(), getElevatorSetpointMeters(), toleranceMeters);
  }

  public double getCurrentPositionRotations() {
    return inputs.leaderRotPosition;
  }

  public Command resetZeroPoint() {

    return runEnd(
            (() -> io.setDutyCycleOutIgnoreLimits()),
            () -> {
              io.zero();
              zerod = true;
            })
        .until(
            () ->
                (currentFilterValue > ElevatorConstants.blockedCurrent
                    && MathUtil.isNear(0, inputs.leaderVelocityRotPerSec, 0.1)))
        .withName("Elevator Zero Command");
  }
}
