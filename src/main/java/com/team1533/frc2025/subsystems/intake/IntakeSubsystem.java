// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team1533.frc2025.RobotState;
import com.team1533.lib.loops.IStatusSignalLoop;
import com.team1533.lib.subsystems.MotorIO;
import com.team1533.lib.subsystems.MotorInputsAutoLogged;
import com.team1533.lib.subsystems.ServoMotorSubsystem;
import com.team1533.lib.subsystems.ServoMotorSubsystemConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO>
    implements IStatusSignalLoop {

  private IntakeSensorInputsAutoLogged inputsSensors = new IntakeSensorInputsAutoLogged();
  private IntakeSensorIO ioSensors;
  private AtomicBoolean laserHasReef = new AtomicBoolean(false);

  private final RobotState state;

  private Debouncer laserDebouncer =
      new Debouncer(IntakeConstants.kIntakeLaserDebounceTime, DebounceType.kRising);

  public IntakeSubsystem(
      ServoMotorSubsystemConfig c, final MotorIO io, final IntakeSensorIO sensorIO) {
    super(c, new MotorInputsAutoLogged(), io);
    this.ioSensors = sensorIO;
    this.state = RobotState.getInstance();
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public void setTeleopDefaultCommand() {
    setDefaultCommand(dutyCycleCommand(() -> 0.0).withName("Zero intake Duty Cycle"));
  }

  public boolean hasReefAtBannerLaser() {
    return inputsSensors.intakeLaserBlocked;
  }

  @Override
  public List<BaseStatusSignal> getStatusSignals() {
    return new ArrayList<>();
  }

  @Override
  public void onLoop() {
    ioSensors.updateInputs(inputsSensors);
    laserHasReef.set(laserDebouncer.calculate(inputsSensors.intakeLaserBlocked));
    state.updateLastTriggeredIntakeLaserTimestamp(laserHasReef.get());
    Logger.recordOutput(
        "Intake/lastTriggeredLaserTimestamp", state.getLastTriggeredIntakeLaserTimestamp());
    Logger.processInputs(getName() + "/sensors", inputsSensors);
  }
}
