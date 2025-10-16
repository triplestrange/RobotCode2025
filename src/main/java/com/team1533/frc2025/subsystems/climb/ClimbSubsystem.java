// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.climb;

import com.team1533.frc2025.RobotState;
import com.team1533.lib.subsystems.MotorIO;
import com.team1533.lib.subsystems.MotorInputsAutoLogged;
import com.team1533.lib.subsystems.ServoMotorSubsystem;
import com.team1533.lib.subsystems.ServoMotorSubsystemConfig;

public class ClimbSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  private final RobotState state;

  public ClimbSubsystem(ServoMotorSubsystemConfig c, final MotorIO io, final ClimbIO sensorIO) {
    super(c, new MotorInputsAutoLogged(), io);
    this.state = RobotState.getInstance();
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public void setTeleopDefaultCommand() {
    setDefaultCommand(dutyCycleCommand(() -> 0.0).withName("Zero intake Duty Cycle"));
  }
}
