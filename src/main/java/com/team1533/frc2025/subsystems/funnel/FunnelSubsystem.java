// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.funnel;

import com.team1533.frc2025.RobotState;
import com.team1533.lib.subsystems.MotorIO;
import com.team1533.lib.subsystems.MotorInputsAutoLogged;
import com.team1533.lib.subsystems.ServoMotorSubsystem;
import com.team1533.lib.subsystems.ServoMotorSubsystemConfig;

public class FunnelSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  private final RobotState state = RobotState.getInstance();

  public FunnelSubsystem(ServoMotorSubsystemConfig c, final MotorIO io) {
    super(c, new MotorInputsAutoLogged(), io);
    setTeleopDefaultCommand();
  }

  @Override
  public void periodic() {
    super.periodic();
    state.setFunnelRots(getCurrentPosition());
  }

  // TODO: change this if its too annoying
  public void setTeleopDefaultCommand() {
    setDefaultCommand(motionMagicSetpointCommand(() -> 0.0).withName("Stow Funnel"));
  }
}
