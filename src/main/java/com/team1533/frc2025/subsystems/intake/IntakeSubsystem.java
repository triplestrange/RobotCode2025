// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.intake;

import com.team1533.lib.subsystems.MotorIO;
import com.team1533.lib.subsystems.MotorInputsAutoLogged;
import com.team1533.lib.subsystems.ServoMotorSubsystem;
import com.team1533.lib.subsystems.ServoMotorSubsystemConfig;

public class IntakeSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  public IntakeSubsystem(ServoMotorSubsystemConfig c, final MotorIO io) {
    super(c, new MotorInputsAutoLogged(), io);
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
