// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeSensorIOReal implements IntakeSensorIO {

  protected final DigitalInput intakeLaser;

  public IntakeSensorIOReal() {
    intakeLaser = new DigitalInput(IntakeConstants.kIntakeLaserSensorPort);
  }

  @Override
  public void updateInputs(IntakeSensorInputs inputs) {
    inputs.intakeLaserBlocked = !intakeLaser.get();
  }

  @Override
  public DigitalInput getIntakeLaser() {
    return intakeLaser;
  }
}
