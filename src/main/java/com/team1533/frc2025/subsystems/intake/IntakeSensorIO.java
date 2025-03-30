// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeSensorIO {
  @AutoLog
  public class IntakeSensorInputs {
    public boolean intakeLaserBlocked;
  }

  default void updateInputs(IntakeSensorIO.IntakeSensorInputs inputs) {}

  default DigitalInput getIntakeLaser() {
    return null;
  }
  ;
}
