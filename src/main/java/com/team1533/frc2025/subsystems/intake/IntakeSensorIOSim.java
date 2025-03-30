// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.intake;

import edu.wpi.first.hal.simulation.DIODataJNI;

public class IntakeSensorIOSim extends IntakeSensorIOReal {
  public IntakeSensorIOSim() {
    super();
  }

  public void setNotBlocked() {
    DIODataJNI.setValue(this.getIntakeLaser().getChannel(), false);
  }

  public void setBlocked() {
    DIODataJNI.setValue(this.getIntakeLaser().getChannel(), true);
  }
}
