// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

public interface MotorIO {
  void readInputs(MotorInputs inputs);

  void setOpenLoopDutyCycle(double dutyCycle);

  // These are in the "units" of the subsystem (rad, m).
  void setPositionSetpoint(double units);

  void setMotionMagicSetpoint(double units);

  void setNeutralMode(NeutralModeValue mode);

  void setVelocitySetpoint(double unitsPerSecond);

  void setCurrentPositionAsZero();

  void setCurrentPosition(double positionUnits);

  void setEnableSoftLimits(boolean forward, boolean reverse);
}
