// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

public interface MotorIO {
  default void updateInputs(MotorInputs inputs) {};

  default void setOpenLoopDutyCycle(double dutyCycle) {};

  // These are in the "units" of the subsystem (rad, m).
  default void setPositionSetpoint(double units) {};

  default void setMotionMagicSetpoint(double units) {};

  default void setNeutralMode(NeutralModeValue mode) {};

  default void setVelocitySetpoint(double unitsPerSecond) {};

  default void setCurrentPositionAsZero() {};

  default void setCurrentPosition(double positionUnits) {};

  default void setEnableSoftLimits(boolean forward, boolean reverse) {};
}
