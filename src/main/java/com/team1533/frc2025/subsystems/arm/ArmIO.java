// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team1533.frc2025.Constants.Gains;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class FastArmIOInputs {
    public double FusedCANcoderPositionRots = 0.0;
  }

  @AutoLog
  class ArmIOInputs {
    public boolean leaderConnected = false;
    public double leaderVelocityRotPerSec = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;
    public double leaderStatorAmps = 0.0;
    public double leaderTempCelc = 0.0;

    public boolean followerConnected = false;
    public double followerVelocityRotPerSec = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerCurrentAmps = 0.0;
    public double followerStatorAmps = 0.0;
    public double followerTempCelc = 0.0;

    public boolean absoluteEncoderConnected = true;
    public double absoluteEncoderPositionRots = 0.0;
    public double relativeEncoderPositionRots = 0.0;

    public double leaderRotPosition = 0.0;
    public double followerRotPosition = 0.0;

    public double armVelMetersPerSecond = 0.0;
    public double armAccelMetersPerSecondPerSecond = 0.0;
  }

  default List<BaseStatusSignal> getStatusSignals() {
    return Arrays.asList();
  }
  ;

  default void updateInputs(ArmIOInputs inputs) {}

  default void updateFastInputs(FastArmIOInputs inputs) {}

  default void runVolts(double volts) {}

  default void setDutyCycleOut(double output) {}

  default void setPositionSetpoint(double positionRotation) {}

  default void setPositionSetpoint(double positionRotation, double rotationsPerSec) {}

  default void setCurrentSetpoint(double amps) {}

  default void setBrakeMode(boolean enabled) {}

  default void setMotionMagicSetpoint(double positionRotations) {}

  default void setPID(Gains gains) {}

  default void stop() {}
}
