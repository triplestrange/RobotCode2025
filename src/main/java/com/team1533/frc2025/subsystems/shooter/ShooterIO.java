// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team1533.frc2025.Constants.Gains;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterIOInputs {
    public boolean leaderConnected = false;
    public double leaderVelocityRotPerSec = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;
    public double leaderStatorAmps = 0.0;
    public double leaderTempCelc = 0.0;

    public boolean follower1Connected = false;
    public double follower1VelocityRotPerSec = 0.0;
    public double follower1AppliedVolts = 0.0;
    public double follower1CurrentAmps = 0.0;
    public double follower1StatorAmps = 0.0;
    public double follower1TempCelc = 0.0;

    public boolean follower2Connected = false;
    public double follower2VelocityRotPerSec = 0.0;
    public double follower2AppliedVolts = 0.0;
    public double follower2CurrentAmps = 0.0;
    public double follower2StatorAmps = 0.0;
    public double follower2TempCelc = 0.0;

  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void runVolts(double volts) {}

  default void setDutyCycleOut(double output) {}

  default void setVelocitySetpoint(double velocity) {}

  default void setBrakeMode(boolean enabled) {}

  default void setPID(Gains gains) {}

  default void stop() {}
}
