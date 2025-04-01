// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.arm;

import com.team1533.frc2025.Constants;
import com.team1533.frc2025.Constants.Gains;

public class ArmConstants {
  public static final int leaderTalonCanID = 21;
  public static final int followerTalonCanID = 22;
  public static final int pivotEncoderCanID = 27;
  public static final String canBUS = "rio";

  public static final boolean leaderInverted = true;

  public static final double reduction = 240.;
  public static final double SensorToMechanismRatio = 1.0;
  public static final double frictionVoltage = 0.25;

  public static final double torqueCurrentLimit = 120;
  public static final double statorCurrentLimit = 120;
  public static final double supplyCurrentLimit = 60;
  public static final double supplyCurrentLowerLimit = 40;
  public static final double supplyCurrentLowerLimitTime = 1;

  public static final double motionMagicCruiseVelocity = 1;
  public static final double motionMagicAcceleration = 2;
  public static final double motionMagicJerk = 10;
  // TODO: Tune these tommorrow
  public static final double motionMagicExpo_kA = 12.0 / 2;
  public static final double motionMagicExpo_kV = 12.0 / 0.375;

  public static final double absEncoderOffset = -0.177246;
  public static final double absEncoderDiscontinuity = 0.8;

  public static final double reverseSoftLimitThreshold = 0.01;
  public static final double forwardSoftLimitThreshold = 0.25;

  public static final double toleranceRotations = 0.03;

  public static final Gains gains =
      switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(150, 0.0, 0, 0, 0.0, 0.0, 0);
        default -> new Gains(0, 0, 0, 0, 0, 0, 0);
      };
}
