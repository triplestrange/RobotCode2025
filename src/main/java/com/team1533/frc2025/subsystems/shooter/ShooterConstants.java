// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.shooter;

import com.team1533.frc2025.Constants;
import com.team1533.frc2025.Constants.Gains;

public class ShooterConstants {
  public static final int leaderTalonCanID = 40;
  public static final int follower1TalonCanID = 41;
  public static final int follower2TalonCanID = 42;
  public static final String canBUS = "DriveTrain";

  public static final boolean leaderInverted = true;

  public static final double reduction = 1.0;
  public static final double SensorToMechanismRatio = 1.0;

  public static final double torqueCurrentLimit = 120;
  public static final double statorCurrentLimit = 120;
  public static final double supplyCurrentLimit = 60;
  public static final double supplyCurrentLowerLimit = 40;
  public static final double supplyCurrentLowerLimitTime = 1;

  public static final Gains gains =
      switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(0, 0.0, 0, 0, 0.0, 0.0, 0);
        default -> new Gains(0, 0, 0, 0, 0, 0, 0);
      };
}
