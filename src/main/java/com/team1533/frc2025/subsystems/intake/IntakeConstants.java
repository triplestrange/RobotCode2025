// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.intake;

public class IntakeConstants {
  public static final int intakeTalonCanID = 21;
  public static final int rRollerTalonCanID = 22;
  public static final int lRollerTalonCanID = 23;
  public static final int fCANrangeCanID = 24;
  public static final int rCANrangeCanID = 25;
  public static final int lCANrangeCanID = 26;
  public static final int bCANrangeCanID = 27;
  public static final String canBUS = "rio";

  //Tune? Check values
  public static final double intakeStatorCurrentLimit = 200;
  public static final double intakeSupplyCurrentLimit = 40;

  public static final double rollerStatorCurrentLimit = 200;
  public static final double rollerSupplyCurrentLimit = 40;
}