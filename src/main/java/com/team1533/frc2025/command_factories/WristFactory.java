// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.command_factories;

import com.team1533.frc2025.RobotContainer;
import com.team1533.frc2025.subsystems.wrist.WristConstants;
import com.team1533.frc2025.subsystems.wrist.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class WristFactory {

  private static final WristSubsystem wrist = RobotContainer.getInstance().getWristSubsystem();

  public static Command moveWristMotionMagic(DoubleSupplier setpoint) {
    return wrist
        .motionMagicPositionCommand(setpoint)
        .until(wrist.atSetpoint(WristConstants.toleranceRotations));
  }

  public static Command hold() {
    return wrist.holdSetpointCommand();
  }
}
