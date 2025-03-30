// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.command_factories;

import com.team1533.frc2025.RobotContainer;
import com.team1533.frc2025.subsystems.funnel.FunnelConstants;
import com.team1533.frc2025.subsystems.funnel.FunnelSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class FunnelFactory {
  private static final FunnelSubsystem funnel = RobotContainer.getInstance().getFunnelSubsystem();

  public static Command moveFunnelMotionMagic(DoubleSupplier setpoint) {
    return funnel
        .motionMagicSetpointCommand(setpoint)
        .until(
            () ->
                MathUtil.isNear(
                    setpoint.getAsDouble(),
                    funnel.getCurrentPosition(),
                    FunnelConstants.toleranceRotations));
  }

  public static Command moveArmDutyCycle(DoubleSupplier setpoint) {
    return funnel.dutyCycleCommand(setpoint);
  }

  public static Command hold() {
    return funnel.holdSetpointCommand();
  }
}
