// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.command_factories;

import com.team1533.frc2025.Constants.SuperStructureStates;
import com.team1533.frc2025.RobotContainer;
import com.team1533.frc2025.subsystems.elevator.ElevatorConstants;
import com.team1533.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team1533.frc2025.subsystems.funnel.FunnelSubsystem;
import com.team1533.frc2025.subsystems.wrist.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ElevatorFactory {

  private static final ElevatorSubsystem elev = RobotContainer.getInstance().getElevatorSubsystem();
  private static final WristSubsystem wrist = RobotContainer.getInstance().getWristSubsystem();
  private static final FunnelSubsystem funnel = RobotContainer.getInstance().getFunnelSubsystem();

  public static final BooleanSupplier funnelBlocksElevator =
      () ->
          (funnel.getCurrentPosition() < 0.078
              && (elev.getCurrentPosition() <= 0.151 || elev.getElevatorSetpointMeters() <= 0.151));

  public static Command moveElevMotionMagic(DoubleSupplier setpoint) {

    return elev.moveElevatorSetpoint(setpoint)
        .andThen(
            elev.motionMagicPositionCommand(setpoint)
                .until(elev.atSetpoint(ElevatorConstants.toleranceMeters)));
  }

  public static Command moveElevMotionMagic(SuperStructureStates setpoint) {

    return elev.moveElevatorSetpoint(setpoint.getState()::elevGoalMeters)
        .andThen(
            elev.motionMagicPositionCommand(setpoint.getState()::elevGoalMeters)
                .until(elev.atSetpoint(ElevatorConstants.toleranceMeters)));
  }

  public static Command hold() {
    return elev.holdSetpointCommand();
  }

  public static Command resetZero() {
    return elev.resetZeroPoint();
  }
}
