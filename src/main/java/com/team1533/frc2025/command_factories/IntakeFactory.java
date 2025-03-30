// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.command_factories;

import com.team1533.frc2025.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

/**
 * Factory class for creating commands related to the intake subsystem.
 *
 * <p>This class provides a method to create a command for running the intake at a
 *
 * <p>specified speed. The speed is provided through a {@link DoubleSupplier}, allowing for dynamic
 * adjustment of the intake's duty cycle output.
 */
public class IntakeFactory {
  private static final RobotContainer container = RobotContainer.getInstance();

  public static Command runIntake(DoubleSupplier intakeSpeed) {
    return container
        .getIntakeSubsystem()
        .dutyCycleCommand(intakeSpeed)
        .withName("Duty Cycle Intake");
  }
}
