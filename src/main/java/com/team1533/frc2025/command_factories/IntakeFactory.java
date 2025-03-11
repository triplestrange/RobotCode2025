package com.team1533.frc2025.command_factories;

import java.util.function.DoubleSupplier;

import com.team1533.frc2025.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Factory class for creating commands related to the intake subsystem.
 *
 * <p>
 * 
 * This class provides a method to create a command for running the intake at a
 * 
 * specified speed.
 * The speed is provided through a {@link DoubleSupplier}, allowing for dynamic
 * adjustment of the intake's duty cycle output.
 */

public class IntakeFactory {
    private static final RobotContainer container = RobotContainer.getInstance();

    public static Command runIntake(DoubleSupplier intakeSpeed) {
        return container.getIntakeSubsystem().dutyCycleCommand(intakeSpeed).withName("Duty Cycle Intake");
    }
}