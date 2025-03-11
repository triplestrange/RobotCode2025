package com.team1533.frc2025.command_factories;

import java.util.function.DoubleSupplier;

import com.team1533.frc2025.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorFactory {

    private static final RobotContainer container = RobotContainer.getInstance();

    public static Command moveArmMotionMagic(DoubleSupplier setpoint) {
        return container.getElevatorSubsystem().motionMagicPositionCommand(setpoint);
    }

}
