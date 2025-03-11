package com.team1533.frc2025.command_factories;

import java.util.function.DoubleSupplier;

import com.team1533.frc2025.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmFactory {

    private static final RobotContainer container = RobotContainer.getInstance();

    public static Command moveArmMotionMagic(DoubleSupplier setpoint) {
        return container.getArmSubsystem().motionMagicPositionCommand(setpoint);
    }

    public static Command moveArmDutyCycle(DoubleSupplier setpoint) {
        return container.getArmSubsystem().manualDutyCycle(setpoint);
    }

}
