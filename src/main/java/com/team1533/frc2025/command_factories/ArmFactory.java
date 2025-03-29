package com.team1533.frc2025.command_factories;

import java.util.function.DoubleSupplier;

import com.team1533.frc2025.RobotContainer;
import com.team1533.frc2025.subsystems.arm.ArmConstants;
import com.team1533.frc2025.subsystems.arm.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmFactory {

    private static final ArmSubsystem arm = RobotContainer.getInstance().getArmSubsystem();

    public static Command moveArmMotionMagic(DoubleSupplier setpoint) {
        return arm.motionMagicPositionCommand(setpoint).until(arm.atSetpoint(ArmConstants.toleranceRotations));
    }

    public static Command moveArmDutyCycle(DoubleSupplier setpoint) {
        return arm.manualDutyCycle(setpoint);
    }

    public static Command hold() {
        return arm.holdSetpointCommand();
    }

}
