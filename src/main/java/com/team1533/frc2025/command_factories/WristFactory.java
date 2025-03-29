package com.team1533.frc2025.command_factories;

import java.util.function.DoubleSupplier;

import com.team1533.frc2025.RobotContainer;
import com.team1533.frc2025.subsystems.wrist.WristConstants;
import com.team1533.frc2025.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class WristFactory {

    private static final WristSubsystem wrist = RobotContainer.getInstance().getWristSubsystem();

    public static Command moveWristMotionMagic(DoubleSupplier setpoint) {
        return wrist.motionMagicPositionCommand(setpoint).until(wrist.atSetpoint(WristConstants.toleranceRotations));
    }

    public static Command hold() {
        return wrist.holdSetpointCommand();
    }

}
