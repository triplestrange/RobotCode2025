package com.team1533.frc2025.command_factories;

import java.util.function.DoubleSupplier;

import com.team1533.frc2025.RobotContainer;
import com.team1533.frc2025.subsystems.elevator.ElevatorConstants;
import com.team1533.frc2025.subsystems.elevator.ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorFactory {

    private static final ElevatorSubsystem elev = RobotContainer.getInstance().getElevatorSubsystem();

    public static Command moveArmMotionMagic(DoubleSupplier setpoint) {
        return elev.motionMagicPositionCommand(setpoint).until(elev.atSetpoint(ElevatorConstants.toleranceMeters));
    }

    public static Command hold() {
        return elev.holdSetpointCommand();
    }

    public static Command resetZero() {
        return elev.resetZeroPoint();
    }

}
