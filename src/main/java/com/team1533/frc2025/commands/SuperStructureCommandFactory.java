package com.team1533.frc2025.commands;

import com.team1533.frc2025.subsystems.arm.ArmSubsystem;
import com.team1533.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team1533.frc2025.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SuperStructureCommandFactory {

    public static Command genericPreset(ArmSubsystem arm, ElevatorSubsystem elevator, WristSubsystem wrist,
            double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {
        
        return new SequentialCommandGroup(
            moveArmOnly(arm, elevator, wrist, 0.21).until(arm.atSetpoint(.03)),
            moveWristOnly(arm, elevator, wrist, 0.22).until(wrist.atSetpoint(.02)),
            moveElevatorOnly(arm, elevator, wrist, elevatorSetpointMeters).until(elevator.atSetpoint(.02)),
            moveWristOnly(arm, elevator, wrist, wristSetpointRotations).until(wrist.atSetpoint(.02)),
            moveArmOnly(arm, elevator, wrist, armSetpointRotations).until(arm.atSetpoint(.03))
        );

        // return new SequentialCommandGroup(
        //     moveArmOnly(arm, elevator, wrist, 0.22).withDeadline(Commands.waitSeconds(3)),
        //     moveWristOnly(arm, elevator, wrist, 0.337).withDeadline(Commands.waitSeconds(3)),
        //     moveElevatorOnly(arm, elevator, wrist, elevatorSetpointMeters).withDeadline(Commands.waitSeconds(3)),
        //     moveWristOnly(arm, elevator, wrist, wristSetpointRotations).withDeadline(Commands.waitSeconds(3)),
        //     moveArmOnly(arm, elevator, wrist, armSetpointRotations).withDeadline(Commands.waitSeconds(3))
        // );
    }

    public static Command moveArmOnly(ArmSubsystem arm, ElevatorSubsystem elevator, WristSubsystem wrist,
            double armSetpointRotations) {

        return new ParallelCommandGroup(
            arm.motionMagicPositionCommand(() -> armSetpointRotations),
            elevator.holdSetpointCommand(),
            wrist.holdSetpointCommand()
        );
    }

    public static Command moveWristOnly(ArmSubsystem arm, ElevatorSubsystem elevator, WristSubsystem wrist,
            double wristSetpointRotations) {

        return new ParallelCommandGroup(
            arm.holdSetpointCommand(),
            elevator.holdSetpointCommand(),
            wrist.motionMagicPositionCommand(() -> wristSetpointRotations)
        );
    }

    public static Command moveElevatorOnly(ArmSubsystem arm, ElevatorSubsystem elevator, WristSubsystem wrist,
            double elevatorSetpointMeters) {

        return new ParallelCommandGroup(
            arm.holdSetpointCommand(),
            elevator.motionMagicPositionCommand(() -> elevatorSetpointMeters),
            wrist.holdSetpointCommand()
        );
    }

}
