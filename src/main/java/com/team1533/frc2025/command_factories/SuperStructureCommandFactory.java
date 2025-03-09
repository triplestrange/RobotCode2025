package com.team1533.frc2025.command_factories;

import com.team1533.frc2025.RobotContainer;
import com.team1533.frc2025.subsystems.arm.ArmSubsystem;
import com.team1533.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team1533.frc2025.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SuperStructureCommandFactory {

        private static final ArmSubsystem arm = RobotContainer.getInstance().getArmSubsystem();
        private static final ElevatorSubsystem elevator = RobotContainer.getInstance().getElevatorSubsystem();
        private static final WristSubsystem wrist = RobotContainer.getInstance().getWristSubsystem();

// Generic Preset - Sets the SuperStructure to Neutral Pos, the moves it to a setpoint
    public static Command genericPreset(
            double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {

        return new SequentialCommandGroup(
                moveArmOnly(0.21).until(arm.atSetpoint(.03)),
                moveWristOnly( 0.22).until(wrist.atSetpoint(.02)),
                moveElevatorOnly(elevatorSetpointMeters).until(elevator.atSetpoint(.02)),
                moveWristOnly(wristSetpointRotations).until(wrist.atSetpoint(.02)),
                moveArmOnly(armSetpointRotations).until(arm.atSetpoint(.03)));
    }

// Neutral Pos - Default Pos of the SuperStructure. Elevator setpoint can be any number within travel distance.
    public static Command neutralPos(
            double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {

        return new SequentialCommandGroup(
                moveArmOnly(0.21).until(arm.atSetpoint(.03)),
                moveWristOnly( 0.22).until(wrist.atSetpoint(.02)),
                moveElevatorOnly(elevatorSetpointMeters).until(elevator.atSetpoint(.02)));
    }

// Forced Pos - Immediately Sets the SuperStructure to a setpoint. To be used in auto, ONLY AFTER NEUTRAL POS.
    public static Command forcedPos(
            double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {

        return new SequentialCommandGroup(
                moveElevatorOnly(elevatorSetpointMeters).until(elevator.atSetpoint(.02)),
                moveWristOnly(wristSetpointRotations).until(wrist.atSetpoint(.02)),
                moveArmOnly(armSetpointRotations).until(arm.atSetpoint(.03)));
    }

    public static Command zeroElevator()   {
        return new SequentialCommandGroup(
        moveArmOnly(0.21).until(arm.atSetpoint(0.03)),
        moveWristOnly(0.22).until(wrist.atSetpoint(0.02)),
        new ParallelDeadlineGroup(elevator.resetZeroPoint(), arm.holdSetpointCommand(),
        wrist.holdSetpointCommand()));
    }

    // Climb Sequence?

    public static Command climbSequence() {

        return   moveArmOnly(0.125).until(arm.atSetpoint(0.03)).andThen(new ParallelCommandGroup(
                arm.motionMagicPositionCommand(() -> 0),
                wrist.motionMagicPositionCommand(() -> 0),
                elevator.motionMagicPositionCommand(() -> 0.4)));
        }

        // return new SequentialCommandGroup(
        // moveArmOnly(arm, elevator, wrist,
        // 0.22).withDeadline(Commands.waitSeconds(3)),
        // moveWristOnly(arm, elevator, wrist,
        // 0.337).withDeadline(Commands.waitSeconds(3)),
        // moveElevatorOnly(arm, elevator, wrist,
        // elevatorSetpointMeters).withDeadline(Commands.waitSeconds(3)),
        // moveWristOnly(arm, elevator, wrist,
        // wristSetpointRotations).withDeadline(Commands.waitSeconds(3)),
        // moveArmOnly(arm, elevator, wrist,
        // armSetpointRotations).withDeadline(Commands.waitSeconds(3))
        // );

    public static Command moveArmOnly(
            double armSetpointRotations) {

        return new ParallelCommandGroup(
                arm.motionMagicPositionCommand(() -> armSetpointRotations),
                elevator.holdSetpointCommand(),
                wrist.holdSetpointCommand());
    }

    public static Command moveWristOnly(
            double wristSetpointRotations) {

        return new ParallelCommandGroup(
                arm.holdSetpointCommand(),
                elevator.holdSetpointCommand(),
                wrist.motionMagicPositionCommand(() -> wristSetpointRotations));
    }

    public static Command moveElevatorOnly(
            double elevatorSetpointMeters) {

        return new ParallelCommandGroup(
                arm.holdSetpointCommand(),
                elevator.motionMagicPositionCommand(() -> elevatorSetpointMeters),
                wrist.holdSetpointCommand());
    }

}
