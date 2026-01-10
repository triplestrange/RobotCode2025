// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.command_factories;

import com.team1533.frc2025.RobotContainer;
import com.team1533.frc2025.subsystems.climb.ClimbSubsystem;
import com.team1533.frc2025.subsystems.intake.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import com.team1533.frc2025.subsystems.arm.*;

public class SuperStructureCommandFactory {

  private static final RobotContainer container = RobotContainer.getInstance();

//Triggers to find:
//Make Wrist move to 0.05 whenever elevator is moving
//Ensure that robot transitions to default state between intaking and scoring
//Ensure that robot clears the reef when transitioning between levels

// Intake Pos
//       Arm (-4.8/360),
//       Wrist (0.374),
//       Elevator (0.065)

//  Stowed Pos
//       Arm (-4.8/360)
//       Wrist (0)
//       Elevator (0)

  public static Command zeroElevator() {
    return new SequentialCommandGroup(
        moveArmOnly(0.21).until(container.getArmSubsystem().atSetpoint(0.03)),
        moveWristOnly(0.22).until(container.getWristSubsystem().atSetpoint(0.02)),
        new ParallelDeadlineGroup(
            container.getElevatorSubsystem().resetZeroPoint(),
            container.getArmSubsystem().holdSetpointCommand(),
            container.getWristSubsystem().holdSetpointCommand()));
    }

  public static Command stow() {

    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            ArmFactory.moveArmMotionMagic(() -> -5.0/360),
            ElevatorFactory.moveElevMotionMagic(() -> 0),
            WristFactory.moveWristMotionMagic(() -> 0.05))
            .andThen(moveWristOnly(0)));
  }

  public static Command defaultPos() {
    
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            ArmFactory.moveArmMotionMagic(() -> 0.15625),
            ElevatorFactory.moveElevMotionMagic(() -> 0),
            WristFactory.moveWristMotionMagic(() -> 0.067)));
  }

  public static Command intakingIsBad(double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {
    return new SequentialCommandGroup(
      moveElevatorOnly(elevatorSetpointMeters),
      new ParallelCommandGroup(
        ArmFactory.moveArmMotionMagic(() -> armSetpointRotations),
        WristFactory.moveWristMotionMagic(() -> wristSetpointRotations)
      )
    );
  }

  public static Command defaultParallelPreset(double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) { 

    return new ParallelCommandGroup(
        ArmFactory.moveArmMotionMagic(() -> armSetpointRotations),
        WristFactory.moveWristMotionMagic(() -> wristSetpointRotations),
        ElevatorFactory.moveElevMotionMagic(() -> elevatorSetpointMeters));
        //.until(container.getArmSubsystem().atSetpoint(ArmConstants.toleranceRotations))
    }

  public static Command scoringParallelPreset(double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations, boolean isFacingForward) { 

    return new ParallelCommandGroup(
        ArmFactory.moveArmMotionMagic(() -> armSetpointRotations),
        WristFactory.moveWristMotionMagic(() -> wristSetpointRotations),
        ElevatorFactory.moveElevMotionMagic(() -> elevatorSetpointMeters))
        .raceWith(container.getIntakeSubsystem().coralShiftCommand(isFacingForward));
        //.until(container.getArmSubsystem().atSetpoint(ArmConstants.toleranceRotations))
      }

  public static Command algaeSucks(double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {
    return new SequentialCommandGroup(
      moveElevatorOnly(0.1),
      new ParallelCommandGroup(
        ArmFactory.moveArmMotionMagic(() -> armSetpointRotations),
        ElevatorFactory.moveElevMotionMagic(() -> elevatorSetpointMeters),
        WristFactory.moveWristMotionMagic(() -> wristSetpointRotations)
      )
    );
  }

  public static Command climb() {
    return new SequentialCommandGroup(
      new ParallelCommandGroup(
        ArmFactory.moveArmMotionMagic(() -> 95.0/360),
        ElevatorFactory.moveElevMotionMagic(() -> 0.232),
        WristFactory.moveWristMotionMagic(() -> 0.25)
        ),
      moveWristOnly(0.55),
      container.getClimbSubsystem().runUntilStall(),
      new ParallelCommandGroup(
        ArmFactory.moveArmMotionMagic(() -> 10.0/360),
        WristFactory.moveWristMotionMagic(() -> 0.27)
        ),
      new ParallelCommandGroup(
        ArmFactory.moveArmMotionMagic(() -> -5.0/360),
        ElevatorFactory.moveElevMotionMagic(() -> 0.0),
        WristFactory.moveWristMotionMagic(() -> 0.125)
        )
      );
    }
 
  public static Command autoPreset(
      double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {
    return new SequentialCommandGroup(
        moveArmOnly(armSetpointRotations),
        moveElevatorOnly(elevatorSetpointMeters),
        moveWristOnly(wristSetpointRotations));
  }

  public static Command moveArmOnly(double armSetpointRotations) {
    return new ParallelCommandGroup(ElevatorFactory.hold(), WristFactory.hold())
        .raceWith(ArmFactory.moveArmMotionMagic(() -> armSetpointRotations));
    }

  public static Command moveWristOnly(double wristSetpointRotations) {
    return new ParallelCommandGroup(ArmFactory.hold(), ElevatorFactory.hold())
        .raceWith(WristFactory.moveWristMotionMagic(() -> wristSetpointRotations));
    }

  public static Command moveElevatorOnly(double elevatorSetpointMeters) {
    return new ParallelCommandGroup(ArmFactory.hold(), WristFactory.hold())
        .raceWith(ElevatorFactory.moveElevMotionMagic(() -> elevatorSetpointMeters));
    }
}