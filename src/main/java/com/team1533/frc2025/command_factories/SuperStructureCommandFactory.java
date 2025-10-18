// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.command_factories;

import com.team1533.frc2025.RobotContainer;
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
//

  // TODO: find the actual values for these triggers

  // public static final Trigger elevatorZeroRequest =
  //     new Trigger(container.getElevatorSubsystem()::isZerod).whileFalse(zeroElevator());

  public static Command zeroElevator() {
    return new SequentialCommandGroup(
        moveArmOnly(0.21).until(container.getArmSubsystem().atSetpoint(0.03)),
        moveWristOnly(0.22).until(container.getWristSubsystem().atSetpoint(0.02)),
        new ParallelDeadlineGroup(
            container.getElevatorSubsystem().resetZeroPoint(),
            container.getArmSubsystem().holdSetpointCommand(),
            container.getWristSubsystem().holdSetpointCommand()));
    }

  //   Generic Preset - Sets the SuperStructure to Neutral Pos, the moves it to a
  //   setpoint

  public static Command genericPreset(
    double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {

    return new SequentialCommandGroup(
        moveArmOnly(0.21),
        moveWristOnly(0.22),
        moveElevatorOnly(elevatorSetpointMeters),
        moveWristOnly(wristSetpointRotations),
        moveArmOnly(armSetpointRotations));
  }

  public static Command stow() {

    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            ArmFactory.moveArmMotionMagic(() -> -5.0/360),
            ElevatorFactory.moveElevMotionMagic(() -> 0),
            WristFactory.moveWristMotionMagic(() -> 0.05))
            .andThen(moveWristOnly(0)));
  }

    public static Command defaultparallelPreset(double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) { 

    return new ParallelCommandGroup(
        ArmFactory.moveArmMotionMagic(() -> armSetpointRotations),
        WristFactory.moveWristMotionMagic(() -> wristSetpointRotations),
        ElevatorFactory.moveElevMotionMagic(() -> elevatorSetpointMeters));
        //.until(container.getArmSubsystem().atSetpoint(ArmConstants.toleranceRotations))
    }

    // public static Command defaultArmPreset(double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {
    
    //     // return (new SequentialCommandGroup(
    //     //     new ParallelCommandGroup(
    //     //         ElevatorFactory.hold()
    //     //         .raceWith(
    //     //             ArmFactory.moveArmMotionMagic(() -> armSetpointRotations),
    //     //             WristFactory.moveWristMotionMagic(() -> wristSetpointRotations)))
    //     // .andThen(new ParallelCommandGroup(
    //     //     WristFactory.hold(),
    //     //     ArmFactory.hold()
    //     //     .raceWith(ElevatorFactory.moveElevMotionMagic(() -> elevatorSetpointMeters))
    //     //     )
    //     // )

    //     // ))
    
    //     return (new SequentialCommandGroup(
    //         ArmFactory.moveArmMotionMagic((() -> armSetpointRotations)
            
    //         ,
    //     ))
    // ;


    //     .andThen(
    //         new ParallelCommandGroup(
    //             WristFactory.hold(),ArmFactory.hold().raceWith(
    //             ElevatorFactory.moveElevMotionMagic(() -> elevatorSetpointMeters))))
    //     .andThen(
    //         new ParallelCommandGroup(
    //             ArmFactory.hold(),
    //             ElevatorFactory.hold().raceWith(
    //             WristFactory.moveWristMotionMagic(() -> wristSetpointRotations))))    
    //     );
    // }



// public static Command stow(double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {
//     return new SequentialCommandGroup(
//         ArmFactory.hold(), ElevatorFactory.hold()
//     )
// }

// Intake Pos
//       Arm (-4.8/360),
//       Wrist (0.374),
//       Elevator (0.065)

//  Stowed Pos
//       Arm (-4.8/360)
//       Wrist (0)
//       Elevator (0)

  public static Command reefToFeeder(
      double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {

    return (new ParallelCommandGroup(
            ElevatorFactory.hold()
                .raceWith(
                    ArmFactory.moveArmMotionMagic(() -> 0.21),
                    WristFactory.moveWristMotionMagic(() -> 0.24)))
        .andThen(
            new ParallelCommandGroup(
                ArmFactory.moveArmMotionMagic(() -> 0.15),
                WristFactory.moveWristMotionMagic(() -> 0.71),
                ElevatorFactory.moveElevMotionMagic(() -> 0.043))));
  }

  public static Command autoPreset(
      double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {

    return new SequentialCommandGroup(
        moveArmOnly(armSetpointRotations),
        moveElevatorOnly(elevatorSetpointMeters),
        moveWristOnly(wristSetpointRotations));
  }

    public static Command climbPrep(
      double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {

    return new SequentialCommandGroup(
        moveArmOnly(0.21),
        moveWristOnly(0.22),
        moveElevatorOnly(elevatorSetpointMeters),
        moveWristOnly(wristSetpointRotations),
        moveArmOnly(armSetpointRotations));
  }

  public static Command climbPreset(
      double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {

    return (new ParallelCommandGroup(
            ArmFactory.moveArmMotionMagic(() -> 0.125),
            WristFactory.moveWristMotionMagic(() -> 0.125),
            ElevatorFactory.moveElevMotionMagic(() -> 0.22))
        .andThen(
            new ParallelCommandGroup(
                ArmFactory.moveArmMotionMagic(() -> 0.007),
                ElevatorFactory.moveElevMotionMagic(() -> 0.3),
                WristFactory.moveWristMotionMagic(() -> 0.01))));
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
