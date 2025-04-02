// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.command_factories;

import static com.team1533.frc2025.Constants.SuperStructureStates;

import com.team1533.frc2025.Constants.SuperStructureStates;
import com.team1533.frc2025.RobotContainer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.BooleanSupplier;

public class SuperStructureCommandFactory {

  private static final RobotContainer container = RobotContainer.getInstance();

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

  public static BooleanSupplier wristCollidesRobot =
      () ->
          (0.157 > container.getArmSubsystem().getCurrentPosition()
              && 0.674 <= container.getWristSubsystem().getSetpoint());

  public static BooleanSupplier funnelBlocksWrist =
      () ->
          (0.18017578125 > container.getWristSubsystem().getSetpoint()
              && 0.04 >= container.getFunnelSubsystem().getCurrentPosition());
  public static BooleanSupplier wristBlocksFunnel =
      () ->
          (container.getWristSubsystem().getCurrentPosition() <= .191
              && container.getFunnelSubsystem().getPositionSetpoint() > 0);
  public static BooleanSupplier wristCollidesReef = () -> false;

  public static BooleanSupplier elevatorBlocksWristDown =
      () ->
          (SuperStructureStates.FEEDER.getState().elevGoalMeters()
                  <= container.getElevatorSubsystem().getCurrentPosition()
              && SuperStructureStates.FEEDER.getState().wristGoalRots()
                  >= container.getWristSubsystem().getSetpoint());

  public static final BooleanSupplier wristBlockElevatorUp =
      () ->
          ((SuperStructureStates.FEEDER.getState().elevGoalMeters() + Units.inchesToMeters(5))
                  >= container.getElevatorSubsystem().getCurrentPosition()
              && SuperStructureStates.FEEDER.getState().elevGoalMeters() + Units.inchesToMeters(5)
                  <= container.getElevatorSubsystem().getElevatorSetpointMeters()
              && (SuperStructureStates.FEEDER.getState().wristGoalRots() + 0.03)
                  <= container.getWristSubsystem().getSetpoint());

  //   Generic Preset - Sets the SuperStructure to Neutral Pos, the moves it to a
  //   setpoint

  public static Command genericPreset(
      double armSetpointRotations,
      double elevatorSetpointMeters,
      double wristSetpointRotations,
      double funnelSetpointRotations) {

    return new SequentialCommandGroup(
        moveArmOnly(0.21),
        moveWristOnly(0.22),
        moveFunnelOnly(funnelSetpointRotations),
        moveElevatorOnly(elevatorSetpointMeters),
        moveWristOnly(wristSetpointRotations),
        moveArmOnly(armSetpointRotations));
  }

  public static Command stowedPreset(
      double armSetpointRotations,
      double elevatorSetpointMeters,
      double wristSetpointRotations,
      double funnelSetpointRotations) {

    return new SequentialCommandGroup(
        moveArmOnly(0.21),
        moveWristOnly(0.22),
        moveFunnelOnly(0.25),
        moveElevatorOnly(elevatorSetpointMeters),
        moveFunnelOnly(0.05),
        moveWristOnly(0.005),
        moveFunnelOnly(funnelSetpointRotations),
        moveArmOnly(armSetpointRotations));
  }

  public static Command climbPrep(
      double armSetpointRotations,
      double elevatorSetpointMeters,
      double wristSetpointRotations,
      double funnelSetpointRotations) {

    return new SequentialCommandGroup(
        moveArmOnly(0.21),
        moveWristOnly(0.22),
        moveFunnelOnly(0.25),
        moveElevatorOnly(elevatorSetpointMeters),
        moveFunnelOnly(0.005),
        moveWristOnly(wristSetpointRotations),
        moveArmOnly(armSetpointRotations));
  }

  public static Command climbPreset(
      double armSetpointRotations,
      double elevatorSetpointMeters,
      double wristSetpointRotations,
      double funnelSetpointRotations) {

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

  // Neutral Pos - Default Pos of the SuperStructure. Elevator setpoint can be any
  // number within travel distance.
  // public static Command neutralPos(double elevatorSetpointMeters) {
  //   return new SequentialCommandGroup(
  //       moveArmOnly(0.21), moveWristOnly(0.22), moveElevatorOnly(elevatorSetpointMeters));
  // }

  // Forced Pos - Immediately Sets the SuperStructure to a setpoint. To be used in
  // auto, ONLY AFTER NEUTRAL POS.
  // public static Command forcedPos(
  //     double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations)
  // {

  //   return new SequentialCommandGroup(
  //       moveElevatorOnly(elevatorSetpointMeters),
  //       moveWristOnly(wristSetpointRotations),
  //       moveArmOnly(armSetpointRotations));
  // }

  //   public static Command zeroElevator() {
  //     // TODO: fix the command factories but in the case that I am not there in person this is an
  //     // example of how I think they should look
  //     return WristFactory.moveWristMotionMagic(() -> 0.3)
  //         .onlyIf(wristBlocksFunnel)
  //         .andThen(ElevatorFactory.moveElevMotionMagic(() -> 0))
  //         .andThen(ElevatorFactory.resetZero())
  //         .withName("Zero Elevator");
  //   }

  // public static Command L1() {}

  // public static Command L2() {}

  // public static Command L3() {}

  public static Command L4() {
    return ArmFactory.moveArmMotionMagic(SuperStructureStates.REEF_CLEARANCE)
        .onlyIf(() -> (wristCollidesRobot.getAsBoolean() || wristCollidesReef.getAsBoolean()))
        .andThen(
            WristFactory.moveWristMotionMagic(SuperStructureStates.ELEVATOR_CLEARANCE)
                .onlyIf(wristBlockElevatorUp))
        .andThen(
            ArmFactory.moveArmMotionMagic(SuperStructureStates.L4)
                .alongWith(WristFactory.moveWristMotionMagic(SuperStructureStates.L4))
                .alongWith(ElevatorFactory.moveElevMotionMagic(SuperStructureStates.L4)));
  }

  // public static Command stow() {}

  // public static Command algaeLower() {}

  // public static Command algaeUpper() {}

  // public static Command algaeGround() {}

  // public static Command feeder() {
  //
  // ArmFactory.moveArmMotionMagic(SuperStructureStates.SAFE).onlyIf().alongWith(WristFactory.moveWristMotionMagic(SuperStructureStates.L4)).alongWith(ElevatorFactory.moveElevMotionMagic(SuperStructureStates.L4)).andThen(null)
  // }

  // public static Command barge() {}

  // public static Command processor() {}

  // public static Command climb() {}

  // public static Command climbPrep() {}

  // Climb Sequence?

  // public static Command climbSequence() {

  //   return moveArmOnly(0.125)
  //       .andThen(
  //           new ParallelCommandGroup(ArmFactory.moveArmMotionMagic(() -> 0)),
  //           WristFactory.moveWristMotionMagic(() -> 0),
  //           ElevatorFactory.moveArmMotionMagic(() -> 0.4));
  // }

  public static Command moveArmOnly(double armSetpointRotations) {

    return new ParallelCommandGroup(
            ElevatorFactory.hold(), WristFactory.hold(), FunnelFactory.hold())
        .raceWith(ArmFactory.moveArmMotionMagic(() -> armSetpointRotations));
  }

  public static Command moveWristOnly(double wristSetpointRotations) {

    return new ParallelCommandGroup(ArmFactory.hold(), ElevatorFactory.hold(), FunnelFactory.hold())
        .raceWith(WristFactory.moveWristMotionMagic(() -> wristSetpointRotations));
  }

  public static Command moveElevatorOnly(double elevatorSetpointMeters) {

    return new ParallelCommandGroup(ArmFactory.hold(), WristFactory.hold(), FunnelFactory.hold())
        .raceWith(ElevatorFactory.moveElevMotionMagic(() -> elevatorSetpointMeters));
  }

  public static Command moveFunnelOnly(double funnelSetpointRotations) {
    return new ParallelCommandGroup(ArmFactory.hold(), ElevatorFactory.hold(), WristFactory.hold())
        .raceWith(FunnelFactory.moveFunnelMotionMagic(() -> funnelSetpointRotations));
  }
}
