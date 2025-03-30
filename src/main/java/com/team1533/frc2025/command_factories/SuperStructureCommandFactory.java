// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.command_factories;

import java.io.Console;

import com.google.protobuf.WireFormat;
import com.team1533.frc2025.Constants;
import com.team1533.frc2025.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SuperStructureCommandFactory {

  private final static RobotContainer container = RobotContainer.getInstance();

  //TODO: find the actual values for these triggers
  public final static Trigger wristBlockFunnel = new Trigger(null);

  public final static Trigger funnelBlockElevator = new Trigger(null);

  public final static Trigger elevatorBlockWristDown = new Trigger(null);

  public final static Trigger wristBlockElevatorUp = new Trigger(null);

  public final static Trigger wristCollision = new Trigger(null);

  public final static Trigger funnelBlockWrist = new Trigger(null);

  public final static Trigger elevatorZeroRequest = new Trigger(container.getElevatorSubsystem()::isZerod).onFalse(zeroElevator());

  // Generic Preset - Sets the SuperStructure to Neutral Pos, the moves it to a
  // setpoint
  public static Command genericPreset(
      double armSetpointRotations,
      double elevatorSetpointMeters,
      double wristSetpointRotations,
      double funnelSetpointRotations) {


    return new SequentialCommandGroup(
        moveArmOnly(0.21),
        moveWristOnly(0.22),
        moveElevatorOnly(elevatorSetpointMeters),
        moveWristOnly(wristSetpointRotations),
        moveArmOnly(armSetpointRotations),
        moveFunnelOnly(funnelSetpointRotations));
  }

  // Neutral Pos - Default Pos of the SuperStructure. Elevator setpoint can be any
  // number within travel distance.
  public static Command neutralPos(
      double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {
    return new SequentialCommandGroup(
        moveArmOnly(0.21), moveWristOnly(0.22), moveElevatorOnly(elevatorSetpointMeters));
  }

  // Forced Pos - Immediately Sets the SuperStructure to a setpoint. To be used in
  // auto, ONLY AFTER NEUTRAL POS.
  public static Command forcedPos(
      double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {

    return new SequentialCommandGroup(
        moveElevatorOnly(elevatorSetpointMeters),
        moveWristOnly(wristSetpointRotations),
        moveArmOnly(armSetpointRotations));
  }

  public static Command zeroElevator() {
    
    // return new SequentialCommandGroup(
    //     moveArmOnly(0.21),
    //     moveWristOnly(0.22),
    //     new ParallelDeadlineGroup(
    //         ElevatorFactory.resetZero(), ArmFactory.hold(), WristFactory.hold()));
// TODO: fix the command factories but in the case that I am not there in person this is an example of how I think they should look
    return moveWristOnly(0.3).onlyIf(wristBlockFunnel).andThen(moveFunnelOnly(0.25)).onlyIf(funnelBlockElevator).andThen(new ParallelCommandGroup(ArmFactory.hold(), WristFactory.hold(), FunnelFactory.hold()).raceWith(ElevatorFactory.resetZero()));
  }

  // Climb Sequence?

  public static Command climbSequence() {

    return moveArmOnly(0.125)
        .andThen(
            new ParallelCommandGroup(ArmFactory.moveArmMotionMagic(() -> 0)),
            WristFactory.moveWristMotionMagic(() -> 0),
            ElevatorFactory.moveArmMotionMagic(() -> 0.4));
  }

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
        .raceWith(ElevatorFactory.moveArmMotionMagic(() -> elevatorSetpointMeters));
  }

  public static Command moveFunnelOnly(double funnelSetpointRotations) {
    return new ParallelCommandGroup(ArmFactory.hold(), ElevatorFactory.hold(), WristFactory.hold())
        .raceWith(FunnelFactory.moveFunnelMotionMagic(() -> funnelSetpointRotations));
  }
}
