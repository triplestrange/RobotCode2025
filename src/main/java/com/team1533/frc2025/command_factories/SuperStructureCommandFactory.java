package com.team1533.frc2025.command_factories;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SuperStructureCommandFactory {

        private final Trigger wristBlockElevator = new Trigger(null, null);

        private final Trigger armBlockWrist = new Trigger(null);

        private final Trigger elevatorZeroRequest = new Trigger(null);

        // Generic Preset - Sets the SuperStructure to Neutral Pos, the moves it to a
        // setpoint
        public static Command genericPreset(
                        double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {

                return new SequentialCommandGroup(
                                moveArmOnly(0.21),
                                moveWristOnly(0.22),
                                moveElevatorOnly(elevatorSetpointMeters),
                                moveWristOnly(wristSetpointRotations),
                                moveArmOnly(armSetpointRotations));
        }

        // Neutral Pos - Default Pos of the SuperStructure. Elevator setpoint can be any
        // number within travel distance.
        public static Command neutralPos(
                        double armSetpointRotations, double elevatorSetpointMeters, double wristSetpointRotations) {
                return new SequentialCommandGroup(
                                moveArmOnly(0.21),
                                moveWristOnly(0.22),
                                moveElevatorOnly(elevatorSetpointMeters));
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
                return new SequentialCommandGroup(
                                moveArmOnly(0.21),
                                moveWristOnly(0.22),
                                new ParallelDeadlineGroup(ElevatorFactory.resetZero(), ArmFactory.hold(),
                                                WristFactory.hold()));
        }

        // Climb Sequence?

        public static Command climbSequence() {

                return moveArmOnly(0.125).andThen(new ParallelCommandGroup(
                                ArmFactory.moveArmMotionMagic(() -> 0)),
                                WristFactory.moveWristMotionMagic(() -> 0),
                                ElevatorFactory.moveArmMotionMagic(() -> 0.4));
        }

        public static Command moveArmOnly(
                        double armSetpointRotations) {

                return new ParallelCommandGroup(
                        ArmFactory.moveArmMotionMagic(() -> armSetpointRotations),
                                ElevatorFactory.hold(),
                                WristFactory.hold());
        }

        public static Command moveWristOnly(
                        double wristSetpointRotations) {

                return new ParallelCommandGroup(
                               ArmFactory.hold(),
                                ElevatorFactory.hold(),
                                WristFactory.moveWristMotionMagic(() -> wristSetpointRotations));
        }

        public static Command moveElevatorOnly(
                        double elevatorSetpointMeters) {

                return new ParallelCommandGroup(
                                ArmFactory.hold(),
                                ElevatorFactory.moveArmMotionMagic(() -> elevatorSetpointMeters),
                               WristFactory.hold());
        }

}
