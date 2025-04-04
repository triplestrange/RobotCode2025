// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025;

import static com.team1533.frc2025.subsystems.vision.VisionConstants.camera0Name;
import static com.team1533.frc2025.subsystems.vision.VisionConstants.robotToCamera0;
import static com.team1533.frc2025.subsystems.vision.VisionConstants.robotToCamera1;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team1533.frc2025.command_factories.SuperStructureCommandFactory;
import com.team1533.frc2025.generated.TunerConstants;
import com.team1533.frc2025.subsystems.arm.ArmIO;
import com.team1533.frc2025.subsystems.arm.ArmIOReal;
import com.team1533.frc2025.subsystems.arm.ArmIOSim;
import com.team1533.frc2025.subsystems.arm.ArmSubsystem;
import com.team1533.frc2025.subsystems.drive.DriveConstants;
import com.team1533.frc2025.subsystems.drive.DriveSubsystem;
import com.team1533.frc2025.subsystems.drive.GyroIO;
import com.team1533.frc2025.subsystems.drive.GyroIOPigeon2;
import com.team1533.frc2025.subsystems.drive.GyroIOSim;
import com.team1533.frc2025.subsystems.drive.ModuleIO;
import com.team1533.frc2025.subsystems.drive.ModuleIOTalonFXReal;
import com.team1533.frc2025.subsystems.drive.ModuleIOTalonFXSim;
import com.team1533.frc2025.subsystems.intake.IntakeConstants;
import com.team1533.frc2025.subsystems.intake.IntakeSubsystem;
import com.team1533.frc2025.subsystems.vision.VisionConstants;
import com.team1533.frc2025.subsystems.vision.VisionIO;
import com.team1533.frc2025.subsystems.vision.VisionIOPhotonVision;
import com.team1533.frc2025.subsystems.vision.VisionIOPhotonVisionSim;
import com.team1533.frc2025.subsystems.vision.VisionSubsystem;
import com.team1533.frc2025.subsystems.wrist.WristIO;
import com.team1533.frc2025.subsystems.wrist.WristIOReal;
import com.team1533.frc2025.subsystems.wrist.WristIOSim;
import com.team1533.frc2025.subsystems.wrist.WristSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.team1533.frc2025.subsystems.elevator.*;
import com.team1533.lib.subsystems.MotorIO;
import com.team1533.lib.subsystems.SimTalonFXIO;
import com.team1533.lib.subsystems.TalonFXIO;
import com.team1533.lib.swerve.DriveCharacterizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import lombok.Setter;

public class RobotContainer {

        private final CommandPS5Controller driveController = new CommandPS5Controller(0);
        private final CommandPS5Controller operatorController = new CommandPS5Controller(1);


        @AutoLogOutput @Getter
        private boolean algaeMode = false;
        @Getter @AutoLogOutput @Setter
        private boolean left = true;
        @Getter @AutoLogOutput @Setter
        private boolean right = true;

        private Trigger inCoralMode = new Trigger(() -> !algaeMode);
        private Trigger inAlgaeMode = new Trigger(() -> algaeMode);

        @Getter
        private DriveSubsystem driveSubsystem;
        @Getter
        private VisionSubsystem visionSubsystem;
        @Getter
        private ArmSubsystem armSubsystem;
        @Getter
        private ElevatorSubsystem elevatorSubsystem;
        @Getter
        private WristSubsystem wristSubsystem;
        @Getter
        private IntakeSubsystem intakeSubsystem;

        private final LoggedDashboardChooser<Command> autoChooser;

        public SwerveDriveSimulation driveSimulation = null;

        @Getter
        private static RobotContainer instance;

        public RobotContainer(Robot robot) {
                instance = this;

                switch (Constants.getRobot()) {
                        case COMPBOT:

                                driveSubsystem = new DriveSubsystem(new GyroIOPigeon2(),
                                                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                                                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                                                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                                                new ModuleIOTalonFXReal(TunerConstants.BackRight));

                                visionSubsystem = new VisionSubsystem(driveSubsystem,
                                                new VisionIOPhotonVision(VisionConstants.camera0Name, robotToCamera0),
                                                new VisionIOPhotonVision(VisionConstants.camera1Name, robotToCamera1));

                                armSubsystem = new ArmSubsystem(new ArmIOReal());
                                elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOReal());
                                wristSubsystem = new WristSubsystem(new WristIOReal());
                                intakeSubsystem = new IntakeSubsystem(IntakeConstants.config,
                                                new TalonFXIO(IntakeConstants.config));

                                break;

                        case SIMBOT:

                                driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig,
                                                Pose2d.kZero);
                                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                                driveSubsystem = new DriveSubsystem(new GyroIOSim(driveSimulation.getGyroSimulation()),
                                                new ModuleIOTalonFXSim(TunerConstants.FrontLeft,
                                                                driveSimulation.getModules()[0]),
                                                new ModuleIOTalonFXSim(TunerConstants.FrontRight,
                                                                driveSimulation.getModules()[1]),
                                                new ModuleIOTalonFXSim(TunerConstants.BackLeft,
                                                                driveSimulation.getModules()[2]),
                                                new ModuleIOTalonFXSim(TunerConstants.BackRight,
                                                                driveSimulation.getModules()[3]));

                                visionSubsystem = new VisionSubsystem(
                                                driveSubsystem,
                                                new VisionIOPhotonVisionSim(
                                                                camera0Name, robotToCamera0,
                                                                driveSimulation::getSimulatedDriveTrainPose)
                                );
                                armSubsystem = new ArmSubsystem(new ArmIOSim());

                                elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());

                                wristSubsystem = new WristSubsystem(new WristIOSim());

                                intakeSubsystem = new IntakeSubsystem(IntakeConstants.config,
                                                new SimTalonFXIO(IntakeConstants.config));

                                break;

                        default:

                                driveSubsystem = new DriveSubsystem(new GyroIO() {
                                }, new ModuleIO() {
                                }, new ModuleIO() {
                                }, new ModuleIO() {
                                }, new ModuleIO() {
                                });

                                visionSubsystem = new VisionSubsystem(driveSubsystem, new VisionIO() {
                                }, new VisionIO() {
                                });

                                armSubsystem = new ArmSubsystem(new ArmIO() {
                                });

                                elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {
                                });

                                wristSubsystem = new WristSubsystem(new WristIO() {

                                });

                                break;
                }

                NamedCommands.registerCommand("Arm L4", SuperStructureCommandFactory
                        .genericPreset( 0.205, 1.07, 0.337).asProxy());

                NamedCommands.registerCommand("Outtake", (intakeSubsystem.dutyCycleCommand(() -> -0.2)).withTimeout(0.75));

                NamedCommands.registerCommand("Arm Neutral", SuperStructureCommandFactory
                        .genericPreset( 0.21, 0.4, 0.22).asProxy());

                NamedCommands.registerCommand("Arm Feeder While Moving", SuperStructureCommandFactory
                        .forcedPos( 0.15, 0.045, 0.71).asProxy());

                NamedCommands.registerCommand("Arm Feeder", SuperStructureCommandFactory
                        .genericPreset( 0.15, 0.045, 0.71).asProxy());

                NamedCommands.registerCommand("Intake", (intakeSubsystem.dutyCycleCommand(() -> 0.5)).withTimeout(1.3));

                NamedCommands.registerCommand("Swerve Stop", driveSubsystem.runOnce(driveSubsystem::stop));

                // Set up auto routines
                // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
                autoChooser = new LoggedDashboardChooser<>("Auto Choices");

                // Set up SysId routines
                autoChooser.addOption("Drive Wheel Radius Characterization",
                                DriveCharacterizer.wheelRadiusCharacterization(driveSubsystem));
                // autoChooser.addOption("Drive Simple FF Characterization",
                //                 DriveCharacterizer.feedforwardCharacterization(driveSubsystem));
                // autoChooser.addOption(
                //                 "Drive SysId (Quasistatic Forward)",
                //                 driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                // autoChooser.addOption(
                //                 "Drive SysId (Quasistatic Reverse)",
                //                 driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                // autoChooser.addOption("Drive SysId (Dynamic Forward)",
                //                 driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
                // autoChooser.addOption("Drive SysId (Dynamic Reverse)",
                //                 driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                // autoChooser.addOption("test path", AutoBuilder.buildAuto("Test Path"));

                // autoChooser.addOption("test path 2", AutoBuilder.buildAuto("Test Path 2"));

                // autoChooser.addOption("pid tuning adventures", AutoBuilder.buildAuto("New Auto"));

                autoChooser.addDefaultOption("None", Commands.none());

                autoChooser.addOption("Right Level 2 Middle ID 21", AutoBuilder.buildAuto("RL2 Mid"));
                autoChooser.addOption("Left a lot of coral", AutoBuilder.buildAuto("2pl"));
                autoChooser.addOption("Big Boi Auto", AutoBuilder.buildAuto("Left 2 Piece"));
                autoChooser.addOption("Left 2 Piece", AutoBuilder.buildAuto("Left 2 Piece"));

                // configure buetton bindings
                configureButtonBindings();

        }

        // Button Binds
        private void configureButtonBindings() {

                // Driver Binds

                // Mode Toggle
                driveController.R3().whileTrue(Commands.startEnd(() -> algaeMode = true, () -> algaeMode = false));

                // Arm Stop
                driveController.PS()
                                .onTrue(armSubsystem.setSetpointHere().alongWith(elevatorSubsystem.setSetpointHere())
                                                .alongWith(wristSubsystem.setSetpointHere()));

                // Swerve Drive
                driveSubsystem.setDefaultCommand(
                                driveSubsystem.run(() -> driveSubsystem.teleopControl(-driveController.getLeftY(),
                                                -driveController.getLeftX(), -driveController.getRightX())));

                // Gyro Rotation Reset
                driveController.options().onTrue(driveSubsystem.runOnce(driveSubsystem::teleopResetRotation));

                // Climb Vertical
                driveController.povUp().onTrue(SuperStructureCommandFactory.genericPreset( 0.25, 0, 0.5));

                // Climb Prep
                driveController.povRight().onTrue(SuperStructureCommandFactory.climbPrep());

                // Climb Sequence
                driveController.povDown().onTrue(SuperStructureCommandFactory.climbSequence());

                // L4 Coral Automation
                driveController.triangle().and(inCoralMode).onTrue(SuperStructureCommandFactory
                                .genericPreset( 0.205, 1.07, 0.337));

                // L3 Coral Automation
                driveController.circle().and(inCoralMode)
                                .onTrue(SuperStructureCommandFactory.genericPreset( 0.16, 0.387106, 0.22));

                // L2 Coral Automation
                driveController.cross().and(inCoralMode).onTrue(SuperStructureCommandFactory.genericPreset( 0.1, 0.086995, 0.145));

                // // L1 Coral Automation
                // driveController.povRight().and(inCoralMode).onTrue(SuperStructureCommandFactory
                //                 .genericPreset( 0.15, 0.0445, 0.71));

                // Zero Preset
                driveController.povLeft().and(inCoralMode).onTrue(SuperStructureCommandFactory
                                .genericPreset( 0, 0, 0));

                // Coral Feeder Automation
                driveController.square().and(inCoralMode).onTrue(SuperStructureCommandFactory
                                .genericPreset( 0.155, 0.045
                                , 0.71));

                // Coral Intake
                driveController.R1().and(inCoralMode).whileTrue(intakeSubsystem.dutyCycleCommand(() -> 0.5));

                // Coral Outtake
                driveController.L1().and(inCoralMode).whileTrue(intakeSubsystem.dutyCycleCommand(() -> -0.2));

                // Processor Algae
                driveController.square().and(inAlgaeMode).onTrue(SuperStructureCommandFactory
                                .genericPreset( 0.08, 0, 0.5));

                // Low Reef Algae
                driveController.cross().and(inAlgaeMode).onTrue(SuperStructureCommandFactory.genericPreset( 0.155, 0.055, 0.44));

                // High Reef Algae
                driveController.circle().and(inAlgaeMode).onTrue(SuperStructureCommandFactory
                                .genericPreset( 0.18, 0.48, 0.47));

                // Barge Algae
                driveController.triangle().and(inAlgaeMode).onTrue(SuperStructureCommandFactory
                                .genericPreset( 0.24, 1.07, 0.3));
                
                // Algae Intake
                driveController.R1().and(inAlgaeMode).whileTrue(intakeSubsystem.dutyCycleCommand(() -> -0.75)).onFalse(intakeSubsystem.dutyCycleCommand(() -> -0.5));

                // Algae Outtake
                driveController.L1().and(inAlgaeMode).whileTrue(intakeSubsystem.dutyCycleCommand(() -> 0.5));

                // Auto Align Arm Neutral Pos
                driveController.L2().and(() -> wristSubsystem.getCurrentPosition() > 0.65).onTrue(SuperStructureCommandFactory
                                .genericPreset(0.21, 0.043, 0.22));
                driveController.R2().and(() -> wristSubsystem.getCurrentPosition() > 0.65).onTrue(SuperStructureCommandFactory
                                .genericPreset(0.21, 0.045, 0.22));

                // Auto Align Options
                driveController.L2().whileTrue(Commands.runEnd(() -> setRight(false), () -> setRight(true)));
                driveController.R2().whileTrue(Commands.runEnd(() -> setLeft(false), () -> setLeft(true)));

                
        //Operator Binds

                //Operator Manual Arm Override
                new Trigger(() -> Math.abs(operatorController.getLeftY()) > 0.1)
                        .whileTrue(armSubsystem.runDutyCycle(() -> 0.3* operatorController.getLeftY()));

                //Operator Manual Wrist Override        
                new Trigger(() -> Math.abs(operatorController.getRightY()) > 0.1)
                        .whileTrue(wristSubsystem.runDutyCycle(() -> 0.15* operatorController.getRightY()));

                //Operator Manual Elevator Override
                new Trigger(() -> Math.abs((operatorController.getR2Axis() -operatorController.getL2Axis()) / 2) > 0.1)
                        .whileTrue(elevatorSubsystem.runDutyCycle(() -> 0.25* ((operatorController.getR2Axis() -operatorController.getL2Axis()) / 2)));
                
                //Operator Elevator Zero
                operatorController.cross().onTrue(SuperStructureCommandFactory.zeroElevator());

                operatorController.square().onTrue(new InstantCommand(() -> {setRight(false);
                setLeft(false);}));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.get();
        }

        public void resetSimulationField() {
                if (Constants.getRobot() != Constants.RobotType.SIMBOT)
                        return;

                SimulatedArena.getInstance().resetFieldForAuto();
                driveSubsystem.setPose(new Pose2d(2, 2, Rotation2d.kZero));
        }

        public void displaySimFieldToAdvantageScope() {
                if (Constants.getRobot() != Constants.RobotType.SIMBOT)
                        return;

                Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
                Logger.recordOutput(
                                "FieldSimulation/Coral",
                                SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
                Logger.recordOutput(
                                "FieldSimulation/Algae",
                                SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
        }
}
