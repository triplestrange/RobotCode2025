// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025;

import static com.team1533.frc2025.subsystems.vision.VisionConstants.camera0Name;
import static com.team1533.frc2025.subsystems.vision.VisionConstants.camera1Name;
import static com.team1533.frc2025.subsystems.vision.VisionConstants.robotToCamera0;
import static com.team1533.frc2025.subsystems.vision.VisionConstants.robotToCamera1;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
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
import com.team1533.frc2025.subsystems.elevator.ElevatorSubsystem;
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
import com.team1533.frc2025.subsystems.elevator.*;
import com.team1533.frc2025.subsystems.elevator.ElevatorIOReal;
import com.team1533.lib.subsystems.MotorIO;
import com.team1533.lib.subsystems.SimTalonFXIO;
import com.team1533.lib.subsystems.TalonFXIO;
import com.team1533.lib.swerve.DriveCharacterizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;

public class RobotContainer {

        private final CommandPS5Controller driveController = new CommandPS5Controller(0);

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
                                intakeSubsystem = new IntakeSubsystem(IntakeConstants.config, new TalonFXIO(IntakeConstants.config));

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
                                                                driveSimulation::getSimulatedDriveTrainPose),
                                                new VisionIOPhotonVisionSim(
                                                                camera1Name, robotToCamera1,
                                                                driveSimulation::getSimulatedDriveTrainPose));
                                armSubsystem = new ArmSubsystem(new ArmIOSim());

                                elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());

                                wristSubsystem = new WristSubsystem(new WristIOSim());

                                intakeSubsystem = new IntakeSubsystem(IntakeConstants.config, new SimTalonFXIO(IntakeConstants.config));

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

                // Set up auto routines
                autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

                // Set up SysId routines
                autoChooser.addOption("Drive Wheel Radius Characterization",
                                DriveCharacterizer.wheelRadiusCharacterization(driveSubsystem));
                autoChooser.addOption("Drive Simple FF Characterization",
                                DriveCharacterizer.feedforwardCharacterization(driveSubsystem));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Forward)",
                                driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption(
                                "Drive SysId (Quasistatic Reverse)",
                                driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                autoChooser.addOption("Drive SysId (Dynamic Forward)",
                                driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
                autoChooser.addOption("Drive SysId (Dynamic Reverse)",
                                driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

                autoChooser.addOption("test path", AutoBuilder.buildAuto("Test Path"));

                autoChooser.addOption("test path 2", AutoBuilder.buildAuto("Test Path 2"));

                // configure button bindings
                configureButtonBindings();

        }

        private void configureButtonBindings() {
                driveSubsystem.setDefaultCommand(
                                driveSubsystem.run(() -> driveSubsystem.teleopControl(-driveController.getLeftY(),
                                                -driveController.getLeftX(), -driveController.getRightX())));
                                                
                driveController.create().onTrue(driveSubsystem.runOnce(driveSubsystem::teleopResetRotation));

                driveController.cross().onTrue(wristSubsystem.motionMagicPositionCommand(() -> 0.5));
                driveController.circle().onTrue(wristSubsystem.motionMagicPositionCommand(() -> 0.337));
                driveController.triangle().onTrue(wristSubsystem.motionMagicPositionCommand(() -> 0));

                driveController.povDown().onTrue(armSubsystem.motionMagicPositionCommand(() -> 0));
                driveController.povLeft().onTrue(armSubsystem.motionMagicPositionCommand(() -> 0.195));
                driveController.povUp().onTrue(armSubsystem.motionMagicPositionCommand(() -> 0.25));

                driveController.L1().onTrue(elevatorSubsystem.motionMagicPositionCommand(() -> Units.inchesToMeters(3)));
                driveController.R1().onTrue(elevatorSubsystem.motionMagicPositionCommand(()-> 1.058));

                intakeSubsystem.setDefaultCommand(intakeSubsystem
                                .dutyCycleCommand(() -> ((driveController.getR2Axis() - driveController.getL2Axis()) / 2)));

                

                // L4:
                // a 0.195
                // e 1.058
                // w 0.337

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

                driveSimulation.setSimulationWorldPose(Pose2d.kZero);
                SimulatedArena.getInstance().resetFieldForAuto();
                driveSubsystem.setPose();
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
