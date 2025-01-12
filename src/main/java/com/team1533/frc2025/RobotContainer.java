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
import com.team1533.frc2025.subsystems.drive.DriveSubsystem;
import com.team1533.frc2025.subsystems.drive.GyroIO;
import com.team1533.frc2025.subsystems.drive.GyroIOPigeon2;
import com.team1533.frc2025.subsystems.drive.GyroIOSim;
import com.team1533.frc2025.subsystems.drive.ModuleIO;
import com.team1533.frc2025.subsystems.drive.ModuleIOTalonFXReal;
import com.team1533.frc2025.subsystems.drive.ModuleIOTalonFXSim;
import com.team1533.frc2025.subsystems.vision.VisionConstants;
import com.team1533.frc2025.subsystems.vision.VisionIO;
import com.team1533.frc2025.subsystems.vision.VisionIOPhotonVision;
import com.team1533.frc2025.subsystems.vision.VisionIOPhotonVisionSim;
import com.team1533.frc2025.subsystems.vision.VisionSubsystem;
import com.team1533.lib.swerve.DriveCharacterizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;

public class RobotContainer {

        @Getter
        private DriveSubsystem driveSubsystem;
        @Getter
        private VisionSubsystem visionSubsystem;

        private final LoggedDashboardChooser<Command> autoChooser;

        private SwerveDriveSimulation driveSimulation = null;

        public RobotContainer(Robot robot) {

                switch (Constants.getMode()) {
                        case REAL:

                                driveSubsystem = new DriveSubsystem(new GyroIOPigeon2(),
                                                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                                                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                                                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                                                new ModuleIOTalonFXReal(TunerConstants.BackRight));

                                visionSubsystem = new VisionSubsystem(driveSubsystem,
                                                new VisionIOPhotonVision(VisionConstants.camera0Name, robotToCamera0),
                                                new VisionIOPhotonVision(VisionConstants.camera1Name, robotToCamera1));

                                break;

                        case SIM:

                                driveSimulation = new SwerveDriveSimulation(DriveSubsystem.mapleSimConfig,
                                                new Pose2d(3, 3, Rotation2d.fromDegrees(0)));
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

                // configure button bindings
                configureButtonBindings();

        }

        private void configureButtonBindings() {

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
                if (Constants.getMode() != Constants.Mode.SIM)
                        return;

                driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().resetFieldForAuto();
        }

        public void displaySimFieldToAdvantageScope() {
                if (Constants.getMode() != Constants.Mode.SIM)
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
