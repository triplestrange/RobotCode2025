// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.team1533.frc2025.Constants.Mode;
import com.team1533.frc2025.generated.TunerConstants;
import com.team1533.frc2025.subsystems.drive.DriveSubsystem;
import com.team1533.frc2025.subsystems.drive.GyroIO;
import com.team1533.frc2025.subsystems.drive.GyroIOPigeon2;
import com.team1533.frc2025.subsystems.drive.GyroIOSim;
import com.team1533.frc2025.subsystems.drive.ModuleIO;
import com.team1533.frc2025.subsystems.drive.ModuleIOTalonFXReal;
import com.team1533.frc2025.subsystems.drive.ModuleIOTalonFXSim;
import com.team1533.frc2025.subsystems.vision.VisionIO;
import com.team1533.frc2025.subsystems.vision.VisionIOPhotonVision;
import com.team1533.frc2025.subsystems.vision.VisionSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Getter;

public class RobotContainer {

    private final Mode currentMode = Constants.getMode();
    
    private SwerveDriveSimulation driveSimulation = null;


    private DriveSubsystem buildDriveSubsystem() {
        if (currentMode == Mode.REAL) {
            return new DriveSubsystem(new GyroIOPigeon2(), new ModuleIOTalonFXReal(TunerConstants.FrontLeft), new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft), new ModuleIOTalonFXReal(TunerConstants.BackRight));
        }

        else if (currentMode == Mode.SIM) {
            driveSimulation = new SwerveDriveSimulation(DriveSubsystem.mapleSimConfig, new Pose2d(3, 3, Rotation2d.fromDegrees(0)));
            SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
            return new DriveSubsystem(new GyroIOSim(driveSimulation.getGyroSimulation()), new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]), new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                    new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]), new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]));
        }

        else {
            return new DriveSubsystem(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        }
    }
    
    @Getter
    private final DriveSubsystem driveSubsystem = buildDriveSubsystem();
    

    public RobotContainer(Robot robot) {
    }
}
