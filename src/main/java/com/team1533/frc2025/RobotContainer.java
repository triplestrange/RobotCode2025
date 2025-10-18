// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025;

import static com.team1533.frc2025.subsystems.vision.VisionConstants.camera0Name;
import static com.team1533.frc2025.subsystems.vision.VisionConstants.robotToCamera0;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.team1533.frc2025.command_factories.SuperStructureCommandFactory;
import com.team1533.frc2025.generated.TunerConstants;
import com.team1533.frc2025.subsystems.arm.ArmIO;
import com.team1533.frc2025.subsystems.arm.ArmIOReal;
import com.team1533.frc2025.subsystems.arm.ArmIOSim;
import com.team1533.frc2025.subsystems.arm.ArmSubsystem;
import com.team1533.frc2025.subsystems.climb.ClimbSubsystem;
import com.team1533.frc2025.subsystems.climb.ClimbConstants;
import com.team1533.frc2025.subsystems.drive.DriveConstants;
import com.team1533.frc2025.subsystems.drive.DriveSubsystem;
import com.team1533.frc2025.subsystems.drive.GyroIO;
import com.team1533.frc2025.subsystems.drive.GyroIOPigeon2;
import com.team1533.frc2025.subsystems.drive.GyroIOSim;
import com.team1533.frc2025.subsystems.drive.ModuleIO;
import com.team1533.frc2025.subsystems.drive.ModuleIOTalonFXReal;
import com.team1533.frc2025.subsystems.drive.ModuleIOTalonFXSim;
import com.team1533.frc2025.subsystems.elevator.*;
import com.team1533.frc2025.subsystems.intake.IntakeConstants;
import com.team1533.frc2025.subsystems.intake.IntakeIOReal;
import com.team1533.frc2025.subsystems.intake.IntakeIO;
import com.team1533.frc2025.subsystems.intake.IntakeIOReal;
import com.team1533.frc2025.subsystems.intake.IntakeIOSim;
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
import com.team1533.lib.loops.StatusSignalLoop;
import com.team1533.lib.subsystems.SimTalonFXIO;
import com.team1533.lib.subsystems.TalonFXIO;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import lombok.Getter;
import lombok.Setter;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private final CommandPS5Controller driveController = new CommandPS5Controller(0);
  private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

  @AutoLogOutput @Getter private boolean algaeMode = false;
  @AutoLogOutput @Getter private boolean troughMode = false;
  @AutoLogOutput @Getter private boolean coralMode = false;
  @Getter @AutoLogOutput @Setter private boolean left = true;
  @Getter @AutoLogOutput @Setter private boolean right = true;
  @Getter @AutoLogOutput @Setter private boolean isFacingForward = true;

  private Trigger inCoralMode = new Trigger(() -> !algaeMode && !troughMode);
  private Trigger inTroughMode = new Trigger(() -> troughMode);
  private Trigger inAlgaeMode = new Trigger(() -> algaeMode);
  private Trigger facingForward = new Trigger(() -> isFacingForward);
  private Trigger facingBackward = new Trigger(() -> !isFacingForward);
  
  @Getter private final DriveSubsystem driveSubsystem;
  @Getter private final VisionSubsystem visionSubsystem;
  @Getter private final ArmSubsystem armSubsystem;
  @Getter private final ElevatorSubsystem elevatorSubsystem;
  @Getter private final WristSubsystem wristSubsystem;
  @Getter private final IntakeSubsystem intakeSubsystem;
  @Getter private final ClimbSubsystem climbSubsystem;

  private final LoggedDashboardChooser<Command> autoChooser;

  public SwerveDriveSimulation driveSimulation = null;

  @Getter private static RobotContainer instance;

  private final RobotState state;

  private final StatusSignalLoop fastLoop = new StatusSignalLoop(250, "Fast Looper");

  public RobotContainer() {
    instance = this;

    this.state = new RobotState();

    switch (Constants.getRobot()) {
      case COMPBOT:
        driveSubsystem =
            new DriveSubsystem(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight));

        visionSubsystem =
            new VisionSubsystem(
                state, new VisionIOPhotonVision(VisionConstants.camera0Name, robotToCamera0));

        armSubsystem = new ArmSubsystem(new ArmIOReal());
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOReal());
        wristSubsystem = new WristSubsystem(new WristIOReal());
        climbSubsystem = new ClimbSubsystem(ClimbConstants.config, new TalonFXIO(ClimbConstants.config));
        intakeSubsystem = new IntakeSubsystem(new IntakeIOReal());    

        break;

      case SIMBOT:
        driveSimulation = new SwerveDriveSimulation(DriveConstants.mapleSimConfig, Pose2d.kZero);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        driveSubsystem =
            new DriveSubsystem(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(TunerConstants.BackRight, driveSimulation.getModules()[3]));

        visionSubsystem =
            new VisionSubsystem(
                state,
                new VisionIOPhotonVisionSim(
                    camera0Name, robotToCamera0, driveSimulation::getSimulatedDriveTrainPose));
        armSubsystem = new ArmSubsystem(new ArmIOSim());
        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
        wristSubsystem = new WristSubsystem(new WristIOSim());
        climbSubsystem = new ClimbSubsystem(ClimbConstants.config, new SimTalonFXIO(ClimbConstants.config));
        intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());

        break;

      default:
        driveSubsystem =
            new DriveSubsystem(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        visionSubsystem = new VisionSubsystem(state, new VisionIO() {});

        armSubsystem = new ArmSubsystem(new ArmIO() {});

        elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {});

        wristSubsystem = new WristSubsystem(new WristIO() {});

        climbSubsystem = new ClimbSubsystem(ClimbConstants.config, new TalonFXIO(ClimbConstants.config));

        intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});

        break;
    }

    //Auto Commands

    // NamedCommands.registerCommand(
    //     "Arm L4", SuperStructureCommandFactory.genericPreset(0.205, 1.07, 0.337).asProxy());

    // NamedCommands.registerCommand(
    //     "Arm L4P", SuperStructureCommandFactory.feederToReef(0.205, 1.07, 0.337).asProxy());

    // NamedCommands.registerCommand(
    //     "L4 to Feeder", SuperStructureCommandFactory.reefToFeeder(0, 0, 0).asProxy());

    // NamedCommands.registerCommand(
    //     "Arm in Drive", SuperStructureCommandFactory.autoPreset(0.21, 0.8, 0.22, 0.25).asProxy());

    // NamedCommands.registerCommand(
    //     "Outtake", (intakeSubsystem.dutyCycleCommand(() -> -0.3)).withTimeout(0.5));

    // NamedCommands.registerCommand(
    //     "Arm Neutral", SuperStructureCommandFactory.genericPreset(0.21, 0.4, 0.22).asProxy());

    // NamedCommands.registerCommand(
    //     "Arm Feeder While Moving",
    //     SuperStructureCommandFactory.genericPreset(0.15, 0.045, 0.71).asProxy());

    // NamedCommands.registerCommand(
    //     "Arm Feeder", SuperStructureCommandFactory.genericPreset(0.15, 0.045, 0.71).asProxy());

    // NamedCommands.registerCommand(
    //     "Intake", (intakeSubsystem.dutyCycleCommand(() -> 0.5)).withTimeout(1.5));

    // NamedCommands.registerCommand("Swerve Stop", driveSubsystem.runOnce(driveSubsystem::stop));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    autoChooser.addDefaultOption("None", Commands.none());

    // autoChooser.addOption("Right Level 2 Middle ID 21", AutoBuilder.buildAuto("RL2 Mid"));
    // autoChooser.addOption("Left a lot of coral", AutoBuilder.buildAuto("2pl"));
    // autoChooser.addOption("Asheville Auto", AutoBuilder.buildAuto("Left 2 Piece"));
    // autoChooser.addOption("3 Piece", AutoBuilder.buildAuto("3PL4"));
    // autoChooser.addOption("Test Path", AutoBuilder.buildAuto("test"));
    // autoChooser.addOption("Big Boi Left", AutoBuilder.buildAuto("Big Boi"));
    // autoChooser.addOption("Big Boi Right", new PathPlannerAuto("Big Boi", true));

    // configure button bindings
    configureButtonBindings();
    fastLoop.register(armSubsystem);
    fastLoop.register(elevatorSubsystem);
    // fastLoop.register(intakeSubsystem);
    fastLoop.register(wristSubsystem);
    fastLoop.start();
  }

  // Button Binds
  private void configureButtonBindings() {

    // Driver Binds

    //Temp Climb
    //driveController.povUp().whileTrue(climbSubsystem.runUntilStall());

    // Algae Mode Toggle
    driveController
        .R3()
        .whileTrue(Commands.startEnd(() -> algaeMode = true, () -> algaeMode = false));

    // Trough Mode Toggle
    driveController
        .L3()
        .whileTrue(Commands.startEnd(() -> troughMode = true, () -> troughMode = false));

//     // Trough Mode Toggle
//     driveController
//         .L3()
//         .whileTrue(Commands.startEnd(() -> troughMode = true, () -> troughMode = false));

//     // Arm Stop
//     driveController
//         .PS()
//         .onTrue(
//             armSubsystem
//                 .setSetpointHere()
//                 .alongWith(elevatorSubsystem.setSetpointHere())
//                 .alongWith(wristSubsystem.setSetpointHere()));

    // Swerve Drive
    driveSubsystem.setDefaultCommand(
        driveSubsystem.run(
            () ->
                driveSubsystem.teleopControl(
                    -driveController.getLeftY(),
                    -driveController.getLeftX(),
                    -driveController.getRightX())));

    // Gyro Rotation Reset
    driveController.options().onTrue(driveSubsystem.runOnce(driveSubsystem::teleopResetRotation));

    // //Default Intake Pos?
    // driveController.square().onTrue(SuperStructureCommandFactory.defaultparallelPreset());

    // //Default Intake Pos?
    // driveController.povRight().onTrue(SuperStructureCommandFactory.neutralparallelPreset());

    //Intake Coral
    //Works
    driveController.R1().whileTrue(intakeSubsystem.intakeCoralCommand());

    //Outtake Coral
    //Works; not automated
    //driveController.L1().whileTrue(intakeSubsystem.outtakeCoralBackCommand());
    driveController.L1().whileTrue(intakeSubsystem.outtakeCoralFrontCommand());

    // Algae Intake
    driveController.R1().and(inAlgaeMode).whileTrue(intakeSubsystem.intakeAlgaeCommand())
        .onFalse(intakeSubsystem.holdAlgaeCommand());

    // Algae Outtake
    driveController.L1().and(inAlgaeMode).whileTrue(intakeSubsystem.outtakeAlgaeCommand());

    //Coral Ground Intake Pos

    //Algae Ground Intake Pos

    //Front L4
    driveController
    .triangle()
    .and(inCoralMode)
    //.and(facingForward)
    .onTrue(SuperStructureCommandFactory.defaultparallelPreset(0.196, 1.03, 0.275));

    //Front L3

    //Front L2

    //Front High Algae
    driveController
    .circle()
    .and(inAlgaeMode)
    //.and(facingForward)
    .onTrue(SuperStructureCommandFactory.defaultparallelPreset(0.188, 0.445, 0.545));

    //Front Low Algae
    driveController
    .cross()
    .and(inAlgaeMode)
    //.and(facingForward)
    .onTrue(SuperStructureCommandFactory.defaultparallelPreset(0.164, 0.2, 0.565));



//Good Presets

    // //Back L4
    //driveController
    //.triangle()
    // .and(inCoralMode)
    // //.and(facingBackward)
    // .onTrue(
    //     SuperStructureCommandFactory.defaultparallelPreset(0.243, 1.085, 0));

    // //Back L3
    // driveController
    // .circle()
    // .and(inCoralMode)
    // //.and(facingBackward)
    // .onTrue(SuperStructureCommandFactory.defaultparallelPreset(0.24, 0.34, 19.75/360));

    // //Back L2
    // driveController
    // .cross()
    // .and(inCoralMode)
    // //.and(facingBackward)
    // .onTrue(SuperStructureCommandFactory.defaultparallelPreset(0.225, 0, 10.0/360));

    // //Back High Algae
    // driveController
    // .circle()
    // .and(inAlgaeMode)
    // //.and(facingBackward)
    // .onTrue(SuperStructureCommandFactory.defaultparallelPreset(0.23, 0.36, 0.2));

    // //Back Low Algae
    // driveController
    // .cross()
    // .and(inAlgaeMode)
    // //.and(facingBackward)
    // .onTrue(SuperStructureCommandFactory.defaultparallelPreset(0.23, 0, 0.2));


    //Processor

    //Barge
    
    
    //Default Pos
    driveController.povRight().onTrue(SuperStructureCommandFactory.defaultparallelPreset(0.2, 0.2, 0));

    //Stow
    driveController.povLeft().onTrue(SuperStructureCommandFactory.stow());

    //Intake Pos
    driveController.square().onTrue(SuperStructureCommandFactory.defaultparallelPreset(-5.0/360, 0.065, 0.374));




//     // Auto Align Arm Neutral Pos
//     driveController
//         .L2()
//         .and(() -> wristSubsystem.getCurrentPosition() > 0.65)
//         .onTrue(SuperStructureCommandFactory.genericPreset(0.21, 0.043, 0.22));
//     driveController
//         .R2()
//         .and(() -> wristSubsystem.getCurrentPosition() > 0.65)
//         .onTrue(SuperStructureCommandFactory.genericPreset(0.21, 0.045, 0.22));

//     // Auto Align Options
//     driveController.L2().whileTrue(Commands.runEnd(() -> setRight(false), () -> setRight(true)));
//     driveController.R2().whileTrue(Commands.runEnd(() -> setLeft(false), () -> setLeft(true)));

// Operator Binds


//Operator Manual Arm Override
new Trigger(() -> Math.abs(operatorController.getLeftY()) > 0.1)
        .whileTrue(armSubsystem.runDutyCycle(() -> 0.3* operatorController.getLeftY()));

//Operator Manual Wrist Override        
new Trigger(() -> Math.abs(operatorController.getRightY()) > 0.1)
        .whileTrue(wristSubsystem.runDutyCycle(() -> 0.15* operatorController.getRightY()));

//Operator Manual Elevator Override
new Trigger(() -> Math.abs((operatorController.getR2Axis() -operatorController.getL2Axis()) / 2) > 0.1)
        .whileTrue(elevatorSubsystem.runDutyCycle(() -> 0.25* ((operatorController.getR2Axis() -operatorController.getL2Axis()) / 2)));


    // Operator Elevator Zero
    operatorController.cross().onTrue(SuperStructureCommandFactory.zeroElevator());
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
    if (Constants.getRobot() != Constants.RobotType.SIMBOT) return;

    SimulatedArena.getInstance().resetFieldForAuto();
    driveSubsystem.setPose(new Pose2d(2, 2, Rotation2d.kZero));
  }

  public void displaySimFieldToAdvantageScope() {
    if (Constants.getRobot() != Constants.RobotType.SIMBOT) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
