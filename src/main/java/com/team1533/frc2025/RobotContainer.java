// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025;

import static com.team1533.frc2025.subsystems.vision.VisionConstants.camera0Name;
import static com.team1533.frc2025.subsystems.vision.VisionConstants.robotToCamera0;
import static com.team1533.frc2025.subsystems.vision.VisionConstants.robotToCamera1;
import static com.team1533.frc2025.subsystems.vision.VisionConstants.robotToCamera2;

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
  @AutoLogOutput @Getter private boolean coralMode = false;
  @Getter @AutoLogOutput @Setter private boolean left = true;
  @Getter @AutoLogOutput @Setter private boolean right = true;
  @Getter @AutoLogOutput @Setter private boolean isFacingForward = true;

  private Trigger inCoralMode = new Trigger(() -> !algaeMode);
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
                state,
                new VisionIOPhotonVision(VisionConstants.camera0Name, robotToCamera0),
                new VisionIOPhotonVision(VisionConstants.camera1Name, robotToCamera1),
                new VisionIOPhotonVision(VisionConstants.camera2Name, robotToCamera2));

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


    //The Basics .tm

    // Algae Mode Toggle
    driveController
        .R3()
        .whileTrue(Commands.startEnd(() -> algaeMode = true, () -> algaeMode = false));
        //.onTrue(Commands.runOnce(() -> algaeMode = !algaeMode));

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

    // Auto Align Options
    driveController.L2().whileTrue(Commands.runEnd(() -> setRight(false), () -> setRight(true)));
    driveController.R2().whileTrue(Commands.runEnd(() -> setLeft(false), () -> setLeft(true)));


    //Intake Commands/Presets

    //Coral Intake
    driveController
    .R1()
    .and(inCoralMode)
    .whileTrue(SuperStructureCommandFactory.stow()
    .andThen(SuperStructureCommandFactory.intakingIsBad(-5.0/360, 0.078, 0.377))
    .andThen(intakeSubsystem.intakeCoralCommand()))
    .onFalse(SuperStructureCommandFactory.moveWristOnly(0.3)
    .andThen(SuperStructureCommandFactory.defaultPos()));

    //Trough Intake
    driveController
    .L3()
    .and(inCoralMode)
    .whileTrue(SuperStructureCommandFactory.stow()
    .andThen(SuperStructureCommandFactory.intakingIsBad(-5.0/360, 0.078, 0.377))
    .andThen(intakeSubsystem.intakeTroughCommand()))
    .onFalse(SuperStructureCommandFactory.moveWristOnly(0.3)
    .andThen(SuperStructureCommandFactory.defaultPos()))
    .onFalse(intakeSubsystem.spinAlgaeRollersCommand(0.25));

    //Algae Ground Intake Pos
    driveController
    .square()
    .and(inAlgaeMode)
    .onTrue(SuperStructureCommandFactory.stow().andThen(SuperStructureCommandFactory.intakingIsBad(0.05, 0.1687, 0.5781)))
    .onFalse(SuperStructureCommandFactory.moveWristOnly(0.3)
    .andThen(SuperStructureCommandFactory.defaultPos()));

    //Outtake Coral Front
    driveController
    .L1().and(facingForward)
    .whileTrue(intakeSubsystem.spinCoralRollersCommand(-0.75, 0.75, -0.75));

    //Outtake Coral Back
    driveController.L1()
    .and(facingBackward)
    .whileTrue(intakeSubsystem.spinCoralRollersCommand(0.75,-0.75,0.75));

    // Algae Intake
    driveController.R1().and(inAlgaeMode).whileTrue(intakeSubsystem.spinAlgaeRollersCommand(-0.75))
        .onFalse(intakeSubsystem.spinAlgaeRollersCommand(-0.15));

    // Algae Outtake
    driveController.L1().and(inAlgaeMode).whileTrue(intakeSubsystem.spinAlgaeRollersCommand(0.75));

    //The other shit .tm

    // L1/Trough
    driveController.square()
    .and(inCoralMode)
    .onTrue(SuperStructureCommandFactory.defaultParallelPreset(0.1206, 0.0742, 0.492));

    //Front L4
    driveController
    .triangle()
    .and(inCoralMode)
    .and(facingForward)
    .onTrue(SuperStructureCommandFactory.scoringParallelPreset(0.187, 1.054, 0.251, true));

    //Front L3
    driveController
    .circle()
    .and(inCoralMode)
    .and(facingForward)
    .onTrue(SuperStructureCommandFactory.scoringParallelPreset(0.145, 0.492, 0.128, true));

    //Front L2
    driveController
    .cross()
    .and(inCoralMode)
    .and(facingForward)
    .onTrue(SuperStructureCommandFactory.scoringParallelPreset(0.0863, 0.278, 0.0586, true));

    //Front High Algae
    driveController
    .circle()
    .and(inAlgaeMode)
    .and(facingForward)
    .onTrue(SuperStructureCommandFactory.defaultParallelPreset(0.188, 0.445, 0.545));

    //Front Low Algae
    driveController
    .cross()
    .and(inAlgaeMode)
    .and(facingForward)
    .onTrue(SuperStructureCommandFactory.defaultParallelPreset(0.164, 0.2, 0.565));

    //Default Pos
    driveController.povRight().onTrue(SuperStructureCommandFactory.defaultPos());

    //Stow
    driveController.povLeft().onTrue(SuperStructureCommandFactory.stow());

    //Back L4
    driveController
    .triangle()
    .and(inCoralMode)
    .and(facingBackward)
    .onTrue(SuperStructureCommandFactory.scoringParallelPreset(0.243, 1.085, 0, false));

    //Back L3
    driveController
    .circle()
    .and(inCoralMode)
    .and(facingBackward)
    .onTrue(SuperStructureCommandFactory.scoringParallelPreset(0.24, 0.34, 19.75/360, false));

    //Back L2
    driveController
    .cross()
    .and(inCoralMode)
    .and(facingBackward)
    .onTrue(SuperStructureCommandFactory.scoringParallelPreset(0.225, 0, 10.0/360, false));

    //Back High Algae
    driveController
    .circle()
    .and(inAlgaeMode)
    .and(facingBackward)
    .onTrue(SuperStructureCommandFactory.defaultParallelPreset(0.23, 0.36, 0.2));

    //Back Low Algae
    driveController
    .cross()
    .and(inAlgaeMode)
    .and(facingBackward)
    .onTrue(SuperStructureCommandFactory.defaultParallelPreset(0.23, 0, 0.2));

    //Processor

    //Barge

    //Climb
    driveController
    .povUp()
    .onTrue(SuperStructureCommandFactory.climb());

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
