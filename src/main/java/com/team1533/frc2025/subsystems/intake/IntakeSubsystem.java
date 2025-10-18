// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.intake;
import org.littletonrobotics.junction.Logger;

import com.team1533.frc2025.RobotState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team1533.lib.time.RobotTime;


public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private final RobotState state;

  public IntakeSubsystem(final IntakeIO io) {
    this.io = io;
    this.state = RobotState.getInstance();
    setDefaultCommand(run(this::stopAll).withName("Intake Stop"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void stopAll() {
    io.stopIntake();
    io.stopLRoller();
    io.stopRRoller();
  }

  public void intakeCoral() {
    io.setIntakeDutyCycleOut(0.75);
    if(inputs.lCANrangeRange && inputs.rCANrangeRange) {
      io.setRRollerDutyCycleOut(-0.75);
      io.setLRollerDutyCycleOut(-0.75);
    }
    else {
      io.setRRollerDutyCycleOut(0.75);
      io.setLRollerDutyCycleOut(-0.75);
    }
  }

  public void outtakeCoralFront() {
    io.setIntakeDutyCycleOut(0.625);
    io.setRRollerDutyCycleOut(0.75);
    io.setLRollerDutyCycleOut(-0.75);
  }

  public void outtakeCoralBack() {
    io.setIntakeDutyCycleOut(-0.25);
    io.setRRollerDutyCycleOut(-0.25);
    io.setLRollerDutyCycleOut(0.25);
  }

  public void intakeAlgae() {
    io.setIntakeDutyCycleOut(-0.75);
  }

  public void holdAlgae() {
    io.setIntakeDutyCycleOut(-0.15);
  }

  public void outtakeAlgae() {
    io.setIntakeDutyCycleOut(0.75);
  }

  //Commands

  public Command intakeCoralCommand() {
    return run(this::intakeCoral).withName("Intake Coral");
  }

  public Command outtakeCoralFrontCommand() {
    return run(this::outtakeCoralFront).withName("Outtake Coral Front");
  }

  public Command outtakeCoralBackCommand() {
    return run(this::outtakeCoralBack).withName("Outtake Coral Back");
  }

  public Command intakeAlgaeCommand() {
    return run(this::intakeAlgae).withName("Intake Algae");
  }

  public Command holdAlgaeCommand() {
    return run(this::holdAlgae).withName("Hold Algae");
  }

  public Command outtakeAlgaeCommand() {
    return run(this::outtakeAlgae).withName("Outtake Algae");
  }

}
