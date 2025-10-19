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

  //Stop All Rollers
  public void stopAll() {
    io.stopIntake();
    io.stopLRoller();
    io.stopRRoller();
  }

  //Intake Coral
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

  //Intake Trough
  public void intakeTrough() {
    if(inputs.fCANrangeRange && inputs.rCANrangeRange && inputs.lCANrangeRange) {
      io.setIntakeDutyCycleOut(0.75);
      io.setRRollerDutyCycleOut(0);
      io.setLRollerDutyCycleOut(0);
    }
    
    
    else if(inputs.rCANrangeRange && !inputs.lCANrangeRange) {
      io.setIntakeDutyCycleOut(0.75);
      io.setRRollerDutyCycleOut(0.625);
      io.setLRollerDutyCycleOut(0.625);
    }
    
    else if(inputs.lCANrangeRange && !inputs.rCANrangeRange) {
      io.setIntakeDutyCycleOut(0.75);
      io.setRRollerDutyCycleOut(-0.625);
      io.setLRollerDutyCycleOut(-0.625);
    }

    else {
      io.setIntakeDutyCycleOut(0.625);
      io.setRRollerDutyCycleOut(0.25);
      io.setLRollerDutyCycleOut(0.25);
    }
  }

 //Coral Shifting
 public void coralShift(boolean isFacingForward) {
  if(isFacingForward) {
    if(inputs.bCANrangeRange) {
    io.setIntakeDutyCycleOut(-0.05);
    io.setRRollerDutyCycleOut(-0.05);
    io.setLRollerDutyCycleOut(0.05);
    System.out.println("BACKKK");
  }
  
    else {
    io.setIntakeDutyCycleOut(0.05);
    io.setRRollerDutyCycleOut(0.05);
    io.setLRollerDutyCycleOut(-0.05);
    System.out.println("not BACKKK");
  }}

  else {
    if(inputs.fCANrangeRange) {
      io.setIntakeDutyCycleOut(0.05);
      io.setRRollerDutyCycleOut(0.05);
      io.setLRollerDutyCycleOut(-0.05);}
    
      else {
      io.setIntakeDutyCycleOut(-0.05);
      io.setRRollerDutyCycleOut(-0.05);
      io.setLRollerDutyCycleOut(0.05);}
  }
}

  //Spin All Rollers
  public void spinCoralRollers(double setIntakeDutyCycleOut, double setRRollerDutyCycleOut, double setLRollerDutyCycleOut) {
    io.setIntakeDutyCycleOut(setIntakeDutyCycleOut);
    io.setRRollerDutyCycleOut(setRRollerDutyCycleOut);
    io.setLRollerDutyCycleOut(setLRollerDutyCycleOut);
  }

  //Spin Algae Rollers
  public void spinAlgaeRollers(double setIntakeDutyCycleOut) {
    io.setIntakeDutyCycleOut(setIntakeDutyCycleOut);
  }


  //Commands

  public Command intakeCoralCommand() {
    return run(this::intakeCoral).withName("Intake Coral");
  }

  public Command intakeTroughCommand() {
    return run(this::intakeTrough).withName("Intake Trough");
  }

  public Command coralShiftCommand(boolean isFacingForward) {
    return run(() -> coralShift(isFacingForward));
  }

  public Command spinCoralRollersCommand(double setIntakeDutyCycleOut, double setRRollerDutyCycleOut, double setLRollerDutyCycleOut) {
    return run(() -> spinCoralRollers(setIntakeDutyCycleOut, setRRollerDutyCycleOut, setLRollerDutyCycleOut)).withName("Spin Coral Rollers");
  }

  public Command spinAlgaeRollersCommand(double setIntakeDutyCycleOut) {
    return run(() -> spinAlgaeRollers(setIntakeDutyCycleOut)).withName("Spin Algae Rollers");
  }
}
