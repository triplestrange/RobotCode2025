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
    io.setIntakeDutyCycleOut(0.625);
    if(inputs.lCANrangeRange && inputs.rCANrangeRange) {
      io.setRRollerDutyCycleOut(-0.625);
      io.setLRollerDutyCycleOut(-0.625);
    }
    else {
      io.setRRollerDutyCycleOut(0.625);
      io.setLRollerDutyCycleOut(-0.625);
    }
  }

  public Command intakeCoralCommand(){
    return run(this::intakeCoral).withName("Intake Coral");
  }

}
