// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team1533.frc2025.RobotState;
import com.team1533.lib.loops.IStatusSignalLoop;
import com.team1533.lib.time.RobotTime;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final RobotState state;

  public ShooterSubsystem(final ShooterIO io) {
    this.io = io;
    this.state = RobotState.getInstance();
  }


  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);


    Logger.recordOutput("Shooter/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
  }


  public Command runDutyCycle(DoubleSupplier percentOutput) {
    return runEnd(() -> io.setDutyCycleOut(percentOutput.getAsDouble()), () -> io.setDutyCycleOut(0.0))
        .withName("Arm DutyCycleControl");
  }

  public Command runVoltage(DoubleSupplier percentOutput) {
    return runEnd(() -> io.runVolts(percentOutput.getAsDouble()), () -> io.setDutyCycleOut(0.0))
        .withName("Arm DutyCycleControl");
  }

}
