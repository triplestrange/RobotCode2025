// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.climb;

import com.team1533.frc2025.RobotState;
import com.team1533.frc2025.subsystems.elevator.ElevatorConstants;
import com.team1533.lib.subsystems.MotorIO;
import com.team1533.lib.subsystems.MotorInputsAutoLogged;
import com.team1533.lib.subsystems.ServoMotorSubsystem;
import com.team1533.lib.subsystems.ServoMotorSubsystemConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  private final RobotState state;

  public ClimbSubsystem(ServoMotorSubsystemConfig c, final MotorIO io) {
    super(c, new MotorInputsAutoLogged(), io);
    this.state = RobotState.getInstance();
    setDefaultCommand(dutyCycleCommand(() -> 0.0).withName("Zero Climb Duty Cycle"));
  }

  @Override
  public void periodic() {
    super.periodic();
  }

//Fix
  public Command runUntilStall() {

    return dutyCycleCommand(
            (() -> io.setDutyCycleOutIgnoreLimits()),
            () -> {
              io.zero();
              zerod = true;
            })
        .until(
            () ->
                (currentFilterValue > ElevatorConstants.blockedCurrent
                    && MathUtil.isNear(0, inputs.leaderVelocityRotPerSec, 0.1)))
        .withName("Elevator Zero Command");
  }
}
