// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.funnel;

import com.team1533.frc2025.RobotState;
import com.team1533.lib.subsystems.MotorIO;
import com.team1533.lib.subsystems.MotorInputsAutoLogged;
import com.team1533.lib.subsystems.ServoMotorSubsystem;
import com.team1533.lib.subsystems.ServoMotorSubsystemConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class FunnelSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  private final RobotState state = RobotState.getInstance();

  public FunnelSubsystem(ServoMotorSubsystemConfig c, final MotorIO io) {
    super(c, new MotorInputsAutoLogged(), io);
    setTeleopDefaultCommand();
  }

  @Override
  public void periodic() {
    super.periodic();
    state.setFunnelRots(getCurrentPosition());
  }

  public void setTeleopDefaultCommand() {
    this.setDefaultCommand(holdSetpointCommand().withName("Funnel Maintain SetPoint"));
  }

  public Command resetZeroPoint() {

    return runEnd(
            (() -> io.setOpenLoopDutyCycle(-.07)),
            () -> {
              io.setCurrentPositionAsZero();
            })
        .until(
            () ->
                (inputs.currentStatorAmps > FunnelConstants.blockedCurrent
                    && MathUtil.isNear(0, inputs.velocityUnitsPerSecond, 0.1)))
        .withName("Funnel Zero Command");
  }
}
