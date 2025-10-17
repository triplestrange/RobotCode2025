// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import com.team1533.frc2025.Constants.Gains;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public class IntakeInputs {}

  default void updateInputs(IntakeIO.IntakeInputs inputs) {}

  // Maybe add Canranges here?
  // @AutoLog
  // class FastWristIOInputs {
  //   public double FusedCANcoderPositionRots = 0.0;
  // }

  @AutoLog
  class IntakeIOInputs {
    public boolean intakeConnected = false;
    public double intakeVelocityRotPerSec = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeSupplyAmps = 0.0;
    public double intakeStatorAmps = 0.0;
    public double intakeTempCelc = 0.0;

    public boolean rRollerConnected = false;
    public double rRollerVelocityRotPerSec = 0.0;
    public double rRollerAppliedVolts = 0.0;
    public double rRollerSupplyAmps = 0.0;
    public double rRollerStatorAmps = 0.0;
    public double rRollerTempCelc = 0.0;

    public boolean lRollerConnected = false;
    public double lRollerVelocityRotPerSec = 0.0;
    public double lRollerAppliedVolts = 0.0;
    public double lRollerSupplyAmps = 0.0;
    public double lRollerStatorAmps = 0.0;
    public double lRollerTempCelc = 0.0;

    public boolean fCANrangeConnected = false;
    public boolean fCANrangeRange = false;
    
    public boolean rCANrangeConnected = false;
    public boolean rCANrangeRange = false;

    public boolean lCANrangeConnected = false;
    public boolean lCANrangeRange = false;

    public boolean bCANrangeConnected = false;
    public boolean bCANrangeRange = false;
  }

  // default void updateFastInputs(FastWristIOInputs inputs) {};

  default void updateInputs(IntakeIOInputs inputs) {}
  default void setIntakeDutyCycleOut(double output) {}
  default void setIntakeBrakeMode(boolean enabled) {}
  default void stopIntake() {}

  default void setRRollerDutyCycleOut(double output) {}
  default void setRRollerBrakeMode(boolean enabled) {}
  default void stopRRoller() {}

  default void setLRollerDutyCycleOut(double output) {}
  default void setLRollerBrakeMode(boolean enabled) {}
  default void stopLRoller() {}
}

