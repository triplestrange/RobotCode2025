// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.funnel;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1533.frc2025.Constants;
import com.team1533.frc2025.Constants.Gains;
import com.team1533.lib.drivers.CANDeviceId;
import com.team1533.lib.subsystems.ServoMotorSubsystemConfig;

public class FunnelConstants {

  public static final CANDeviceId funnelTalon = new CANDeviceId(30, "rio");

  public static final boolean leaderInverted = false;

  public static final double reduction = 64.0 / 11.0;

  public static final double frictionVoltage = 0.25;

  public static final double torqueCurrentLimit = 80;
  public static final double statorCurrentLimit = 80;
  public static final double supplyCurrentLimit = 60;
  public static final double supplyCurrentLowerLimit = 40;
  public static final double supplyCurrentLowerLimitTime = 1;
  // TODO: tune these
  public static final double motionMagicCruiseVelocity = 0;
  public static final double MotionMagicExpo_kA = 12.0 / 63.28;
  public static final double MotionMagicExpo_kV = 12.0 / 13.23;

  public static final double reverseSoftLimitThreshold = 0;
  public static final double forwardSoftLimitThreshold = 0.712;

  public static final double toleranceRotations = 0.05;
  public static final double blockedCurrent = 60;

  public static final Gains gains =
      switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(20, 0.0, 0, 0, 0.0, 0.0, 0);
        default -> new Gains(0, 0, 0, 0, 0, 0, 0);
      };

  public static final ServoMotorSubsystemConfig config = new ServoMotorSubsystemConfig();

  static {
    config.fxConfig.TorqueCurrent.PeakForwardTorqueCurrent = torqueCurrentLimit;
    config.fxConfig.TorqueCurrent.PeakReverseTorqueCurrent = -torqueCurrentLimit;

    config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.fxConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    config.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    config.fxConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    config.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.fxConfig.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentLowerLimit;
    config.fxConfig.CurrentLimits.SupplyCurrentLowerTime = supplyCurrentLowerLimitTime;

    config.name = "Funnel";

    config.unitToRotorRatio = 1.0 / reduction;

    config.talonCANID = funnelTalon;

    config.fxConfig.MotorOutput.Inverted =
        leaderInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // TODO: tune these
    config.fxConfig.MotionMagic.MotionMagicExpo_kA = MotionMagicExpo_kA;
    config.fxConfig.MotionMagic.MotionMagicExpo_kV = MotionMagicExpo_kV;
    config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicCruiseVelocity;

    config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimitThreshold;
    config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimitThreshold;

    config.fxConfig.Slot0.kP = gains.kP();
    config.fxConfig.Slot0.kI = gains.kI();
    config.fxConfig.Slot0.kD = gains.kD();
    config.fxConfig.Slot0.kG = gains.ffkG();
    config.fxConfig.Slot0.kS = gains.ffkS();
    config.fxConfig.Slot0.kV = gains.ffkV();
    config.fxConfig.Slot0.kA = gains.ffkA();
  }
}
