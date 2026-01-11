// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.wrist;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1533.frc2025.Constants;
import com.team1533.frc2025.Constants.Gains;
import com.team1533.lib.drivers.CANDeviceId;
//import com.team1533.lib.drivers.TalonFXConfiguration;
import com.team1533.lib.subsystems.MotorSubsystemWithCanCoderConfig;

public class WristConstants {
  //public static final int leaderTalonCanID = 19;
  //public static final int wristEncoderCanID = 20;
  public static final CANDeviceId wristTalon = new CANDeviceId(19, "rio");
  //public static final TalonFXConfiguration wristEncoderID = new TalonFXConfiguration(20, "rio");


  public static final double reduction = (155.0 / 6);
  public static final double rotorToSensorRatio = 1.;
  public static final double sensorToMechanismRatio = 1.;
  public static final double frictionVoltage = 0.25;
  public static final double blockedCurrent = 50;

  // public static final boolean leaderInverted = false;

  // public static final double reduction = (155.0 / 6);
  // public static final double SensorToMechanismRatio = 1.0;
  // public static final double frictionVoltage = 0.25;

  // public static final double torqueCurrentLimit = 80;
  // public static final double statorCurrentLimit = 80;
  // public static final double supplyCurrentLimit = 60;
  // public static final double supplyCurrentLowerLimit = 40;
  // public static final double supplyCurrentLowerLimitTime = 1;

  // public static final double motionMagicCruiseVelocity = 6.0;
  // public static final double motionMagicAcceleration = 2;
  // public static final double motionMagicJerk = 10;
  // // TODO: tune these values
  // public static final double motionMagicExpo_kA = 12.0 / 2;
  // public static final double motionMagicExpo_kV = 12.0 / 6.0;

  // public static final double absEncoderOffset = 0.875 - 0.005859375 + 0.0259;
  // public static final double absEncoderDiscontinuity = 0.9;

  // public static final double reverseSoftLimitThreshold = 0;
  // public static final double forwardSoftLimitThreshold = 0.712;

  // public static final double toleranceRotations = 0.02;

  public static final Gains gains =
      switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(60, 0.0, 3.75, 0, 0.0, 0.0, 0);
        default -> new Gains(0, 0, 0, 0, 0, 0, 0);
      };

  public static final MotorSubsystemWithCanCoderConfig config = new MotorSubsystemWithCanCoderConfig();

  static {
    // Feedback Configs
    config.fxConfig.Feedback.RotorToSensorRatio = rotorToSensorRatio;
    config.fxConfig.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;

    config.fxConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    config.fxConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.fxConfig.CurrentLimits.StatorCurrentLimit = 100;
    config.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    config.fxConfig.CurrentLimits.SupplyCurrentLimit = 40;
    config.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.fxConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    config.fxConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

    config.name = "Wrist";

    config.kMinPositionUnits = 0;
    config.kMaxPositionUnits = 0;
    config.momentOfInertia = 1;

    config.talonCANID = wristTalon;
    //config.fxConfig = wristEncoderID;
    config.unitToRotorRatio = reduction;
  }
}
