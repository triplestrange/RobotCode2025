// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team1533.frc2025.Constants.Gains;
import com.team1533.lib.util.CTREUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.Arrays;
import java.util.List;

public class ShooterIOReal implements ShooterIO {
  protected final TalonFX leaderTalon;
  protected final TalonFX follower1Talon;
  protected final TalonFX follower2Talon;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final DutyCycleOut dutyCycleOutControl =
      new DutyCycleOut(0).withEnableFOC(true);
  private final VelocityVoltage voltageVelocity = new VelocityVoltage(0).withEnableFOC(true);

  private final StatusSignal<AngularVelocity> leaderVelocitySignal;
  private final StatusSignal<Voltage> leaderVoltsSignal;
  private final StatusSignal<Current> leaderCurrentStatorSignal;
  private final StatusSignal<Current> leaderCurrentSupplySignal;
  private final StatusSignal<Temperature> leaderTemperatureSignal;

  private final StatusSignal<AngularVelocity> follower1VelocitySignal;
  private final StatusSignal<Voltage> follower1VoltsSignal;
  private final StatusSignal<Current> follower1CurrentStatorSignal;
  private final StatusSignal<Current> follower1CurrentSupplySignal;
  private final StatusSignal<Temperature> follower1TemperatureSignal;

  private final StatusSignal<AngularVelocity> follower2VelocitySignal;
  private final StatusSignal<Voltage> follower2VoltsSignal;
  private final StatusSignal<Current> follower2CurrentStatorSignal;
  private final StatusSignal<Current> follower2CurrentSupplySignal;
  private final StatusSignal<Temperature> follower2TemperatureSignal;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public ShooterIOReal() {
    leaderTalon = new TalonFX(ShooterConstants.leaderTalonCanID, ShooterConstants.canBUS);
    follower1Talon = new TalonFX(ShooterConstants.follower1TalonCanID, ShooterConstants.canBUS);
    follower1Talon.setControl(new Follower(ShooterConstants.leaderTalonCanID, false));
    follower2Talon = new TalonFX(ShooterConstants.follower2TalonCanID, ShooterConstants.canBUS);
    follower2Talon.setControl(new Follower(ShooterConstants.leaderTalonCanID, false));

    // Leader motor configs
    config.Slot0.kP = ShooterConstants.gains.kP();
    config.Slot0.kI = ShooterConstants.gains.kI();
    config.Slot0.kD = ShooterConstants.gains.kD();
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.TorqueCurrent.PeakForwardTorqueCurrent = ShooterConstants.torqueCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -ShooterConstants.torqueCurrentLimit;
    config.MotorOutput.Inverted =
        ShooterConstants.leaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = ShooterConstants.SensorToMechanismRatio;
    config.Feedback.RotorToSensorRatio = ShooterConstants.reduction;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ShooterConstants.statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLowerLimit = ShooterConstants.supplyCurrentLowerLimit;
    config.CurrentLimits.SupplyCurrentLowerTime = ShooterConstants.supplyCurrentLowerLimitTime;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Base Status Signals

    leaderVelocitySignal = leaderTalon.getRotorVelocity();
    leaderVoltsSignal = leaderTalon.getMotorVoltage();
    leaderCurrentStatorSignal = leaderTalon.getStatorCurrent();
    leaderCurrentSupplySignal = leaderTalon.getSupplyCurrent();
    leaderTemperatureSignal = leaderTalon.getDeviceTemp();

    follower1VelocitySignal = follower1Talon.getRotorVelocity();
    follower1VoltsSignal = follower1Talon.getMotorVoltage();
    follower1CurrentStatorSignal = follower1Talon.getStatorCurrent();
    follower1CurrentSupplySignal = follower1Talon.getSupplyCurrent();
    follower1TemperatureSignal = follower1Talon.getDeviceTemp();    

    follower2VelocitySignal = follower2Talon.getRotorVelocity();
    follower2VoltsSignal = follower2Talon.getMotorVoltage();
    follower2CurrentStatorSignal = follower2Talon.getStatorCurrent();
    follower2CurrentSupplySignal = follower2Talon.getSupplyCurrent();
    follower2TemperatureSignal = follower2Talon.getDeviceTemp();

    CTREUtil.applyConfiguration(leaderTalon, config);
    CTREUtil.applyConfiguration(follower1Talon, config);
    CTREUtil.applyConfiguration(follower2Talon, config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        10,
        leaderVelocitySignal,
        leaderVoltsSignal,
        leaderCurrentStatorSignal,
        leaderCurrentSupplySignal,
        leaderTemperatureSignal,
        follower1VelocitySignal,
        follower1VoltsSignal,
        follower1CurrentStatorSignal,
        follower1CurrentSupplySignal,
        follower1TemperatureSignal,
        follower2VelocitySignal,
        follower2VoltsSignal,
        follower2CurrentStatorSignal,
        follower2CurrentSupplySignal,
        follower2TemperatureSignal
    );

    // Optimize bus utilization
    leaderTalon.optimizeBusUtilization(0, 1.0);
    follower1Talon.optimizeBusUtilization(0, 1.0);
    follower2Talon.optimizeBusUtilization(0, 1.0);

    voltageOut.EnableFOC = true;
    dutyCycleOutControl.EnableFOC = true;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leaderConnected =
        BaseStatusSignal.refreshAll(
                leaderVelocitySignal,
                leaderVoltsSignal,
                leaderCurrentStatorSignal,
                leaderCurrentSupplySignal,
                leaderTemperatureSignal
            ).isOK();

    inputs.leaderVelocityRotPerSec = leaderVelocitySignal.getValueAsDouble();
    inputs.leaderAppliedVolts = leaderVoltsSignal.getValueAsDouble();
    inputs.leaderCurrentAmps = leaderCurrentSupplySignal.getValueAsDouble();
    inputs.leaderStatorAmps = leaderCurrentStatorSignal.getValueAsDouble();
    inputs.leaderTempCelc = leaderTemperatureSignal.getValueAsDouble();

    inputs.follower1Connected =
        BaseStatusSignal.refreshAll(
                follower1VelocitySignal,
                follower1VoltsSignal,
                follower1CurrentStatorSignal,
                follower1CurrentSupplySignal,
                follower1TemperatureSignal
            ).isOK();
    inputs.follower1VelocityRotPerSec = follower1VelocitySignal.getValueAsDouble();
    inputs.follower1AppliedVolts = follower1VoltsSignal.getValueAsDouble();
    inputs.follower1CurrentAmps = follower1CurrentSupplySignal.getValueAsDouble();
    inputs.follower1StatorAmps = follower1CurrentStatorSignal.getValueAsDouble();
    inputs.follower1TempCelc = follower1TemperatureSignal.getValueAsDouble();

    inputs.follower2Connected =
        BaseStatusSignal.refreshAll(
                follower2VelocitySignal,
                follower2VoltsSignal,
                follower2CurrentStatorSignal,
                follower2CurrentSupplySignal,
                follower2TemperatureSignal
            ).isOK();
    inputs.follower2VelocityRotPerSec = follower2VelocitySignal.getValueAsDouble();
    inputs.follower2AppliedVolts = follower2VoltsSignal.getValueAsDouble();
    inputs.follower2CurrentAmps = follower2CurrentSupplySignal.getValueAsDouble();
    inputs.follower2StatorAmps = follower2CurrentStatorSignal.getValueAsDouble();
    inputs.follower2TempCelc = follower2TemperatureSignal.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    leaderTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setDutyCycleOut(double output) {
    leaderTalon.setControl(dutyCycleOutControl.withOutput(output));
  }

  @Override
  public void setVelocitySetpoint(double velocity) {
    leaderTalon.setControl(voltageVelocity.withVelocity(velocity));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    follower1Talon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    follower2Talon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPID(Gains gains) {
    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();
    config.Slot0.kG = gains.ffkG();
    config.Slot0.kS = gains.ffkS();
    config.Slot0.kV = gains.ffkV();
    CTREUtil.applyConfiguration(leaderTalon, config);
  }

  @Override
  public void stop() {
    leaderTalon.setControl(new NeutralOut());
  }
}
