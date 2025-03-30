// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

public class ElevatorIOReal implements ElevatorIO {
  protected final TalonFX leaderTalon;
  protected final TalonFX followerTalon;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(0.0);
  private final DutyCycleOut dutyCycleOutControl =
      new DutyCycleOut(0).withEnableFOC(true).withUpdateFreqHz(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0).withUpdateFreqHz(0.0);
  private final MotionMagicExpoVoltage motionMagicVoltage =
      new MotionMagicExpoVoltage(0).withUpdateFreqHz(0.0);

  private final StatusSignal<Angle> leaderPositionSignal;
  private final StatusSignal<AngularVelocity> leaderVelocitySignal;
  private final StatusSignal<Voltage> leaderVoltsSignal;
  private final StatusSignal<Current> leaderCurrentStatorSignal;
  private final StatusSignal<Current> leaderCurrentSupplySignal;
  private final StatusSignal<Temperature> leaderTemperatureSignal;

  private final StatusSignal<Angle> followerPositionSignal;
  private final StatusSignal<AngularVelocity> followerVelocitySignal;
  private final StatusSignal<Voltage> followerVoltsSignal;
  private final StatusSignal<Current> followerCurrentStatorSignal;
  private final StatusSignal<Current> followerCurrentSupplySignal;
  private final StatusSignal<Temperature> followerTemperatureSignal;

  private final StatusSignal<Angle> elevatorPositionSignal;
  private final StatusSignal<AngularVelocity> elevatorVelocitySignal;
  private final StatusSignal<AngularAcceleration> elevatorAccelerationSignal;

  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public ElevatorIOReal() {

    leaderTalon = new TalonFX(ElevatorConstants.leaderTalonCanID, ElevatorConstants.canBUS);
    followerTalon = new TalonFX(ElevatorConstants.followerTalonCanID, ElevatorConstants.canBUS);
    followerTalon.setControl(new Follower(ElevatorConstants.leaderTalonCanID, true));

    // Leader motor configs
    config.Slot0.kP = ElevatorConstants.gains.kP();
    config.Slot0.kI = ElevatorConstants.gains.kI();
    config.Slot0.kD = ElevatorConstants.gains.kD();
    config.Slot0.kA = ElevatorConstants.gains.ffkA();
    config.Slot0.kG = ElevatorConstants.gains.ffkG();
    config.Slot0.kS = ElevatorConstants.gains.ffkS();
    config.Slot0.kV = ElevatorConstants.gains.ffkV();

    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.TorqueCurrent.PeakForwardTorqueCurrent = ElevatorConstants.torqueCurrentLimit;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -ElevatorConstants.torqueCurrentLimit;
    config.MotorOutput.Inverted =
        ElevatorConstants.leaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ElevatorConstants.reduction;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.supplyCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ElevatorConstants.statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLowerLimit = ElevatorConstants.supplyCurrentLowerLimit;
    config.CurrentLimits.SupplyCurrentLowerTime = ElevatorConstants.supplyCurrentLowerLimitTime;

    config.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = ElevatorConstants.motionMagicAcceleration;
    config.MotionMagic.MotionMagicJerk = ElevatorConstants.motionMagicJerk;
    config.MotionMagic.MotionMagicExpo_kA = ElevatorConstants.motionMagicExpo_kA;
    config.MotionMagic.MotionMagicExpo_kV = ElevatorConstants.motionMagicExpo_kV;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        ElevatorConstants.forwardSoftLimitThreshold;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        ElevatorConstants.reverseSoftLimitThreshold;

    // Base Status Signals
    leaderPositionSignal = leaderTalon.getRotorPosition();
    leaderVelocitySignal = leaderTalon.getRotorVelocity();
    leaderVoltsSignal = leaderTalon.getMotorVoltage();
    leaderCurrentStatorSignal = leaderTalon.getStatorCurrent();
    leaderCurrentSupplySignal = leaderTalon.getSupplyCurrent();
    leaderTemperatureSignal = leaderTalon.getDeviceTemp();

    followerPositionSignal = followerTalon.getRotorPosition();
    followerVelocitySignal = followerTalon.getRotorVelocity();
    followerVoltsSignal = followerTalon.getMotorVoltage();
    followerCurrentStatorSignal = followerTalon.getStatorCurrent();
    followerCurrentSupplySignal = followerTalon.getSupplyCurrent();
    followerTemperatureSignal = followerTalon.getDeviceTemp();

    elevatorPositionSignal = leaderTalon.getPosition();
    elevatorVelocitySignal = leaderTalon.getVelocity();
    elevatorAccelerationSignal = leaderTalon.getAcceleration();

    CTREUtil.applyConfiguration(leaderTalon, config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leaderPositionSignal,
        leaderVelocitySignal,
        leaderVoltsSignal,
        leaderCurrentStatorSignal,
        leaderCurrentSupplySignal,
        leaderTemperatureSignal,
        followerPositionSignal,
        followerVelocitySignal,
        followerVoltsSignal,
        followerCurrentStatorSignal,
        followerCurrentSupplySignal,
        followerTemperatureSignal,
        elevatorAccelerationSignal);
    BaseStatusSignal.setUpdateFrequencyForAll(250, elevatorPositionSignal, elevatorVelocitySignal);

    // Optimize bus utilization
    leaderTalon.optimizeBusUtilization(0, 1.0);
    followerTalon.optimizeBusUtilization(0, 1.0);

    voltageOut.EnableFOC = true;
    dutyCycleOutControl.EnableFOC = true;
    motionMagicVoltage.EnableFOC = true;

    leaderTalon.setPosition(0);
  }

  @Override
  public List<BaseStatusSignal> getStatusSignals() {
    // Only read position and velocity at 250 hz
    return Arrays.asList(elevatorPositionSignal);
  }

  @Override
  public void updateFastInputs(FastElevatorIOInputs inputs) {
    double position =
        BaseStatusSignal.getLatencyCompensatedValueAsDouble(
            elevatorPositionSignal, elevatorVelocitySignal);

    inputs.elevatorPosMeters = position;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leaderConnected =
        BaseStatusSignal.refreshAll(
                leaderPositionSignal,
                leaderVelocitySignal,
                leaderVoltsSignal,
                leaderCurrentStatorSignal,
                leaderCurrentSupplySignal,
                elevatorAccelerationSignal,
                elevatorPositionSignal,
                elevatorVelocitySignal,
                leaderTemperatureSignal)
            .isOK();
    inputs.leaderVelocityRotPerSec = leaderVelocitySignal.getValueAsDouble();
    inputs.leaderAppliedVolts = leaderVoltsSignal.getValueAsDouble();
    inputs.leaderCurrentAmps = leaderCurrentSupplySignal.getValueAsDouble();
    inputs.leaderStatorAmps = leaderCurrentStatorSignal.getValueAsDouble();
    inputs.leaderTempCelc = leaderTemperatureSignal.getValueAsDouble();

    inputs.followerConnected =
        BaseStatusSignal.refreshAll(
                followerPositionSignal,
                followerVelocitySignal,
                followerVoltsSignal,
                followerCurrentStatorSignal,
                followerCurrentSupplySignal,
                followerTemperatureSignal)
            .isOK();
    inputs.followerVelocityRotPerSec = followerVelocitySignal.getValueAsDouble();
    inputs.followerAppliedVolts = followerVoltsSignal.getValueAsDouble();
    inputs.followerCurrentAmps = followerCurrentSupplySignal.getValueAsDouble();
    inputs.followerStatorAmps = followerCurrentStatorSignal.getValueAsDouble();
    inputs.followerTempCelc = followerTemperatureSignal.getValueAsDouble();

    inputs.leaderRotPosition = leaderPositionSignal.getValueAsDouble();
    inputs.followerRotPosition = followerPositionSignal.getValueAsDouble();

    inputs.elevatorVelMetersPerSecond = elevatorVelocitySignal.getValueAsDouble();
    inputs.elevatorAccelMetersPerSecondPerSecond = elevatorAccelerationSignal.getValueAsDouble();
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
  public void setPositionSetpoint(double positionMeters) {
    leaderTalon.setControl(positionTorqueCurrentFOC.withPosition(positionMeters));
  }

  @Override
  public void setPositionSetpoint(double positionMeters, double metersPerSec) {
    leaderTalon.setControl(
        positionTorqueCurrentFOC
            .withPosition(positionMeters)
            .withVelocity(metersPerSec)
            .withFeedForward(18));
  }

  @Override
  public void setDutyCycleOutIgnoreLimits() {
    leaderTalon.setControl(dutyCycleOutControl.withEnableFOC(true).withOutput(-0.07));
  }

  @Override
  public void setCurrentSetpoint(double amps) {
    leaderTalon.setControl(currentControl.withOutput(amps));
  }

  @Override
  public void setMotionMagicSetpoint(double positionRotations) {
    leaderTalon.setControl(motionMagicVoltage.withPosition(positionRotations));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    followerTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
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
  public void zero() {
    leaderTalon.setPosition(0);
  }
}
