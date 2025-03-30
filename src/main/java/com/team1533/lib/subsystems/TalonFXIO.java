// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1533.frc2025.Robot;
import com.team1533.lib.util.CTREUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXIO implements MotorIO {
  protected final TalonFX talon;
  protected final ServoMotorSubsystemConfig config;

  protected final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
  private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0);
  private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);
  private final MotionMagicExpoVoltage motionMagicPositionControl = new MotionMagicExpoVoltage(0.0);
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;

  private BaseStatusSignal[] signals;

  public TalonFXIO(ServoMotorSubsystemConfig config) {
    this.config = config;
    talon = new TalonFX(config.talonCANID.getDeviceNumber(), config.talonCANID.getBus());

    // Current limits and ramp rates do not perform well in sim.
    if (Robot.isSimulation()) {
      this.config.fxConfig.CurrentLimits = new CurrentLimitsConfigs();
      this.config.fxConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs();
      this.config.fxConfig.OpenLoopRamps = new OpenLoopRampsConfigs();
    }

    CTREUtil.applyConfiguration(talon, this.config.fxConfig);

    positionSignal = talon.getPosition();
    velocitySignal = talon.getVelocity();
    voltageSignal = talon.getMotorVoltage();
    currentStatorSignal = talon.getStatorCurrent();
    currentSupplySignal = talon.getSupplyCurrent();

    signals =
        new BaseStatusSignal[] {
          positionSignal, velocitySignal, voltageSignal, currentStatorSignal, currentSupplySignal
        };

    CTREUtil.tryUntilOK(
        () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals), talon.getDeviceID());
    CTREUtil.tryUntilOK(() -> talon.optimizeBusUtilization(), talon.getDeviceID());
  }

  private double rotorToUnits(double rotor) {
    return rotor * config.unitToRotorRatio;
  }

  private double clampPosition(double units) {
    return unitsToRotor(MathUtil.clamp(units, config.kMinPositionUnits, config.kMaxPositionUnits));
  }

  public double unitsToRotor(double units) {
    return units / config.unitToRotorRatio;
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    BaseStatusSignal.refreshAll(signals);

    inputs.unitPosition = rotorToUnits(positionSignal.getValueAsDouble());
    inputs.velocityUnitsPerSecond = rotorToUnits(velocitySignal.getValueAsDouble());
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
    inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
  }

  @Override
  public void setOpenLoopDutyCycle(double dutyCycle) {
    talon.setControl(dutyCycleControl.withOutput(dutyCycle));
  }

  @Override
  public void setPositionSetpoint(double units) {
    talon.setControl(positionVoltageControl.withPosition(clampPosition(units)));
  }

  @Override
  public void setMotionMagicSetpoint(double units) {
    talon.setControl(motionMagicPositionControl.withPosition(clampPosition(units)));
  }

  @Override
  public void setNeutralMode(NeutralModeValue mode) {
    config.fxConfig.MotorOutput.NeutralMode = mode;
    CTREUtil.applyConfiguration(talon, config.fxConfig);
  }

  @Override
  public void setEnableSoftLimits(boolean fwd, boolean rev) {
    config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = fwd;
    config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = rev;
    CTREUtil.applyConfiguration(talon, config.fxConfig);
  }

  @Override
  public void setVelocitySetpoint(double unitsPerSecond) {
    talon.setControl(velocityVoltageControl.withVelocity(unitsToRotor(unitsPerSecond)));
  }

  @Override
  public void setCurrentPositionAsZero() {
    setCurrentPosition(0.0);
  }

  @Override
  public void setCurrentPosition(double positionUnits) {
    talon.setPosition(positionUnits / config.unitToRotorRatio);
  }
}
