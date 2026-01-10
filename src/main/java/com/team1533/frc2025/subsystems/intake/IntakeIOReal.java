// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.intake;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team1533.lib.util.CTREUtil;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOReal implements IntakeIO {
    protected final TalonFX intakeTalon;
    protected final TalonFX rRollerTalon;
    protected final TalonFX lRollerTalon;
    protected final CANrange fCANrange;
    protected final CANrange rCANrange;
    protected final CANrange lCANrange;
    protected final CANrange bCANrange;

  private final DutyCycleOut dutyCycleOutControl =
      new DutyCycleOut(0);

  private final StatusSignal<AngularVelocity> intakeVelocitySignal;
  private final StatusSignal<Voltage> intakeVoltsSignal;
  private final StatusSignal<Current> intakeCurrentStatorSignal;
  private final StatusSignal<Current> intakeCurrentSupplySignal;
  private final StatusSignal<Temperature> intakeTemperatureSignal;

  private final StatusSignal<AngularVelocity> rRollerVelocitySignal;
  private final StatusSignal<Voltage> rRollerVoltsSignal;
  private final StatusSignal<Current> rRollerCurrentStatorSignal;
  private final StatusSignal<Current> rRollerCurrentSupplySignal;
  private final StatusSignal<Temperature> rRollerTemperatureSignal;

  private final StatusSignal<AngularVelocity> lRollerVelocitySignal;
  private final StatusSignal<Voltage> lRollerVoltsSignal;
  private final StatusSignal<Current> lRollerCurrentStatorSignal;
  private final StatusSignal<Current> lRollerCurrentSupplySignal;
  private final StatusSignal<Temperature> lRollerTemperatureSignal;

  private final StatusSignal<Boolean> fCANrangeIsDectected;
  private final StatusSignal<Boolean> rCANrangeIsDectected;
  private final StatusSignal<Boolean> lCANrangeIsDectected;
  private final StatusSignal<Boolean> bCANrangeIsDectected;

  private final TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration rollerConfig = new TalonFXConfiguration();

public IntakeIOReal() {
    intakeTalon = new TalonFX(IntakeConstants.intakeTalonCanID, IntakeConstants.canBUS);
    rRollerTalon = new TalonFX(IntakeConstants.rRollerTalonCanID, IntakeConstants.canBUS);
    lRollerTalon = new TalonFX(IntakeConstants.lRollerTalonCanID, IntakeConstants.canBUS);
    fCANrange = new CANrange(IntakeConstants.fCANrangeCanID, IntakeConstants.canBUS);
    rCANrange = new CANrange(IntakeConstants.rCANrangeCanID, IntakeConstants.canBUS);
    lCANrange = new CANrange(IntakeConstants.lCANrangeCanID, IntakeConstants.canBUS);
    bCANrange = new CANrange(IntakeConstants.bCANrangeCanID, IntakeConstants.canBUS);

    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.intakeStatorCurrentLimit;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.intakeSupplyCurrentLimit;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.intakeStatorCurrentLimit;
    rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.intakeSupplyCurrentLimit;
    rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

// Base Status Signals
    intakeVelocitySignal = intakeTalon.getRotorVelocity();
    intakeVoltsSignal = intakeTalon.getMotorVoltage();
    intakeCurrentStatorSignal = intakeTalon.getStatorCurrent();
    intakeCurrentSupplySignal = intakeTalon.getSupplyCurrent();
    intakeTemperatureSignal = intakeTalon.getDeviceTemp();

    rRollerVelocitySignal = rRollerTalon.getRotorVelocity();
    rRollerVoltsSignal = rRollerTalon.getMotorVoltage();
    rRollerCurrentStatorSignal = rRollerTalon.getStatorCurrent();
    rRollerCurrentSupplySignal = rRollerTalon.getSupplyCurrent();
    rRollerTemperatureSignal = rRollerTalon.getDeviceTemp();

    lRollerVelocitySignal = lRollerTalon.getRotorVelocity();
    lRollerVoltsSignal = lRollerTalon.getMotorVoltage();
    lRollerCurrentStatorSignal = lRollerTalon.getStatorCurrent();
    lRollerCurrentSupplySignal = lRollerTalon.getSupplyCurrent();
    lRollerTemperatureSignal = lRollerTalon.getDeviceTemp();

    fCANrangeIsDectected = fCANrange.getIsDetected();
    rCANrangeIsDectected = rCANrange.getIsDetected();
    lCANrangeIsDectected = lCANrange.getIsDetected();
    bCANrangeIsDectected = rCANrange.getIsDetected();

    CTREUtil.applyConfiguration(intakeTalon, intakeConfig);
    BaseStatusSignal.setUpdateFrequencyForAll(10,
        intakeVelocitySignal,
        intakeVoltsSignal,
        intakeCurrentStatorSignal,
        intakeCurrentSupplySignal,
        intakeTemperatureSignal);

    CTREUtil.applyConfiguration(rRollerTalon, rollerConfig);
    BaseStatusSignal.setUpdateFrequencyForAll(10,
        rRollerVelocitySignal,
        rRollerVoltsSignal,
        rRollerCurrentStatorSignal,
        rRollerCurrentSupplySignal,
        rRollerTemperatureSignal);

    CTREUtil.applyConfiguration(lRollerTalon, rollerConfig);
    BaseStatusSignal.setUpdateFrequencyForAll(10,
        lRollerVelocitySignal,
        lRollerVoltsSignal,
        lRollerCurrentStatorSignal,
        lRollerCurrentSupplySignal,
        lRollerTemperatureSignal);

    BaseStatusSignal.setUpdateFrequencyForAll(50,
        fCANrangeIsDectected,
        rCANrangeIsDectected,
        lCANrangeIsDectected,
        bCANrangeIsDectected);

    // Optimize bus utilization
    intakeTalon.optimizeBusUtilization(0, 1.0);
    rRollerTalon.optimizeBusUtilization(0, 1.0);
    lRollerTalon.optimizeBusUtilization(0, 1.0);
    // fCANrange.optimizeBusUtilization(0, 1.0);
    // rCANrange.optimizeBusUtilization(0, 1.0);
    // lCANrange.optimizeBusUtilization(0, 1.0);
    // bCANrange.optimizeBusUtilization(0, 1.0);
};

@Override
public void updateInputs(IntakeIOInputs inputs) {

    //Intake
    inputs.intakeConnected =
        BaseStatusSignal.refreshAll(
                intakeVelocitySignal,
                intakeVoltsSignal,
                intakeCurrentStatorSignal,
                intakeCurrentSupplySignal,
                intakeTemperatureSignal)
            .isOK();

    inputs.intakeVelocityRotPerSec = intakeVelocitySignal.getValueAsDouble();
    inputs.intakeAppliedVolts = intakeVoltsSignal.getValueAsDouble();
    inputs.intakeSupplyAmps = intakeCurrentSupplySignal.getValueAsDouble();
    inputs.intakeStatorAmps = intakeCurrentStatorSignal.getValueAsDouble();
    inputs.intakeTempCelc = intakeTemperatureSignal.getValueAsDouble();

    //R Roller
    inputs.rRollerConnected =
        BaseStatusSignal.refreshAll(
                rRollerVelocitySignal,
                rRollerVoltsSignal,
                rRollerCurrentStatorSignal,
                rRollerCurrentSupplySignal,
                rRollerTemperatureSignal)
            .isOK();

    inputs.rRollerVelocityRotPerSec = rRollerVelocitySignal.getValueAsDouble();
    inputs.rRollerAppliedVolts = rRollerVoltsSignal.getValueAsDouble();
    inputs.rRollerSupplyAmps = rRollerCurrentSupplySignal.getValueAsDouble();
    inputs.rRollerStatorAmps = rRollerCurrentStatorSignal.getValueAsDouble();
    inputs.rRollerTempCelc = rRollerTemperatureSignal.getValueAsDouble();

    //L Roller
    inputs.lRollerConnected =
    BaseStatusSignal.refreshAll(
            lRollerVelocitySignal,
            lRollerVoltsSignal,
            lRollerCurrentStatorSignal,
            lRollerCurrentSupplySignal,
            lRollerTemperatureSignal)
        .isOK();

    inputs.lRollerVelocityRotPerSec = lRollerVelocitySignal.getValueAsDouble();
    inputs.lRollerAppliedVolts = lRollerVoltsSignal.getValueAsDouble();
    inputs.lRollerSupplyAmps = lRollerCurrentSupplySignal.getValueAsDouble();
    inputs.lRollerStatorAmps = lRollerCurrentStatorSignal.getValueAsDouble();
    inputs.lRollerTempCelc = lRollerTemperatureSignal.getValueAsDouble();

    //Front CANrange
    inputs.fCANrangeConnected =
    BaseStatusSignal.refreshAll(
           fCANrangeIsDectected)
        .isOK();
    inputs.fCANrangeRange = fCANrangeIsDectected.getValue();

    //Right CANrange
    inputs.rCANrangeConnected =
    BaseStatusSignal.refreshAll(
           rCANrangeIsDectected)
        .isOK();
    inputs.rCANrangeRange = rCANrangeIsDectected.getValue();

    //Left CANrange
    inputs.lCANrangeConnected =
    BaseStatusSignal.refreshAll(
        lCANrangeIsDectected)
        .isOK();
    inputs.lCANrangeRange = lCANrangeIsDectected.getValue();

    //Back CANrange
    inputs.bCANrangeConnected =
    BaseStatusSignal.refreshAll(
        bCANrangeIsDectected)
        .isOK();
    inputs.bCANrangeRange = bCANrangeIsDectected.getValue();
}

@Override
public void setIntakeDutyCycleOut(double output) {
    intakeTalon.setControl(dutyCycleOutControl.withOutput(output));}

@Override
public void setIntakeBrakeMode(boolean enabled) {
    intakeTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);}

@Override
public void stopIntake() {
    intakeTalon.setControl(new NeutralOut());}

@Override
public void setRRollerDutyCycleOut(double output) {
    rRollerTalon.setControl(dutyCycleOutControl.withOutput(output));}

@Override
public void setRRollerBrakeMode(boolean enabled) {
    rRollerTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);}

@Override
public void stopRRoller() {
    rRollerTalon.setControl(new NeutralOut());}

@Override
public void setLRollerDutyCycleOut(double output) {
    lRollerTalon.setControl(dutyCycleOutControl.withOutput(output));}

@Override
public void setLRollerBrakeMode(boolean enabled) {
    lRollerTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);}

@Override
public void stopLRoller() {
    lRollerTalon.setControl(new NeutralOut());}
}
