package com.team1533.frc2025.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1533.lib.util.CTREUtil;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX leaderTalon;
    private final TalonFX followerTalon;

    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0)
            .withUpdateFreqHz(0.0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0).withUpdateFreqHz(0.0);

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

    private final Timer timeSinceReset;

    public ElevatorIOReal() {
        timeSinceReset = new Timer();

        leaderTalon = new TalonFX(ElevatorConstants.leaderTalonCanID, "*");
        followerTalon = new TalonFX(ElevatorConstants.followerTalonCanID, "*");
        followerTalon.setControl(new Follower(ElevatorConstants.leaderTalonCanID, true));

        // Leader motor configs
        config.Slot0.kP = ElevatorConstants.gains.kP();
        config.Slot0.kI = ElevatorConstants.gains.kI();
        config.Slot0.kD = ElevatorConstants.gains.kD();
        config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        config.MotorOutput.Inverted = ElevatorConstants.leaderInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.reduction;

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
                100,
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
                elevatorPositionSignal,
                elevatorVelocitySignal,
                elevatorAccelerationSignal);

        // Optimize bus utilization
        leaderTalon.optimizeBusUtilization(0, 1.0);
        followerTalon.optimizeBusUtilization(0, 1.0);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leaderConnected = BaseStatusSignal.refreshAll(leaderPositionSignal, leaderVelocitySignal,
                leaderVoltsSignal, leaderCurrentStatorSignal, leaderCurrentSupplySignal, elevatorAccelerationSignal,
                elevatorPositionSignal, elevatorVelocitySignal, leaderTemperatureSignal).isOK();
        inputs.leaderVelocityRadPerSec = leaderVelocitySignal.getValueAsDouble();
        inputs.leaderAppliedVolts = leaderVoltsSignal.getValueAsDouble();
        inputs.leaderCurrentAmps = leaderCurrentSupplySignal.getValueAsDouble();
        inputs.leaderStatorAmps = leaderCurrentStatorSignal.getValueAsDouble();
        inputs.leaderTempCelc = leaderTemperatureSignal.getValueAsDouble();

        inputs.followerConnected = BaseStatusSignal.refreshAll(
                followerPositionSignal,
                followerVelocitySignal,
                followerVoltsSignal, followerCurrentStatorSignal, followerCurrentSupplySignal,
                followerTemperatureSignal).isOK();
        inputs.followerVelocityRadPerSec = followerVelocitySignal.getValueAsDouble();
        inputs.followerAppliedVolts = followerVoltsSignal.getValueAsDouble();
        inputs.followerCurrentAmps = followerCurrentSupplySignal.getValueAsDouble();
        inputs.followerStatorAmps = followerCurrentStatorSignal.getValueAsDouble();
        inputs.followerTempCelc = followerTemperatureSignal.getValueAsDouble();

        inputs.leaderRadPosition = leaderPositionSignal.getValueAsDouble();
        inputs.followerRadPosition = followerPositionSignal.getValueAsDouble();

        inputs.secondsSinceReset = timeSinceReset.get();
        inputs.elevatorPosMeters = elevatorPositionSignal.getValueAsDouble();
        inputs.elevatorVelMetersPerSecond = elevatorVelocitySignal.getValueAsDouble();
        inputs.elevatorAccelMetersPerSecondPerSecond = elevatorAccelerationSignal.getValueAsDouble();

    }

    @Override
    public void runVolts(double volts) {
        leaderTalon.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setPositionSetpoint(double positionMeters) {
        leaderTalon.setControl(positionTorqueCurrentFOC.withPosition(positionMeters));
    }

    @Override
    public void setPositionSetpoint(double positionMeters, double feedForward) {
        leaderTalon.setControl(positionTorqueCurrentFOC.withPosition(positionMeters).withFeedForward(feedForward));
    }

    @Override
    public void setCurrentSetpoint(double amps) {
        leaderTalon.setControl(currentControl.withOutput(amps));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        followerTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setPID(double p, double i, double d) {
        config.Slot0.kP = p;
        config.Slot0.kI = i;
        config.Slot0.kD = d;
        CTREUtil.applyConfiguration(leaderTalon, config);
    }

    @Override
    public void resetTimer() {
        timeSinceReset.reset();
    }

}
