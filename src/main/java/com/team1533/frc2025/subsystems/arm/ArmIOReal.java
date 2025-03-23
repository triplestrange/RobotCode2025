package com.team1533.frc2025.subsystems.arm;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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
import edu.wpi.first.wpilibj.Timer;

public class ArmIOReal implements ArmIO {
    protected final TalonFX leaderTalon;
    protected final TalonFX followerTalon;
    protected final CANcoder pivotEncoder;

    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final DutyCycleOut dutyCycleOutControl = new DutyCycleOut(0).withEnableFOC(true).withUpdateFreqHz(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0)
            .withUpdateFreqHz(0.0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0).withUpdateFreqHz(0.0);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withUpdateFreqHz(0.0);

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

    private final StatusSignal<Angle> encoderAbsolutePositionRotations;
    private final StatusSignal<Angle> encoderRelativePositionRotations;
    private final StatusSignal<Angle> fusedCanCoderRotations;

    private final StatusSignal<AngularVelocity> armVelocitySignal;
    private final StatusSignal<AngularAcceleration> armAccelerationSignal;

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    public ArmIOReal() {
        leaderTalon = new TalonFX(ArmConstants.leaderTalonCanID, ArmConstants.canBUS);
        followerTalon = new TalonFX(ArmConstants.followerTalonCanID, ArmConstants.canBUS);
        followerTalon.setControl(new Follower(ArmConstants.leaderTalonCanID, true));
        pivotEncoder = new CANcoder(ArmConstants.pivotEncoderCanID, ArmConstants.canBUS);

        // Leader motor configs
        config.Slot0.kP = ArmConstants.gains.kP();
        config.Slot0.kI = ArmConstants.gains.kI();
        config.Slot0.kD = ArmConstants.gains.kD();
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.TorqueCurrent.PeakForwardTorqueCurrent = ArmConstants.torqueCurrentLimit;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -ArmConstants.torqueCurrentLimit;
        config.MotorOutput.Inverted = ArmConstants.leaderInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.SensorToMechanismRatio = ArmConstants.SensorToMechanismRatio;
        config.Feedback.RotorToSensorRatio = ArmConstants.reduction;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = ArmConstants.statorCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ArmConstants.supplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLowerLimit = ArmConstants.supplyCurrentLowerLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = ArmConstants.supplyCurrentLowerLimitTime;

        config.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.motionMagicCruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = ArmConstants.motionMagicAcceleration;
        config.MotionMagic.MotionMagicJerk = ArmConstants.motionMagicJerk;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ArmConstants.forwardSoftLimitThreshold;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ArmConstants.reverseSoftLimitThreshold;

        // Cancoder configs
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = ArmConstants.absEncoderDiscontinuity;
        encoderConfig.MagnetSensor.MagnetOffset = ArmConstants.absEncoderOffset;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        // Base Status Signals
        fusedCanCoderRotations = leaderTalon.getPosition();

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

        encoderAbsolutePositionRotations = pivotEncoder.getAbsolutePosition();
        encoderRelativePositionRotations = pivotEncoder.getPosition();
        armVelocitySignal = leaderTalon.getVelocity();
        armAccelerationSignal = leaderTalon.getAcceleration();

        CTREUtil.applyConfiguration(leaderTalon, config);
        CTREUtil.applyConfiguration(pivotEncoder, encoderConfig);

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
                encoderAbsolutePositionRotations,
                encoderRelativePositionRotations,
                armAccelerationSignal);

        BaseStatusSignal.setUpdateFrequencyForAll(250, fusedCanCoderRotations, armVelocitySignal);

        // Optimize bus utilization
        leaderTalon.optimizeBusUtilization(0, 1.0);
        followerTalon.optimizeBusUtilization(0, 1.0);

        voltageOut.EnableFOC = true;
        dutyCycleOutControl.EnableFOC = true;

    }

    @Override
    public List<BaseStatusSignal> getStatusSignals() {
        // Only read position and velocity at 250 hz
        return Arrays.asList(fusedCanCoderRotations, armVelocitySignal);
    }

    @Override
    public void updateFastInputs(FastArmIOInputs inputs) {
        double position = BaseStatusSignal.getLatencyCompensatedValueAsDouble(fusedCanCoderRotations,
                armVelocitySignal);

        inputs.FusedCANcoderPositionRots = position;

    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.leaderConnected = BaseStatusSignal.refreshAll(leaderPositionSignal, leaderVelocitySignal,
                leaderVoltsSignal, leaderCurrentStatorSignal, leaderCurrentSupplySignal,
                armVelocitySignal, armAccelerationSignal, leaderTemperatureSignal, fusedCanCoderRotations).isOK();

        inputs.leaderVelocityRotPerSec = leaderVelocitySignal.getValueAsDouble();
        inputs.leaderAppliedVolts = leaderVoltsSignal.getValueAsDouble();
        inputs.leaderCurrentAmps = leaderCurrentSupplySignal.getValueAsDouble();
        inputs.leaderStatorAmps = leaderCurrentStatorSignal.getValueAsDouble();
        inputs.leaderTempCelc = leaderTemperatureSignal.getValueAsDouble();

        inputs.followerConnected = BaseStatusSignal.refreshAll(
                followerPositionSignal,
                followerVelocitySignal,
                followerVoltsSignal, followerCurrentStatorSignal, followerCurrentSupplySignal,
                followerTemperatureSignal).isOK();
        inputs.followerVelocityRotPerSec = followerVelocitySignal.getValueAsDouble();
        inputs.followerAppliedVolts = followerVoltsSignal.getValueAsDouble();
        inputs.followerCurrentAmps = followerCurrentSupplySignal.getValueAsDouble();
        inputs.followerStatorAmps = followerCurrentStatorSignal.getValueAsDouble();
        inputs.followerTempCelc = followerTemperatureSignal.getValueAsDouble();

        inputs.absoluteEncoderConnected = BaseStatusSignal
                .refreshAll(encoderAbsolutePositionRotations, encoderRelativePositionRotations).isOK();

        inputs.leaderRotPosition = leaderPositionSignal.getValueAsDouble();
        inputs.followerRotPosition = followerPositionSignal.getValueAsDouble();

        inputs.absoluteEncoderPositionRots = encoderAbsolutePositionRotations.getValueAsDouble();
        inputs.relativeEncoderPositionRots = encoderRelativePositionRotations.getValueAsDouble();
        inputs.armVelMetersPerSecond = armVelocitySignal.getValueAsDouble();
        inputs.armAccelMetersPerSecondPerSecond = armAccelerationSignal.getValueAsDouble();
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
    public void setPositionSetpoint(double positionRotations) {
        leaderTalon.setControl(positionTorqueCurrentFOC.withPosition(positionRotations));
    }

    @Override
    public void setPositionSetpoint(double positionRotations, double rotationsPerSec) {
        leaderTalon.setControl(
                positionTorqueCurrentFOC.withPosition(positionRotations).withVelocity(rotationsPerSec));
    }

    @Override
    public void setMotionMagicSetpoint(double positionRotations) {
        leaderTalon.setControl(motionMagicVoltage.withPosition(positionRotations));
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
