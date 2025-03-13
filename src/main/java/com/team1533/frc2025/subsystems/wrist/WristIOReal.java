package com.team1533.frc2025.subsystems.wrist;

import java.rmi.ConnectIOException;
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
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.team1533.frc2025.Constants.Gains;
import com.team1533.lib.util.CTREUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

public class WristIOReal implements WristIO {
    protected final TalonFX leaderTalon;
    protected final CANcoder wristEncoder;

    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true).withUpdateFreqHz(0.0);
    private final DutyCycleOut dutyCycleOutControl = new DutyCycleOut(0).withEnableFOC(true).withUpdateFreqHz(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0)
            .withUpdateFreqHz(0.0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0).withUpdateFreqHz(0.0);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withEnableFOC(true)
            .withUpdateFreqHz(0.0);

    private final StatusSignal<Angle> leaderPositionSignal;
    private final StatusSignal<AngularVelocity> leaderVelocitySignal;
    private final StatusSignal<Voltage> leaderVoltsSignal;
    private final StatusSignal<Current> leaderCurrentStatorSignal;
    private final StatusSignal<Current> leaderCurrentSupplySignal;
    private final StatusSignal<Temperature> leaderTemperatureSignal;

    private final StatusSignal<Angle> encoderAbsolutePositionRotations;
    private final StatusSignal<Angle> encoderRelativePositionRotations;

    private final StatusSignal<Angle> fusedCancoderSignal;

    private final StatusSignal<AngularVelocity> wristVelocitySignal;
    private final StatusSignal<AngularAcceleration> wristAccelerationSignal;

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    public WristIOReal() {
        leaderTalon = new TalonFX(WristConstants.leaderTalonCanID, WristConstants.canBUS);
        wristEncoder = new CANcoder(WristConstants.wristEncoderCanID, WristConstants.canBUS);

        // Leader motor configs
        config.Slot0.kP = WristConstants.gains.kP();
        config.Slot0.kI = WristConstants.gains.kI();
        config.Slot0.kD = WristConstants.gains.kD();
        config.Slot0.kA = WristConstants.gains.ffkA();
        config.Slot0.kG = WristConstants.gains.ffkG();
        config.Slot0.kS = WristConstants.gains.ffkS();
        config.Slot0.kV = WristConstants.gains.ffkV();

        config.TorqueCurrent.PeakForwardTorqueCurrent = WristConstants.torqueCurrentLimit;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -WristConstants.torqueCurrentLimit;
        config.MotorOutput.Inverted = WristConstants.leaderInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = WristConstants.reduction;

        config.Feedback.FeedbackRemoteSensorID = wristEncoder.getDeviceID();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.SensorToMechanismRatio = WristConstants.SensorToMechanismRatio;
        config.Feedback.RotorToSensorRatio = WristConstants.reduction;

        config.CurrentLimits.StatorCurrentLimit = WristConstants.statorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = WristConstants.supplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLowerLimit = WristConstants.supplyCurrentLowerLimit;
        config.CurrentLimits.SupplyCurrentLowerTime = WristConstants.supplyCurrentLowerLimitTime;

        config.MotionMagic.MotionMagicCruiseVelocity = WristConstants.motionMagicCruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = WristConstants.motionMagicAcceleration;
        config.MotionMagic.MotionMagicJerk = WristConstants.motionMagicJerk;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WristConstants.forwardSoftLimitThreshold;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = WristConstants.reverseSoftLimitThreshold;

        // Cancoder configs
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = WristConstants.absEncoderDiscontinuity;
        encoderConfig.MagnetSensor.MagnetOffset = WristConstants.absEncoderOffset;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        // Base Status Signals
        leaderPositionSignal = leaderTalon.getRotorPosition();
        leaderVelocitySignal = leaderTalon.getRotorVelocity();
        leaderVoltsSignal = leaderTalon.getMotorVoltage();
        leaderCurrentStatorSignal = leaderTalon.getStatorCurrent();
        leaderCurrentSupplySignal = leaderTalon.getSupplyCurrent();
        leaderTemperatureSignal = leaderTalon.getDeviceTemp();

        encoderAbsolutePositionRotations = wristEncoder.getAbsolutePosition();
        encoderRelativePositionRotations = wristEncoder.getPosition();
        wristVelocitySignal = leaderTalon.getVelocity();
        wristAccelerationSignal = leaderTalon.getAcceleration();

        fusedCancoderSignal = leaderTalon.getPosition();

        CTREUtil.applyConfiguration(leaderTalon, config);
        CTREUtil.applyConfiguration(wristEncoder, encoderConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                leaderPositionSignal,
                leaderVelocitySignal,
                leaderVoltsSignal,
                leaderCurrentStatorSignal,
                leaderCurrentSupplySignal,
                leaderTemperatureSignal,
                encoderAbsolutePositionRotations,
                encoderRelativePositionRotations,
                wristAccelerationSignal);
        BaseStatusSignal.setUpdateFrequencyForAll(250, fusedCancoderSignal, wristVelocitySignal);

        // Optimize bus utilization
        leaderTalon.optimizeBusUtilization(0, 1.0);
        wristEncoder.optimizeBusUtilization(0, 1.0);

        voltageOut.EnableFOC = true;
        dutyCycleOutControl.EnableFOC = true;
        motionMagicVoltage.EnableFOC = true;

    }

    @Override
    public List<BaseStatusSignal> getStatusSignals() {
        // Only read position and velocity at 250 hz
        return Arrays.asList(fusedCancoderSignal, wristVelocitySignal);
    }

    @Override
    public void updateFastInputs(FastWristIOInputs inputs) {
        double position = BaseStatusSignal.getLatencyCompensatedValueAsDouble(fusedCancoderSignal, wristVelocitySignal);
        inputs.FusedCANcoderPositionRots = position;
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.leaderConnected = BaseStatusSignal.refreshAll(leaderPositionSignal, leaderVelocitySignal,
                leaderVoltsSignal, leaderCurrentStatorSignal, leaderCurrentSupplySignal,
                wristVelocitySignal, wristAccelerationSignal, leaderTemperatureSignal, fusedCancoderSignal).isOK();
        inputs.leaderVelocityRotPerSec = leaderVelocitySignal.getValueAsDouble();
        inputs.leaderAppliedVolts = leaderVoltsSignal.getValueAsDouble();
        inputs.leaderCurrentAmps = leaderCurrentSupplySignal.getValueAsDouble();
        inputs.leaderStatorAmps = leaderCurrentStatorSignal.getValueAsDouble();
        inputs.leaderTempCelc = leaderTemperatureSignal.getValueAsDouble();

        inputs.absoluteEncoderConnected = BaseStatusSignal
                .refreshAll(encoderAbsolutePositionRotations, encoderRelativePositionRotations).isOK();

        inputs.leaderRotPosition = leaderPositionSignal.getValueAsDouble();

        inputs.absoluteEncoderPositionRots = encoderAbsolutePositionRotations.getValueAsDouble();
        inputs.relativeEncoderPositionRots = encoderRelativePositionRotations.getValueAsDouble();
        inputs.wristVelRotsPerSecond = wristVelocitySignal.getValueAsDouble();
        inputs.wristAccelRotsPerSecondPerSecond = wristAccelerationSignal.getValueAsDouble();
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
