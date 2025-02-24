package com.team1533.frc2025.subsystems.wrist;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1533.frc2025.Constants;
import com.team1533.frc2025.Constants.Gains;
import com.team1533.lib.drivers.CANDeviceId;
import com.team1533.lib.subsystems.ServoMotorSubsystemConfig;

public class WristConstants {
    public static final CANDeviceId wristTalon = new CANDeviceId(04334, "rio");
    public static final CANDeviceId wristCC = new CANDeviceId(342, "rio");

    public static final boolean leaderInverted = true;

    public static final double reduction = 1.;
    public static final double rotorToSensorRatio = 1.;
    public static final double sensorToMechanismRatio = 1.;

    public static final Gains gains = switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(750, 0.0, 75, 0, 0.0, 0.0, 0);
        default -> new Gains(0, 0, 0, 0, 0, 0, 0);
    };

    public static final ServoMotorSubsystemConfig config = new ServoMotorSubsystemConfig();
    static {
        // Feedback Configs
        config.fxConfig.Feedback.FeedbackRemoteSensorID = wristCC.getDeviceNumber();
        config.fxConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.fxConfig.Feedback.RotorToSensorRatio = rotorToSensorRatio;
        config.fxConfig.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;

        config.fxConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        config.fxConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        config.fxConfig.Slot0.kP = 0.0;
        config.fxConfig.Slot0.kI = 0.0;
        config.fxConfig.Slot0.kD = 0.0;
        config.fxConfig.Slot0.kA = 0.0;
        config.fxConfig.Slot0.kG = 0.0;
        config.fxConfig.Slot0.kS = 0.0;
        config.fxConfig.Slot0.kV = 0.0;

        config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.fxConfig.MotionMagic.MotionMagicJerk = 1;
        config.fxConfig.MotionMagic.MotionMagicAcceleration = 1;
        config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 1;

        config.fxConfig.MotionMagic.MotionMagicExpo_kA = 1;
        config.fxConfig.MotionMagic.MotionMagicExpo_kV = 1;

        config.fxConfig.CurrentLimits.StatorCurrentLimit = 1;
        config.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        config.fxConfig.CurrentLimits.SupplyCurrentLimit = 1;
        config.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.fxConfig.CurrentLimits.SupplyCurrentLowerLimit = 1;
        config.fxConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

        config.fxConfig.ClosedLoopGeneral.ContinuousWrap = true;

        config.name = "Wrist";

        config.kMinPositionUnits = 0;
        config.kMaxPositionUnits = 0;
        config.momentOfInertia = 1;

        config.talonCANID = wristTalon;
        config.unitToRotorRatio = reduction;

    }
}
