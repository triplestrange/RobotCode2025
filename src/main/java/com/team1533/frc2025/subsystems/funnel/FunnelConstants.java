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

    public static final double reduction = 64. / 11.;

    public static final double frictionVoltage = 0.25;

    public static final double torqueCurrentLimit = 80;
    public static final double statorCurrentLimit = 80;
    public static final double supplyCurrentLimit = 60;
    public static final double supplyCurrentLowerLimit = 40;
    public static final double supplyCurrentLowerLimitTime = 1;

    public static final double motionMagicCruiseVelocity = 0;
    public static final double motionMagicAcceleration = 0;

    public static final double reverseSoftLimitThreshold = 0;
    public static final double forwardSoftLimitThreshold = 0.712;

    public static final double toleranceRotations = 0.05;

    public static final Gains gains = switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(60, 0.0, 1, 0, 0.0, 0.0, 0);
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

        config.unitToRotorRatio = 1./reduction;

        config.talonCANID = funnelTalon;


        config.fxConfig.MotionMagic.MotionMagicExpo_kA = motionMagicAcceleration;
        config.fxConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicCruiseVelocity;

        config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimitThreshold;
        config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimitThreshold;

        config.fxConfig.Slot0.kP = gains.kP();
        config.fxConfig.Slot0.kI = gains.kI();
        config.fxConfig.Slot0.kD = gains.kD();
        config.fxConfig.Slot0.kG = gains.ffkG();
        config.fxConfig.Slot0.kS= gains.ffkS();
        config.fxConfig.Slot0.kV = gains.ffkV();
        config.fxConfig.Slot0.kA = gains.ffkA();



    }
}
