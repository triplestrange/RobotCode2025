package com.team1533.frc2025.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1533.frc2025.Constants;
import com.team1533.frc2025.Constants.Gains;
import com.team1533.lib.drivers.CANDeviceId;
import com.team1533.lib.subsystems.ServoMotorSubsystemConfig;

public class IntakeConstants {
    public static final CANDeviceId intakeTalon = new CANDeviceId(26, "rio");
    public static final CANDeviceId intakeCC = new CANDeviceId(342, "rio");

    public static final boolean leaderInverted = true;

    public static final double reduction = 1.;
    public static final double rotorToSensorRatio = 1.;
    public static final double sensorToMechanismRatio = 1.;

    public static final Gains gains = switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(0, 0.0, 0, 0, 0.0, 0.0, 0);
        default -> new Gains(0, 0, 0, 0, 0, 0, 0);
    };

    public static final ServoMotorSubsystemConfig config = new ServoMotorSubsystemConfig();
    static {
        // Feedback Configs
        config.fxConfig.Feedback.RotorToSensorRatio = rotorToSensorRatio;
        config.fxConfig.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;

        config.fxConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        config.fxConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.fxConfig.CurrentLimits.StatorCurrentLimit = 1;
        config.fxConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        config.fxConfig.CurrentLimits.SupplyCurrentLimit = 1;
        config.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.fxConfig.CurrentLimits.SupplyCurrentLowerLimit = 1;
        config.fxConfig.CurrentLimits.SupplyCurrentLowerTime = 1;

        config.name = "intake";

        config.kMinPositionUnits = 0;
        config.kMaxPositionUnits = 0;
        config.momentOfInertia = 1;

        config.talonCANID = intakeTalon;
        config.unitToRotorRatio = reduction;

    }
}
