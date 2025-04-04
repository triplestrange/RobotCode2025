package com.team1533.frc2025.subsystems.funnel;

import com.team1533.frc2025.Constants.Gains;
import com.team1533.frc2025.Constants;

public class FunnelConstants {
    public static final int leaderTalonCanID = 30;

    public static final String canBUS = "rio";

    public static final boolean leaderInverted = false;

    public static final double reduction = (64./11.);
    public static final double SensorToMechanismRatio = 1.0;
    public static final double frictionVoltage = 0.25;

    public static final double torqueCurrentLimit = 80;
    public static final double statorCurrentLimit = 80;
    public static final double supplyCurrentLimit = 60;
    public static final double supplyCurrentLowerLimit = 40;
    public static final double supplyCurrentLowerLimitTime = 1;

    public static final double motionMagicCruiseVelocity = 3.9;
    public static final double motionMagicAcceleration = 2;
    public static final double motionMagicJerk = 10;

    
    // public static final double absEncoderOffset = 0.875 - 0.005859375;
    // public static final double absEncoderDiscontinuity = 0.9;

    public static final double reverseSoftLimitThreshold = 0;
    public static final double forwardSoftLimitThreshold = 0.712;

    public static final Gains gains = switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(60, 0.0, 1, 0, 0.0, 0.0, 0);
        default -> new Gains(0, 0, 0, 0, 0, 0, 0);
    };

}
