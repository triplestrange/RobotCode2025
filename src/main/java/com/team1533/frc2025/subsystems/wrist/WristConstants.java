package com.team1533.frc2025.subsystems.wrist;

import com.team1533.frc2025.Constants.Gains;
import com.team1533.frc2025.Constants;

public class WristConstants {
    public static final int leaderTalonCanID = 231;
    public static final int wristEncoderCanID = 340;
    public static final int pivotEncoderCanID =  23247;

    public static final String canBUS = "rio";

    public static final boolean leaderInverted = true;

    public static final double reduction = 1;
    public static final double SensorToMechanismRatio = 1.0;
    public static final double frictionVoltage = 0.25;

    public static final double torqueCurrentLimit = 80;
    public static final double statorCurrentLimit = 80;
    public static final double supplyCurrentLimit = 40;
    public static final double supplyCurrentLowerLimit = 40;
    public static final double supplyCurrentLowerLimitTime = 1;

    public static final double motionMagicCruiseVelocity = 1;
    public static final double motionMagicAcceleration = 1;
    public static final double motionMagicJerk = 1;

    
    public static final double absEncoderOffset = -0.177246;
    public static final double absEncoderDiscontinuity = 0.8;

    public static final double reverseSoftLimitThreshold = 0;
    public static final double forwardSoftLimitThreshold = 0;

    public static final Gains gains = switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(0, 0.0, 0, 0, 0.0, 0.0, 0);
        default -> new Gains(0, 0, 0, 0, 0, 0, 0);
    };

}
