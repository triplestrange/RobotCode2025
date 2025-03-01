package com.team1533.frc2025.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team1533.frc2025.Constants;
import com.team1533.frc2025.Constants.Gains;

public class ElevatorConstants {
    public static final int leaderTalonCanID = 23;
    public static final int followerTalonCanID = 24;
    public static final String canBUS = "rio";

    public static final boolean leaderInverted = true;

    public static final double drumCircumferenceMeters = 24 * 0.005;
    public static final double reduction = 4.0 / drumCircumferenceMeters;
    public static final double frictionVoltage = 0.25;
    public static final double kElevatorPositioningToleranceInches = 0.1;

    public static final double torqueCurrentLimit = 80;
    public static final double statorCurrentLimit = 80;
    public static final double supplyCurrentLimit = 60;
    public static final double supplyCurrentLowerLimit = 40;
    public static final double supplyCurrentLowerLimitTime = 1;

    public static final double motionMagicCruiseVelocity = 2.75;
    public static final double motionMagicAcceleration = 10;
    public static final double motionMagicJerk = 5;

    
    public static final double reverseSoftLimitThreshold = 0;
    public static final double forwardSoftLimitThreshold = 0;


    public static final Gains gains = switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(750, 0.0, 75, 22, 0.0, 0.0, 0);
        default -> new Gains(0, 0, 0, 0, 0, 0, 0);
    };

}
