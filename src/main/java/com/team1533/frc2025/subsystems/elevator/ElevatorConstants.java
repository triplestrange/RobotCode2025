package com.team1533.frc2025.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team1533.frc2025.Constants;

public class ElevatorConstants {
    public static final int leaderTalonCanID = 10;
    public static final int followerTalonCanID = 11;
    public static final String canBUS = "";

    public static final boolean leaderInverted = false;

    public static final double drumCircumferenceMeters = 24 * 0.005;
    public static final double reduction = 4.0 / drumCircumferenceMeters;

    public static final Gains gains = switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(7000.0, 0.0, 250.0, 8.4, 0.0, 0.0, 22.9);
    };

    public record Gains(
            double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {
    }

}
