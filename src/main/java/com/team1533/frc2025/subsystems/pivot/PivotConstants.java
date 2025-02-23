package com.team1533.frc2025.subsystems.pivot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team1533.frc2025.Constants;

public class PivotConstants {
    public static final int leaderTalonCanID = 21;
    public static final int followerTalonCanID = 22;
    public static final String canBUS = "rio";

    public static final boolean leaderInverted = true;

    public static final double reduction = 240.0;
    public static final double frictionVoltage = 0.25;

    public static final Gains gains = switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(750, 0.0, 75, 0, 0.0, 0.0, 0);
        default -> new Gains(0, 0, 0, 0, 0, 0, 0);
    };

    public record Gains(
            double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {
    }

}
