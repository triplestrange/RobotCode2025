package com.team1533.frc2025.subsystems.wrist;

import com.team1533.frc2025.Constants.Gains;
import com.team1533.frc2025.Constants;

public class WristConstants {
    public static final int leaderTalonCanID = 231;
    public static final int wristEncoderCanID = 340;
    public static final String canBUS = "rio";

    public static final boolean leaderInverted = true;

    public static final double reduction = 1;
    public static final double SensorToMechanismRatio = 1.0;
    public static final double frictionVoltage = 0.25;

    public static final Gains gains = switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(0, 0.0, 0, 0, 0.0, 0.0, 0);
        default -> new Gains(0, 0, 0, 0, 0, 0, 0);
    };

}
