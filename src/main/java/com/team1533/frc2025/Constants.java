package com.team1533.frc2025;

import com.team1533.lib.util.Alert;
import com.team1533.lib.util.Alert.AlertType;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {

    public static final double loopPeriodSecs = 0.02;
    public static final double kSimDt = 0.001;
    private static RobotType robotType = RobotType.SIMBOT;
    public static final boolean tuningMode = false;

    public static RobotType getRobot() {
        if (!disableHAL && RobotBase.isReal() && robotType == RobotType.SIMBOT) {
            new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
                    .set(true);
            robotType = RobotType.COMPBOT;
        }
        return robotType;
    }

    public enum RobotType {
        SIMBOT,
        COMPBOT,
        REPLAY
    }

    public static boolean disableHAL = false;

    public static void disableHAL() {
        disableHAL = true;
    }

    /** Checks whether the correct robot is selected when deploying. */
    public static void main(String... args) {
        if (robotType == RobotType.SIMBOT) {
            System.err.println("Cannot deploy, invalid robot selected: " + robotType);
            System.exit(1);
        }
    }

    public record Gains(
            double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {
    }
}
