package com.team1533.frc2025;

import java.time.LocalDate;

import com.team1533.lib.util.Alert;
import com.team1533.lib.util.AllianceFlipUtil;
import com.team1533.lib.util.Alert.AlertType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import lombok.RequiredArgsConstructor;

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

    @RequiredArgsConstructor
    public enum FieldLocation {
        FEEDER_LEFT_LEFT(new Pose2d()),
        FEEDER_LEFT_RIGHT(new Pose2d()),
        FEEDER_RIGHT_RIGHT(new Pose2d()),
        FEEDER_RIGHT_LEFT(new Pose2d()),
        REEF_17(new Pose2d()),
        REEF_18(new Pose2d()),
        REEF_19(new Pose2d()),
        REEF_20(new Pose2d()),
        REEF_21(new Pose2d()),
        REEF_22(new Pose2d()),
        BARGE_LEFT(new Pose2d()),
        BARGE_MIDDLE(new Pose2d()),
        BARGE_RIGHT(new Pose2d()),
        REEF_OFFSET(new Pose2d());

        private final Pose2d pose;

        public Pose2d getPose2dFlipped() {
            return AllianceFlipUtil.apply(pose);
        }

        public Pose2d getPose2dReef(FieldLocation location, boolean isleft) {
            if (isleft)
                return new Pose2d(
                        location.getPose2dFlipped().getTranslation()
                                .plus(REEF_OFFSET.getPose2dFlipped().getTranslation()
                                        .rotateBy(location.getPose2dFlipped().getRotation())),
                        location.getPose2dFlipped().getRotation());
            return new Pose2d(
                    location.getPose2dFlipped().getTranslation()
                            .minus(REEF_OFFSET.getPose2dFlipped().getTranslation()
                                    .rotateBy(location.getPose2dFlipped().getRotation())),
                    location.getPose2dFlipped().getRotation());
        }
    }
}
