package com.team1533.frc2025;

import com.team1533.lib.util.Alert;
import com.team1533.lib.util.AllianceFlipUtil;
import com.team1533.lib.util.Alert.AlertType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import lombok.RequiredArgsConstructor;

public class Constants {

    public static final double loopPeriodSecs = 0.02;
    public static final double kSimDt = 0.001;
    private static RobotType robotType = RobotType.REPLAY;
    public static final boolean tuningMode = false;

    // AprilTag layout
    public static final AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

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

    public static final Pose2d REEF_OFFSET = new Pose2d(0, Units.inchesToMeters(6.5), Rotation2d.kZero);


    @RequiredArgsConstructor
    public enum ReefLocations {
        REEF_17(aprilTagLayout.getTagPose(17).get().toPose2d()),
        REEF_18(aprilTagLayout.getTagPose(18).get().toPose2d()),
        REEF_19(aprilTagLayout.getTagPose(19).get().toPose2d()),
        REEF_20(aprilTagLayout.getTagPose(20).get().toPose2d()),
        REEF_21(aprilTagLayout.getTagPose(21).get().toPose2d()),
        REEF_22(aprilTagLayout.getTagPose(22).get().toPose2d());

        private final Pose2d pose;

        public Pose2d getPose2dFlipped() {
            return AllianceFlipUtil.apply(new Pose2d(pose.getTranslation().plus(new Translation2d(Units.inchesToMeters(18.375), 0).rotateBy(pose.getRotation())), pose.getRotation().plus(Rotation2d.k180deg)));
        }

        public Pose2d getPose2dReef(boolean isleft) {
            if (isleft)
                return AllianceFlipUtil.apply(new Pose2d(
                        pose.getTranslation()
                                .plus(REEF_OFFSET.getTranslation().rotateBy(pose.getRotation().plus(Rotation2d.k180deg)))
                                .plus(new Translation2d(Units.inchesToMeters(18.375), 0).rotateBy(pose.getRotation())),
                        pose.getRotation().plus(Rotation2d.k180deg)));
                        return AllianceFlipUtil.apply(new Pose2d(
                            pose.getTranslation()
                                    .minus(REEF_OFFSET.getTranslation().rotateBy(pose.getRotation().plus(Rotation2d.k180deg)))
                                    .plus(new Translation2d(Units.inchesToMeters(18.375), 0).rotateBy(pose.getRotation())),
                            pose.getRotation().plus(Rotation2d.k180deg)));
        }
    }
}
