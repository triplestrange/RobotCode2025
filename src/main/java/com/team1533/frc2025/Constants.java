// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025;

import com.team1533.lib.util.Alert;
import com.team1533.lib.util.Alert.AlertType;
import com.team1533.lib.util.AllianceFlipUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

public class Constants {

  public static final double kRealDt = 0.02;
  public static final double kSimDt = 0.005;

  private static RobotType robotType = RobotType.SIMBOT;
  public static final boolean tuningMode = false;

  // AprilTag layout
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

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
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}

  public record SuperStructureState(
      double armGoalRots, double elevGoalMeters, double wristGoalRots, double funnelGoalRots) {}

  public static final Pose2d REEF_OFFSET =
      new Pose2d(0, Units.inchesToMeters(6.5), Rotation2d.kZero);

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
      return AllianceFlipUtil.apply(
          new Pose2d(
              pose.getTranslation()
                  .plus(
                      new Translation2d(Units.inchesToMeters(18.375), 0)
                          .rotateBy(pose.getRotation())),
              pose.getRotation().plus(Rotation2d.k180deg)));
    }

    public Pose2d getPose2dReef(boolean isleft) {
      if (isleft)
        return AllianceFlipUtil.apply(
            new Pose2d(
                pose.getTranslation()
                    .plus(
                        REEF_OFFSET
                            .getTranslation()
                            .rotateBy(pose.getRotation().plus(Rotation2d.k180deg)))
                    .plus(
                        new Translation2d(Units.inchesToMeters(18.375), 0)
                            .rotateBy(pose.getRotation())),
                pose.getRotation().plus(Rotation2d.k180deg)));
      return AllianceFlipUtil.apply(
          new Pose2d(
              pose.getTranslation()
                  .minus(
                      REEF_OFFSET
                          .getTranslation()
                          .rotateBy(pose.getRotation().plus(Rotation2d.k180deg)))
                  .plus(
                      new Translation2d(Units.inchesToMeters(18.375), 0)
                          .rotateBy(pose.getRotation())),
              pose.getRotation().plus(Rotation2d.k180deg)));
    }
  }

  @RequiredArgsConstructor
  public enum SuperStructureStates {
    STOW(new SuperStructureState(0, 0, 0, 0)),
    SAFE(new SuperStructureState(0.21, Units.inchesToMeters(1.5), 0.22, .25)),
    L1(new SuperStructureState(0, 0, 0, 0)),
    L2(new SuperStructureState(0.1, 0.086995, 0.145, 0)),
    L3(new SuperStructureState(0.16, 0.387106, 0.22, 0)),
    L4(new SuperStructureState(0.205, 1.07, 0.337, 0)),
    BARGE(new SuperStructureState(0.24, 1.07, 0.3, 0)),
    PROCESSOR(new SuperStructureState(0.08, 0.1, 0.5, 0)),
    FEEDER(new SuperStructureState(0.15, 0.043, 0.71, 0)),
    GROUND_ALGAE(new SuperStructureState(0, 0, 0, 0)),
    ALGAELOWER(new SuperStructureState(0.155, 0.055, 0.455, 0)),
    ALGAEUPPER(new SuperStructureState(0.18, 0.48, 0.52, 0)),
    CLIMB_PREP(new SuperStructureState(0.25, 0, 0.5, 0)),
    CLIMB(new SuperStructureState(0, 0, 0.4, 0)),
    REEF_CLEARANCE(new SuperStructureState(0, 0, 0.5, 0)),
    ELEVATOR_CLEARANCE(new SuperStructureState(0.175, Units.inchesToMeters(1.5), 0.71, 0)),
    WRIST_CLEARANCE(new SuperStructureState(0, 0.043, 0, 0.04));

    @Getter private final SuperStructureState state;
  }
}
