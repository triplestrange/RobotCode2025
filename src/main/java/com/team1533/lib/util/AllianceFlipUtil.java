// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Utility functions for flipping from the blue to red alliance. */
public class AllianceFlipUtil {
  public static AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  /**
   * Flips an x coordinate to the correct side of the field based on the current
   * alliance color.
   */
  public static double applyX(double xCoordinate) {
    if (shouldFlip()) {
      return field.getFieldLength() - xCoordinate;
    } else {
      return xCoordinate;
    }
  }

  public static double applyY(double yCoordinate) {
    if (shouldFlip()) {
      return field.getFieldWidth() - yCoordinate;
    } else {
      return yCoordinate;
    }
  }

  /**
   * Flips a translation to the correct side of the field based on the current
   * alliance color.
   */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
    } else {
      return translation;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return new Rotation2d(-rotation.getCos(), -rotation.getSin());
    } else {
      return rotation;
    }
  }

  /**
   * Flips a pose to the correct side of the field based on the current alliance
   * color.
   */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    } else {
      return pose;
    }
  }

  public static Translation3d apply(Translation3d translation3d) {
    if (shouldFlip()) {
      return new Translation3d(
          applyX(translation3d.getX()), applyY(translation3d.getY()), translation3d.getZ());
    } else {
      return translation3d;
    }
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }
}
