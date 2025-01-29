// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation pinHoleTargetObservation = new TargetObservation(0, 0, 0,
        Rotation2d.kZero,
        Rotation2d.kZero);
    public PoseObservation[] multitagPoseObservations = new PoseObservation[0];

    public int[] tagIds = new int[0];
  }

  /**
   * Represents the angle to a simple target, not used for pose estimation.
   *
   * timestamp
   * z
   * tagID
   * tx
   * ty
   * yaw
   */
  public static record TargetObservation(double timestamp, double z, int tagID, Rotation2d tx, Rotation2d ty) {
  }

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double z,
      double ambiguity,
      int tagCount,
      double averageTagDistance) {
  }

  public default void updateInputs(VisionIOInputs inputs) {
  }
}
