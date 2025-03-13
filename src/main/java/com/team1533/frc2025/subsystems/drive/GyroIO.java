// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public Rotation2d rollPosition = new Rotation2d();
    public Rotation2d pitchPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
    public double rollVelocityRadPerSec = 0.0;
    public double pitchVelocityRadPerSec = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    public Rotation2d[] odometryRollPositions = new Rotation2d[] {};
    public Rotation2d[] odometryPitchPositions = new Rotation2d[] {};

    public double[] odometryYawVelocityRadPerSecs = new double[] {};
    public double[] odometryRollVelocityRadPerSecs = new double[] {};
    public double[] odometryPitchVelocityRadPerSecs = new double[] {};

    public double accelX = 0.0;
    public double accelY = 0.0;

    public double[] odometryAccelXs = new double[] {};
    public double[] odometryAccelYs = new double[] {};

  }

  public default void updateInputs(GyroIOInputs inputs) {
  }
}
