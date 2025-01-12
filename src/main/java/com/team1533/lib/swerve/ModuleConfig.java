// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.swerve;

public class ModuleConfig {
  private int driveMotorChannel;
  private int turningMotorChannel;
  private int absoluteEncoderChannel;
  private boolean turningEncoderReversed;
  private double angleOffset;

  public ModuleConfig(
      int driveMotorChannel,
      int turningMotorChannel,
      int absoluteEncoderChannel,
      boolean turningEncoderReversed,
      double angleOffset) {
    this.driveMotorChannel = driveMotorChannel;
    this.turningMotorChannel = turningMotorChannel;
    this.absoluteEncoderChannel = absoluteEncoderChannel;
    this.turningEncoderReversed = turningEncoderReversed;
    this.angleOffset = angleOffset;
  }

  public int getDriveMotorChannel() {
    return driveMotorChannel;
  }

  public int getTurningMotorChannel() {
    return turningMotorChannel;
  }

  public int getAbsoluteEncoderChannel() {
    return absoluteEncoderChannel;
  }

  public boolean getTurningEncoderReversed() {
    return turningEncoderReversed;
  }

  public double getAngleOffset() {
    return angleOffset;
  }
}
