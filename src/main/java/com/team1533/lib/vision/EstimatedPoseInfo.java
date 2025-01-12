// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class EstimatedPoseInfo {
  private Pose2d pos;
  private double timestampSeconds;
  private double numOfTags;
  private int index;

  public EstimatedPoseInfo(Pose2d pos, double timestampSeconds, double numOfTags, int index) {
    this.pos = pos;
    this.timestampSeconds = timestampSeconds;
    this.numOfTags = numOfTags;
    this.index = index;
  }

  public Pose2d getPose2d() {
    return pos;
  }

  public double getTimestampSeconds() {
    return timestampSeconds;
  }

  public double getNumOfTags() {
    return numOfTags;
  }

  public int getIndex() {
    return index;
  }
}
