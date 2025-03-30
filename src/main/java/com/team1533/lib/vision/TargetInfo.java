// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.vision;

/**
 * A container class for Targets detected by the vision system, containing the location in
 * three-dimensional space.
 */
public class TargetInfo {
  protected double x = 1.0;
  protected double y;
  protected double z;
  protected double skew;
  protected int tagId;

  public TargetInfo(double y, double z, int tagId) {
    this.y = y;
    this.z = z;
    this.tagId = tagId;
  }

  public void setSkew(double skew) {
    this.skew = skew;
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public double getZ() {
    return z;
  }

  public int getTagId() {
    return tagId;
  }

  public double getSkew() {
    return skew;
  }
}
