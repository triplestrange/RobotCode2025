// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;

public class AbsoluteEncoder extends AnalogInput {
  private double angleOffset;
  private boolean reversed = false;

  /**
   * @param channel analog channel of the encoder
   * @param angleOffset offset acquired by the getCalibration method
   */
  public AbsoluteEncoder(int channel, double angleOffset) {
    this(channel, angleOffset, false);
  }

  /**
   * @param channel analog channel of the encoder
   * @param angleOffset offset acquired by the getCalibration method
   * @param reversed boolean indicating the encoder's angle needs to be reversed
   */
  public AbsoluteEncoder(int channel, double angleOffset, boolean reversed) {
    super(channel);
    this.angleOffset = Math.toRadians(angleOffset);
    this.reversed = reversed;
  }

  /**
   * Get the current angle of the encoder in radians between 0 and 2pi.
   *
   * @return current angle in radians
   */
  public double getAngle() {
    double angle = ((getVoltage() - .2) / 4.6) * (2 * Math.PI);
    if (reversed) angle *= -1;
    return normalizeAngle(angle - angleOffset);
  }

  public boolean isConnected() {
    return RobotController.getVoltage5V() > 0;
  }

  /**
   * Convert an angle to its equivalent between 0 and 2pi
   *
   * @param angle angle in radians
   * @return normalized angle between 0 and 2pi
   */
  private double normalizeAngle(double angle) {
    angle %= (2 * Math.PI);
    if (angle < 0) angle += 2 * Math.PI;
    return angle;
  }

  /**
   * When the encoder is pointing at the desired angle, call this method to get the angleOffset such
   * that the current angle will be zero.
   *
   * @return angle offset to calibrate encoder
   */
  public double getCalibration() {
    return normalizeAngle(getAngle() + angleOffset);
  }

  /** Angle returned to a PID controller. */
  public double pidGet() {
    return getAngle();
  }
}
