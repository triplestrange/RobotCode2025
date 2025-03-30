// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.time;

import edu.wpi.first.wpilibj.Timer;

public class RobotTime {
  public static double getTimestampSeconds() {
    double seconds = Timer.getFPGATimestamp();
    return (double) seconds; // * 1.0E-6;
  }
}
