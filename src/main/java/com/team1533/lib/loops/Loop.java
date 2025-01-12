// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.loops;

/**
 * Interface for loops, which are routine that run periodically in the robot code (such as periodic
 * gyroscope calibration, etc.)
 */
public interface Loop {
  void onStart(double timestamp);

  void onLoop(double timestamp);

  void onStop(double timestamp);
}
