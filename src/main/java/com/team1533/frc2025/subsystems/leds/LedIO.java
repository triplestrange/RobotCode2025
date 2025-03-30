// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.leds;

/**
 * Interface for LED subsystem IO operations. This interface defines the methods for reading and
 * writing LED states as well as abstract methods for writing LED pixels to the LEDS on the robot.
 */
public interface LedIO {
  class LedInputs {}

  default void updateInputs(LedIO.LedInputs inputs) {}
  ;

  default void update(final LedIO.LedInputs inputs) {}
  ;

  default void writePixels(LedState state) {}
  ;

  default void writePixels(LedState[] states) {}
  ;
}
