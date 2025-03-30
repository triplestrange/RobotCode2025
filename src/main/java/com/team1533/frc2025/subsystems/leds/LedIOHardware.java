// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.leds;

import com.ctre.phoenix.led.CANdle; // Import the CANdle class from CTRE Phoenix library

/**
 * The {@code LedIOHardware} class implements the {@link LedIO} interface to control LEDs using the
 * CANdle hardware device. It provides methods to set the color of LEDs based on {@link LedState}
 * objects and arrays. The class is also responsible for initializing the CANdle device and managing
 * brightness and color data sent to the LEDs.
 */
public class LedIOHardware implements LedIO {
  private final CANdle candle;

  public LedIOHardware() {
    candle = new CANdle(LedConstants.kCANdleId.getDeviceNumber(), LedConstants.kCANdleId.getBus());
    candle.configAllSettings(LedConstants.config);
  }

  @Override
  public void writePixels(LedState state) {
    if (state == null) state = LedState.kOff;
    candle.setLEDs(state.red, state.green, state.blue);
  }

  @Override
  public void writePixels(LedState[] pixels) {
    // do not write empty data
    if (pixels == null || pixels.length == 0) {
      return;
    }

    LedState run = pixels[0];
    int runStart = 0;
    for (int i = 0; i < pixels.length; i++) {
      if (pixels[i] == null) pixels[i] = LedState.kOff;
      if (!run.equals(pixels[i])) {
        candle.setLEDs(run.red, run.green, run.blue, 255, runStart, i - runStart);
        runStart = i;
        run = pixels[i];
      }
    }

    candle.setLEDs(run.red, run.green, run.blue, 255, runStart, pixels.length - runStart);
  }
}
