// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.leds;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.team1533.frc2025.generated.TunerConstants;
import com.team1533.lib.drivers.CANDeviceId;

public class LedConstants {

  public static final CANDeviceId kCANdleId =
      new CANDeviceId(29, TunerConstants.DrivetrainConstants.CANBusName);
  public static final int kNonCandleLEDCount = 5;
  public static final int kCandleLEDCount = 8;
  public static final int kMaxLEDCount = kNonCandleLEDCount + kCandleLEDCount;

  public static final CANdleConfiguration config = new CANdleConfiguration();

  static {
    // TODO: find the actual settings im guessing
    config.brightnessScalar = 0.5;
    config.disableWhenLOS = false;
    config.enableOptimizations = true;
    config.statusLedOffWhenActive = false;
    config.stripType = LEDStripType.GRB;
    config.v5Enabled = true;
    config.vBatOutputMode = VBatOutputMode.Off;
  }
}
