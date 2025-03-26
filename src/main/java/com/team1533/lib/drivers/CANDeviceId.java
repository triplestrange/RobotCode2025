// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.drivers;

public class CANDeviceId {
  private final int deviceNumber;
  private final String bus;

  public CANDeviceId(int deviceNumber, String bus) {
    this.deviceNumber = deviceNumber;
    this.bus = bus;
  }

  // Use the default bus name (empty string).
  public CANDeviceId(int deviceNumber) {
    this(deviceNumber, "");
  }

  public int getDeviceNumber() {
    return deviceNumber;
  }

  public String getBus() {
    return bus;
  }

  public boolean equals(CANDeviceId other) {
    return other.deviceNumber == deviceNumber && other.bus == bus;
  }
}
