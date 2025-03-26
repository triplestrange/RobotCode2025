// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.function.Supplier;

public class CameraConfig {

  private String camName;
  private Pose3d camPos;
  private Supplier<Pose3d> camPosSupplier;
  private boolean useSupplier = false;

  public CameraConfig(String camName, Pose3d pos) {
    this.camName = camName;
    this.camPos = pos;
  }

  public CameraConfig(String camName, Supplier<Pose3d> posSupplier) {
    this.camName = camName;
    this.camPosSupplier = posSupplier;
    this.useSupplier = true;
  }

  public String getCamName() {
    return camName;
  }

  public Pose3d getCamPos() {

    if (useSupplier) {
      return camPosSupplier.get();
    } else {
      return camPos;
    }
  }
}
