// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;


//Front Cam Data
// 9.75 deg pitch up
// 13.23 deg yaw inward

public class VisionConstants {

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "Back_Camera";
  public static String camera1Name = "Front_Right_Camera";
  public static String camera2Name = "Front_Left_Camera";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(
        Units.inchesToMeters(-14.690224), 0, Units.inchesToMeters(5.958701),
        new Rotation3d(0.0, Units.degreesToRadians(-23.75), Units.degreesToRadians(180.0)));
 
    public static Transform3d robotToCamera1 =
        new Transform3d(Units.inchesToMeters(-2.087630), Units.inchesToMeters(-7.5), Units.inchesToMeters(11.773733),
        new Rotation3d(0.0, Units.degreesToRadians(-9.75), Units.degreesToRadians(13.23)));

    public static Transform3d robotToCamera2 =
        new Transform3d(Units.inchesToMeters(-2.087630), Units.inchesToMeters(7.5), Units.inchesToMeters(11.773733),
        new Rotation3d(0.0, Units.degreesToRadians(-9.75), Units.degreesToRadians(-13.23)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
