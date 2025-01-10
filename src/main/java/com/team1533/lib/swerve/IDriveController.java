package com.team1533.lib.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface IDriveController {
    ChassisSpeeds transform(DriveInput driveInput, Pose2d robotPose);
}
