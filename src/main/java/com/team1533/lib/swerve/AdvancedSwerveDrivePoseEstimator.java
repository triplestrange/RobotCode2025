package com.team1533.lib.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class AdvancedSwerveDrivePoseEstimator extends SwerveDrivePoseEstimator  {

    public AdvancedSwerveDrivePoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle,
    SwerveModulePosition[] modulePositions, Pose2d initalPose2dMeters)   {
        super(kinematics, gyroAngle, modulePositions, initalPose2dMeters);
    }
}
