package com.team1533.lib.odometry;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class StrangeSwerveDrivePoseEstimator extends UnscentedPoseEstimator {
    private final int m_numModules;

    /**
     * Constructs a SwerveDrivePoseEstimator.
     *
     * @param kinematics               A correctly-configured kinematics object for
     *                                 your drivetrain.
     * @param gyroAngle                The current gyro angle.
     * @param modulePositions          The current distance and rotation
     *                                 measurements of the swerve modules.
     * @param initialPoseMeters        The starting pose estimate.
     * @param stateStdDevs             Standard deviations of the pose estimate (x
     *                                 position in meters, y position
     *                                 in meters, and heading in radians). Increase
     *                                 these numbers to trust your state estimate
     *                                 less.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement (x position
     *                                 in meters, y position in meters, and heading
     *                                 in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    public StrangeSwerveDrivePoseEstimator(
            SwerveDriveKinematics kinematics, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions,
            Pose2d initalPose2dMeters,
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs,
            double loopPeriodicSecs) {
        super(
                kinematics,
                new ConstrainedSwerveDriveOdometry(kinematics, gyroAngle, modulePositions, initalPose2dMeters),
                stateStdDevs,
                visionMeasurementStdDevs, loopPeriodicSecs);

        m_numModules = modulePositions.length;
    }

    @Override
    public Pose2d updateWithTime(
            double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
        if (wheelPositions.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }

        return super.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);
    }
}
