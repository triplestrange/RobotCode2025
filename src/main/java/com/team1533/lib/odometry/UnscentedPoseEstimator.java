// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1533.lib.odometry;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.team1533.lib.util.InterpolatingDouble;
import com.team1533.lib.util.InterpolatingTreeMapBuffer;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * This class wraps {@link Odometry} to fuse latency-compensated vision
 * measurements with encoder
 * measurements. Robot code should not use this directly- Instead, use the
 * particular type for your
 * drivetrain (e.g., {@link DifferentialDrivePoseEstimator}). It is intended to
 * be a drop-in
 * replacement for {@link Odometry}; in fact, if you never call {@link
 * PoseEstimator#addVisionMeasurement} and only call
 * {@link PoseEstimator#update} then this will
 * behave exactly the same as Odometry.
 *
 * <p>
 * {@link PoseEstimator#update} should be called every robot loop.
 *
 * <p>
 * {@link PoseEstimator#addVisionMeasurement} can be called as infrequently as
 * you want; if you
 * never call it then this class will behave exactly like regular encoder
 * odometry.
 *
 * @param <T> Wheel positions type.
 */
public class UnscentedPoseEstimator {
    private final ConstrainedSwerveDriveOdometry m_odometry;
    private UnscentedKalmanFilter<N3, N3, N3> kalman;
    private final Matrix<N3, N1> m_q = new Matrix<>(Nat.N3(), Nat.N1());
    private final Matrix<N3, N1> m_r = new Matrix<>(Nat.N3(), Nat.N1());
    private double m_loopPeriodicSecs;

    private static final double kBufferDuration = 1.5;
    private static final int kObservationBufferSize = 50;
    // Maps timestamps to odometry-only pose estimates
    private final TimeInterpolatableBuffer<Pose2d> m_odom_to_robot = TimeInterpolatableBuffer
            .createBuffer(kBufferDuration);
    // Maps timestamps to vision updates
    // Always contains one entry before the oldest entry in m_odom_to_robot,
    // unless there have
    // been no vision measurements after the last reset
    private final InterpolatingTreeMapBuffer<InterpolatingDouble, Pose2d> m_field_to_odom = new InterpolatingTreeMapBuffer<>(
            kObservationBufferSize);
    private VisionUpdate mLatestVisionUpdate;
    private Pose2d m_poseEstimate;

    /**
     * Constructs a PoseEstimator.
     *
     * @param kinematics               A correctly-configured kinematics object for
     *                                 your drivetrain.
     * @param odometry                 A correctly-configured odometry object for
     *                                 your drivetrain.
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
    @SuppressWarnings("PMD.UnusedFormalParameter")
    public UnscentedPoseEstimator(
            SwerveDriveKinematics kinematics,
            ConstrainedSwerveDriveOdometry odometry,
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs, double loopPeriodicSecs) {
        m_odometry = odometry;
        this.m_loopPeriodicSecs = loopPeriodicSecs;
        kalman = new UnscentedKalmanFilter<>(Nat.N3(), Nat.N3(), (x, u) -> VecBuilder.fill(0.0, 0.0, 0.0), (x, u) -> x,
                stateStdDevs, visionMeasurementStdDevs,
                loopPeriodicSecs);

        m_poseEstimate = m_odometry.getPoseMeters();

        for (int i = 0; i < 3; ++i) {
            m_q.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }
        for (int i = 0; i < 3; ++i) {
            m_r.set(i, 0, visionMeasurementStdDevs.get(i, 0) * visionMeasurementStdDevs.get(i, 0));
        }
    }

    /**
     * Sets the pose estimator's trust of global measurements. This might be used to
     * change trust in
     * vision measurements after the autonomous period, or to change trust as
     * distance to a vision
     * target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision
     *                                 measurements. Increase these
     *                                 numbers to trust global measurements from
     *                                 vision less. This matrix is in the form [x,
     *                                 y,
     *                                 theta]ᵀ, with units in meters and radians.
     */

    public void resetKalmanFilters() {
        kalman = new UnscentedKalmanFilter<>(Nat.N3(), Nat.N3(), (x, u) -> VecBuilder.fill(0.0, 0.0, 0.0), (x, u) -> x,
                m_q, m_r,
                m_loopPeriodicSecs);

    }

    /**
     * Resets the robot's position on the field.
     *
     * <p>
     * The gyroscope angle does not need to be reset here on the user's robot code.
     * The library
     * automatically takes care of offsetting the gyro angle.
     *
     * @param gyroAngle      The angle reported by the gyroscope.
     * @param wheelPositions The current encoder readings.
     * @param poseMeters     The position on the field that your robot is at.
     */
    public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions, Pose2d poseMeters) {
        // Reset state estimate and error covariance
        m_odometry.resetPosition(gyroAngle, wheelPositions, poseMeters);
        m_odom_to_robot.clear();
        m_field_to_odom.clear();
        m_poseEstimate = m_odometry.getPoseMeters();
    }

    /**
     * Resets the robot's pose.
     *
     * @param pose The pose to reset to.
     */
    public void resetPose(Pose2d pose) {
        m_odometry.resetPose(pose);
        m_odom_to_robot.clear();
        m_field_to_odom.clear();
        m_poseEstimate = m_odometry.getPoseMeters();
    }

    /**
     * Resets the robot's translation.
     *
     * @param translation The pose to translation to.
     */
    public void resetTranslation(Translation2d translation) {
        m_odometry.resetTranslation(translation);
        m_odom_to_robot.clear();
        m_field_to_odom.clear();
        m_poseEstimate = m_odometry.getPoseMeters();
    }

    /**
     * Resets the robot's rotation.
     *
     * @param rotation The rotation to reset to.
     */
    public void resetRotation(Rotation2d rotation) {
        m_odometry.resetRotation(rotation);
        m_odom_to_robot.clear();
        m_field_to_odom.clear();
        m_poseEstimate = m_odometry.getPoseMeters();
    }

    /**
     * Gets the estimated robot pose.
     *
     * @return The estimated robot pose in meters.
     */
    public Pose2d getEstimatedPosition() {
        return m_poseEstimate;
    }

    public Optional<Pose2d> getOdomToRobot(double timestamp) {
        return m_odom_to_robot.getSample(timestamp);
    }

    public Pose2d getLatestOdomToRobot() {
        return m_odometry.getPoseMeters();
    }

    public Pose2d getFieldToOdom(double timestamp) {
        return m_field_to_odom.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public Optional<VisionUpdate> getLatestVisionUpdate() {
        if (mLatestVisionUpdate != null)
            return Optional.of(mLatestVisionUpdate);
        return Optional.empty();
    }

    /**
     * Return the pose at a given timestamp, if the buffer is not empty.
     *
     * @param timestampSeconds The pose's timestamp in seconds.
     * @return The pose at the given timestamp (or Optional.empty() if the buffer is
     *         empty).
     */
    public Optional<Pose2d> sampleAt(double timestampSeconds) {
        // Step 0: If there are no odometry updates to sample, skip.
        if (m_odom_to_robot.getInternalBuffer().isEmpty()) {
            return Optional.empty();
        }

        // Step 1: Make sure timestamp matches the sample from the odometry pose buffer.
        // (When sampling,
        // the buffer will always use a timestamp between the first and last timestamps)
        double oldestOdometryTimestamp = m_odom_to_robot.getInternalBuffer().firstKey();
        double newestOdometryTimestamp = m_odom_to_robot.getInternalBuffer().lastKey();
        timestampSeconds = MathUtil.clamp(timestampSeconds, oldestOdometryTimestamp, newestOdometryTimestamp);

        // Step 2: If there are no applicable vision updates, use the odometry-only
        // information.
        if (m_field_to_odom.isEmpty()
                || timestampSeconds < m_field_to_odom.firstKey().value) {
            return m_odom_to_robot.getSample(timestampSeconds);
        }

        // Step 3: Get the latest vision update from before or at the timestamp to
        // sample at.
        Transform2d visionUpdate = new Transform2d(
                m_field_to_odom.getInterpolated(new InterpolatingDouble(timestampSeconds)).getX(),
                m_field_to_odom.getInterpolated(new InterpolatingDouble(timestampSeconds)).getY(),
                m_field_to_odom.getInterpolated(new InterpolatingDouble(timestampSeconds)).getRotation());

        // Step 4: Get the pose measured by odometry at the time of the sample.
        var odometryEstimate = m_odom_to_robot.getSample(timestampSeconds);

        // Step 5: Apply the vision compensation to the odometry pose.
        return odometryEstimate.map(odometryPose -> odometryPose.transformBy(visionUpdate));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>
     * This method can be called as infrequently as you want, as long as you are
     * calling {@link
     * PoseEstimator#update} every loop.
     *
     * <p>
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we
     * recommend only adding vision measurements that are already within one meter
     * or so of the
     * current pose estimate.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds. Note that if you
     *                              don't use your own time source by calling {@link
     *                              PoseEstimator#updateWithTime(double,Rotation2d,Object)}
     *                              then you must use a timestamp with
     *                              an epoch since FPGA startup (i.e., the epoch of
     *                              this timestamp is the same epoch as {@link
     *                              edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.)
     *                              This means that you should use {@link
     *                              edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}
     *                              as your time source or sync the epochs.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        // Step 0: If this measurement is old enough to be outside the pose buffer's
        // timespan, skip.
        if (m_odom_to_robot.getInternalBuffer().isEmpty()
                || m_odom_to_robot.getInternalBuffer().lastKey() - kBufferDuration > timestampSeconds) {
            return;
        }

        // Step 2: Get the pose measured by odometry at the moment the vision
        // measurement was made.
        var odometrySample = m_odom_to_robot.getSample(timestampSeconds);

        if (odometrySample.isEmpty()) {
            return;
        }

        // Step 3: Get the vision-compensated pose estimate at the moment the vision
        // measurement was
        // made.
        var field_to_robot = sampleAt(timestampSeconds);

        if (field_to_robot.isEmpty()) {
            return;
        }

        // Step 4: Measure the twist between the old pose estimate and the vision pose.
        var twist = field_to_robot.get().log(visionRobotPoseMeters);

        kalman.correct(VecBuilder.fill(0.0, 0.0, 0.0),
                VecBuilder.fill(twist.dx,
                        twist.dy,
                        twist.dtheta));

        var scaledTwist = new Twist2d(kalman.getXhat(0), kalman.getXhat(1), kalman.getXhat(2));
        // Step 7: Calculate and record the vision update.
        mLatestVisionUpdate = new VisionUpdate(field_to_robot.get().exp(scaledTwist),
                odometrySample.get());

        m_field_to_odom.put(new InterpolatingDouble(timestampSeconds),
                new Pose2d(
                        mLatestVisionUpdate.visionPose.minus(visionRobotPoseMeters).getTranslation(),
                        mLatestVisionUpdate.visionPose.minus(visionRobotPoseMeters).getRotation()));

        // Step 9: Update latest pose estimate. Since we cleared all updates after this
        // vision update,
        // it's guaranteed to be the latest vision update.
        m_poseEstimate = mLatestVisionUpdate.compensate(m_odometry.getPoseMeters());
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>
     * This method can be called as infrequently as you want, as long as you are
     * calling {@link
     * PoseEstimator#update} every loop.
     *
     * <p>
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we
     * recommend only adding vision measurements that are already within one meter
     * or so of the
     * current pose estimate.
     *
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue
     * to apply to future measurements until a subsequent call to {@link
     * PoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds. Note that if you
     *                                 don't use your own time source by calling
     *                                 {@link #updateWithTime}, then you must use a
     *                                 timestamp with an epoch since FPGA startup
     *                                 (i.e., the epoch of this timestamp is the
     *                                 same
     *                                 epoch as
     *                                 {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}).
     *                                 This means that you
     *                                 should use
     *                                 {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}
     *                                 as your time source in
     *                                 this case.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement (x position
     *                                 in meters, y position in meters, and heading
     *                                 in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    @Deprecated
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    @Deprecated
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
    }

    /**
     * Updates the pose estimator with wheel encoder and gyro information. This
     * should be called every
     * loop.
     *
     * @param gyroAngle      The current gyro angle.
     * @param wheelPositions The current encoder readings.
     * @return The estimated pose of the robot in meters.
     */
    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
        return updateWithTime(MathSharedStore.getTimestamp(), gyroAngle, wheelPositions);
    }

    /**
     * Updates the pose estimator with wheel encoder and gyro information. This
     * should be called every
     * loop.
     *
     * @param currentTimeSeconds Time at which this method was called, in seconds.
     * @param gyroAngle          The current gyro angle.
     * @param wheelPositions     The current encoder readings.
     * @return The estimated pose of the robot in meters.
     */
    public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle,
            SwerveModulePosition[] wheelPositions) {
        Pose2d odometryEstimate = m_odometry.update(gyroAngle, wheelPositions);
        Logger.recordOutput("Odometry/Pure Odometry", odometryEstimate);
        m_odom_to_robot.addSample(currentTimeSeconds, odometryEstimate);

        if (m_field_to_odom.isEmpty()) {
            m_poseEstimate = odometryEstimate;
        } else {
            m_poseEstimate = mLatestVisionUpdate.compensate(odometryEstimate);
        }

        return getEstimatedPosition();
    }

    /**
     * Represents a vision update record. The record contains the vision-compensated
     * pose estimate as
     * well as the corresponding odometry pose estimate.
     */
    private static final class VisionUpdate {
        // The vision-compensated pose estimate.
        private final Pose2d visionPose;

        // The pose estimated based solely on odometry.
        private final Pose2d odometryPose;

        /**
         * Constructs a vision update record with the specified parameters.
         *
         * @param visionPose   The vision-compensated pose estimate.
         * @param odometryPose The pose estimate based solely on odometry.
         */
        private VisionUpdate(Pose2d visionPose, Pose2d odometryPose) {
            this.visionPose = visionPose;
            this.odometryPose = odometryPose;
        }

        /**
         * Returns the vision-compensated version of the pose. Specifically, changes the
         * pose from being
         * relative to this record's odometry pose to being relative to this record's
         * vision pose.
         *
         * @param pose The pose to compensate.
         * @return The compensated pose.
         */
        public Pose2d compensate(Pose2d pose) {
            var delta = pose.minus(this.odometryPose);
            return this.visionPose.plus(delta);
        }

    }
}
