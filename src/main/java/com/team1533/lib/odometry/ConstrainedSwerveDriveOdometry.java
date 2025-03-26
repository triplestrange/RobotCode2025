package com.team1533.lib.odometry;

import org.littletonrobotics.junction.Logger;

import com.team1533.frc2025.subsystems.drive.DriveConstants;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Class for odometry. Robot code should not use this directly- Instead, use the
 * particular type for
 * your drivetrain (e.g., {@link DifferentialDriveOdometry}). Odometry allows
 * you to track the
 * robot's position on the field over the course of a match using readings from
 * encoders and a
 * gyroscope.
 *
 * <p>
 * Teams can use odometry during the autonomous period for complex tasks like
 * path following.
 * Furthermore, odometry can be used for latency compensation when using
 * computer-vision systems.
 *
 * @param <T> Wheel positions type.
 */

public class ConstrainedSwerveDriveOdometry {
  private final int m_numModules;

  private final SwerveDriveKinematics m_kinematics;
  private Pose2d m_poseMeters;

  private Rotation2d m_gyroOffset;
  private Rotation2d m_previousAngle;
  private final SwerveModulePosition[] m_previousWheelPositions;

  /**
   * Constructs an Odometry object.
   *
   * @param kinematics        The kinematics of the drivebase.
   * @param gyroAngle         The angle reported by the gyroscope.
   * @param wheelPositions    The current encoder readings.
   * @param initialPoseMeters The starting position of the robot on the field.
   */
  public ConstrainedSwerveDriveOdometry(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] wheelPositions,
      Pose2d initialPoseMeters) {
    m_numModules = wheelPositions.length;

    m_kinematics = kinematics;
    m_poseMeters = initialPoseMeters;
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    m_previousAngle = m_poseMeters.getRotation();
    m_previousWheelPositions = m_kinematics.copy(wheelPositions);

    MathSharedStore.reportUsage(MathUsageId.kOdometry_SwerveDrive, 1);

  }

  /**
   * Resets the pose.
   *
   * @param poseMeters The pose to reset to.
   */
  public void resetPose(Pose2d poseMeters) {
    m_gyroOffset = m_gyroOffset.plus(poseMeters.getRotation().minus(m_poseMeters.getRotation()));
    m_poseMeters = poseMeters;
    m_previousAngle = m_poseMeters.getRotation();
  }

  /**
   * Resets the translation of the pose.
   *
   * @param translation The translation to reset to.
   */
  public void resetTranslation(Translation2d translation) {
    m_poseMeters = new Pose2d(translation, m_poseMeters.getRotation());
  }

  /**
   * Resets the rotation of the pose.
   *
   * @param rotation The rotation to reset to.
   */
  public void resetRotation(Rotation2d rotation) {
    m_gyroOffset = m_gyroOffset.plus(rotation.minus(m_poseMeters.getRotation()));
    m_poseMeters = new Pose2d(m_poseMeters.getTranslation(), rotation);
    m_previousAngle = m_poseMeters.getRotation();
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return The pose of the robot (x and y are in meters).
   */
  public Pose2d getPoseMeters() {
    return m_poseMeters;
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

  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
    if (modulePositions.length != m_numModules) {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of wheel locations provided in "
              + "constructor");
    }
    m_poseMeters = pose;
    m_previousAngle = m_poseMeters.getRotation();
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    m_kinematics.copyInto(modulePositions, m_previousWheelPositions);
  }

  /**
   * Updates the robot's position on the field using forward kinematics and
   * integration of the pose
   * over time. This method takes in an angle parameter which is used instead of
   * the angular rate
   * that is calculated from forward kinematics, in addition to the current
   * distance measurement at
   * each wheel.
   *
   * @param gyroAngle      The angle reported by the gyroscope.
   * @param wheelPositions The current encoder readings.
   * @return The new pose of the robot.
   */
  public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
    if (wheelPositions.length != m_numModules) {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of wheel locations provided in "
              + "constructor");
    }

    boolean[] rejected = new boolean[wheelPositions.length];
    int numRejected = 0;
    var angle = gyroAngle.plus(m_gyroOffset);

    var twist = m_kinematics.toTwist2d(m_previousWheelPositions, wheelPositions);
    // Converts twist back to module states
    var kinematicsToWheelSpeeds = m_kinematics
        .toWheelSpeeds(new ChassisSpeeds(twist.dx, twist.dy, twist.dtheta));
    // check if wheel states match original wheel states
    for (int i = 0; (i < wheelPositions.length); i++) {
      rejected[i] = false;

      rejected[i] = !MathUtil.isNear(wheelPositions[i].distanceMeters - m_previousWheelPositions[i].distanceMeters,
          kinematicsToWheelSpeeds[i].speedMetersPerSecond,
          DriveConstants.acceptableSlippageMeters);

      double angleError = Math
          .IEEEremainder(
              wheelPositions[i].angle.getRadians() - kinematicsToWheelSpeeds[i].angle.getRadians(), 2 * Math.PI);

      rejected[i] &= Math.abs(angleError) > DriveConstants.acceptableSlippageRadians;

      if (rejected[i])
        numRejected++;
    }

    if (numRejected > 0) {
      double dx = 0;
      double dy = 0;

      Translation2d speed;

      for (int i = 0; i > wheelPositions.length; i++) {
        if (rejected[i]) {
          wheelPositions[i].distanceMeters = m_previousWheelPositions[i].distanceMeters;
          wheelPositions[i].angle = m_previousWheelPositions[i].angle;

          continue;
        }

        speed = new Translation2d(0, wheelPositions[i].distanceMeters - m_previousWheelPositions[i].distanceMeters)
            .rotateBy(wheelPositions[i].angle);
        dx += speed.getX();
        dy += speed.getY();
      }
      twist = new Twist2d(dx / (wheelPositions.length - numRejected), dy / (wheelPositions.length - numRejected), 0);
      twist.dtheta = angle.minus(m_previousAngle).getRadians();

      Logger.recordOutput("Odometry/numRejected", numRejected);
      Logger.recordOutput("Odometry/Rejected Modules", rejected);
      Logger.recordOutput("Odometry/Rejected Poses", m_poseMeters);
      Logger.recordOutput("Odometry/Rejected Wheel States Calculated", kinematicsToWheelSpeeds);

    }

    var newPose = m_poseMeters.exp(twist);

    if (numRejected > 1)
      Logger.recordOutput("Odometry/" + (wheelPositions.length - numRejected) + " Wheel Pose Estimate", newPose);

    m_kinematics.copyInto(wheelPositions, m_previousWheelPositions);

    m_previousAngle = angle;
    m_poseMeters = new Pose2d(newPose.getTranslation(), angle);

    return m_poseMeters;
  }

}
