// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import static com.team1533.frc2025.subsystems.vision.VisionConstants.aprilTagLayout;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.ejml.dense.row.decomposition.hessenberg.TridiagonalDecompositionHouseholderOrig_FDRM;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.config.RobotConfig;
import com.team1533.frc2025.RobotContainer;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name             The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation = new TargetObservation(
            Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
            Rotation2d.fromDegrees(result.getBestTarget().getPitch()));

        for (var target : result.targets) {
          // Pinhole model using sensed tag distance instead of height difference

          Optional<Pose3d> tagPose = VisionConstants.aprilTagLayout.getTagPose(target.fiducialId);
          double tagDistance = target.getBestCameraToTarget().getTranslation().getNorm();

          if (tagPose.isEmpty() || tagPose.get().getZ() > Units.inchesToMeters(18) || tagDistance > 2) continue;
          
          // calculate direction vector using pitch/yaw
          Translation3d cameraToTag = new Translation3d(1, -Math.tan(Math.toRadians(target.getYaw())), Math.tan(Math.toRadians(target.getPitch())));
          // rescale to measured tag distance
          cameraToTag = cameraToTag.times(tagDistance / cameraToTag.getNorm());
          
          Translation3d robotToTag = cameraToTag.rotateBy(robotToCamera.getRotation());
          robotToTag = robotToTag.plus(robotToCamera.getTranslation());


          Rotation2d robotRotation = RobotContainer.getInstance().getDriveSubsystem().getRotation();
          // rotate to field coordinates
          Translation2d robotToTagFC = robotToTag.toTranslation2d().rotateBy(robotRotation);
          Translation2d fieldToRobot = tagPose.get().getTranslation().toTranslation2d().minus(robotToTagFC);
          
          Pose3d robotPose = new Pose3d(new Translation3d(fieldToRobot), new Rotation3d(robotRotation));

          poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                0, // Ambiguity
                1, // Tag count
                tagDistance, // Average tag distance
                PoseObservationType.PINHOLE)); // Observation type
          
          tagIds.add((short) target.fiducialId);

        }
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      

      // Multitag
      /*
      if (result.multitagResult.isPresent()) {
        var multitagResult = result.multitagResult.get();

        // Calculate robot pose
        Transform3d fieldToCamera = multitagResult.estimatedPose.best;
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

        // Calculate average tag distance
        double totalTagDistance = 0.0;
        for (var target : result.targets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
        }

        // Add tag IDs
        tagIds.addAll(multitagResult.fiducialIDsUsed);

        // Add observation
        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(), // Timestamp
                robotPose, // 3D pose estimate
                multitagResult.estimatedPose.ambiguity, // Ambiguity
                multitagResult.fiducialIDsUsed.size(), // Tag count
                totalTagDistance / result.targets.size(), // Average tag distance
                PoseObservationType.SOLVE_PNP)); // Observation type
      }
      */
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
