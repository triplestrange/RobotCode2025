// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn
 * motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>
 * Device configuration and other behaviors not exposed by TunerConstants can be
 * customized here.
 */
public class ModuleIOTalonFXReal extends ModuleIOTalonFX {
  // Queue to read inputs from odometry thread
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  public ModuleIOTalonFXReal(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
    super(constants);
    this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    this.drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(super.drivePosition);
    this.turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(super.turnAbsolutePosition);

  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);

    // Update odometry inputs
    inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
        .mapToDouble((Double n) -> Units.rotationsToRadians(n) / constants.DriveMotorGearRatio).toArray();
    inputs.odometryTurnPositions = turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();

  }
}
