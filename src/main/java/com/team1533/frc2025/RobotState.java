// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.frc2025;

import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.IntSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.team1533.frc2025.subsystems.arm.ArmConstants;
import com.team1533.frc2025.subsystems.vision.VisionIO.PoseObservation;
import com.team1533.frc2025.subsystems.vision.VisionIO.PoseObservationType;
import com.team1533.frc2025.subsystems.vision.VisionSubsystem.VisionConsumer;
import com.team1533.frc2025.subsystems.wrist.WristConstants;
import com.team1533.lib.time.RobotTime;
import com.team1533.lib.util.ConcurrentTimeInterpolatableBuffer;
import com.team1533.lib.util.MathHelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import lombok.Setter;

public class RobotState implements VisionConsumer {

    public final static double LOOKBACK_TIME = 1.0;

    // Kinematic Frames
    private final ConcurrentTimeInterpolatableBuffer<Pose2d> fieldToRobot = ConcurrentTimeInterpolatableBuffer
            .createBuffer(LOOKBACK_TIME);
    private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());

    private final AtomicInteger iteration = new AtomicInteger(0);

    private double lastUsedMegatagTimestamp = 0;
    @Getter
    private double lastTriggeredIntakeSensorTimestamp = 0;
    @Getter
    private double lastTriggeredIntakeLaserTimestamp = 0;

    private ConcurrentTimeInterpolatableBuffer<Double> driveYawAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> driveRollAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> drivePitchAngularVelocity = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);

    private ConcurrentTimeInterpolatableBuffer<Double> drivePitchRads = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> driveRollRads = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> driveYawRads = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> accelX = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);
    private ConcurrentTimeInterpolatableBuffer<Double> accelY = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);

    private ConcurrentTimeInterpolatableBuffer<Double> armRots = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);

    private ConcurrentTimeInterpolatableBuffer<Double> elevExtensionM = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);

    private ConcurrentTimeInterpolatableBuffer<Double> wristRots = ConcurrentTimeInterpolatableBuffer
            .createDoubleBuffer(LOOKBACK_TIME);

    private final AtomicBoolean enablePathCancel = new AtomicBoolean(false);

    private Pose3d wristPose3d = new Pose3d();
    private Pose3d elevatorPose3d = new Pose3d();
    private Pose3d armPose3d = new Pose3d();
    private Pose3d funnelPose3d = new Pose3d();

    private Translation2d superstructureOrigin2d = new Translation2d(0.089153, 0.219531);

    public LoggedMechanism2d mechanism = new LoggedMechanism2d(Units.inchesToMeters(
            29.750000), Units.feetToMeters(7.0), new Color8Bit(Color.kDarkBlue));

    LoggedMechanismRoot2d root = mechanism.getRoot(
            "Superstructure Root", superstructureOrigin2d.getX(), superstructureOrigin2d.getY());

    public LoggedMechanismLigament2d arm = root
            .append(new LoggedMechanismLigament2d("arm", 0.66089, 0, 4, new Color8Bit(Color.kBlue)));
    public LoggedMechanismLigament2d elev = arm.append(
            new LoggedMechanismLigament2d("elev", 0.5, 0, 4, new Color8Bit(Color.kBlack)));
    public LoggedMechanismLigament2d wrist1 = elev
            .append(new LoggedMechanismLigament2d("wrist1", 0.097401, 0, 4, new Color8Bit(Color.kCrimson)));
    public LoggedMechanismLigament2d wrist2 = wrist1
            .append(new LoggedMechanismLigament2d("wrist2", 0.18561, 86.308414, 4, new Color8Bit(Color.kCrimson)));
    public LoggedMechanismLigament2d wrist3 = wrist2
            .append(new LoggedMechanismLigament2d("wrist3", 0.165042, 68.159446, 4,
                    new Color8Bit(Color.kAntiqueWhite)));
    private double autoStartTime;

    @Getter
    private static RobotState instance;

    /** Adds a new timestamped vision measurement. */
    @Override
    public void accept(
            PoseObservation observation,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        updatePoseObservation(observation, visionMeasurementStdDevs);
    }

    public RobotState() {
        instance = this;

        // Add one sample to protect callers against null
        fieldToRobot.addSample(0.0, MathHelpers.kPose2dZero);
        driveYawAngularVelocity.addSample(0.0, 0.0);
        armRots.addSample(0.0, 0.0);
        elevExtensionM.addSample(0.0, 0.0);
        wristRots.addSample(0.0, 0.0);
        driveYawRads.addSample(0.0, 0.0);
    }

    public void setAutoStartTime(double timestamp) {
        autoStartTime = timestamp;
    }

    public double getAutoStartTime() {
        return autoStartTime;
    }

    public void enablePathCancel() {
        enablePathCancel.set(true);
    }

    public void disablePathCancel() {
        enablePathCancel.set(false);
    }

    public boolean getPathCancel() {
        return enablePathCancel.get();
    }

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        fieldToRobot.addSample(timestamp, pose);
    }

    public void incrementIterationCount() {
        iteration.incrementAndGet();
    }

    public int getIteration() {
        return iteration.get();
    }

    public IntSupplier getIterationSupplier() {
        return () -> getIteration();
    }

    public void addDriveMotionMeasurements(double timestamp,
            double angularRollRadsPerS,
            double angularPitchRadsPerS,
            double angularYawRadsPerS,
            double pitchRads,
            double rollRads,
            double accelX,
            double accelY,
            ChassisSpeeds desiredFieldRelativeSpeeds,
            ChassisSpeeds measuredSpeeds,
            ChassisSpeeds measuredFieldRelativeSpeeds,
            ChassisSpeeds fusedFieldRelativeSpeeds) {
        this.driveRollAngularVelocity.addSample(timestamp, angularRollRadsPerS);
        this.drivePitchAngularVelocity.addSample(timestamp, angularPitchRadsPerS);
        this.driveYawAngularVelocity.addSample(timestamp, angularYawRadsPerS);
        this.drivePitchRads.addSample(timestamp, pitchRads);
        this.driveRollRads.addSample(timestamp, rollRads);
        this.accelY.addSample(timestamp, accelY);
        this.accelX.addSample(timestamp, accelX);
        this.desiredFieldRelativeChassisSpeeds.set(desiredFieldRelativeSpeeds);
        this.measuredRobotRelativeChassisSpeeds.set(measuredSpeeds);
        this.measuredFieldRelativeChassisSpeeds.set(measuredFieldRelativeSpeeds);
        this.fusedFieldRelativeChassisSpeeds.set(fusedFieldRelativeSpeeds);
    }

    public void addYawMeasurements(Rotation2d yaw, double timestamp) {
        this.driveYawRads.addSample(timestamp, yaw.getRadians());
    }

    public void addArmUpdate(double timestamp, double rots) {
        armRots.addSample(timestamp, rots);
    }

    public double getLatestArmPositionRotations() {
        return this.armRots.getInternalBuffer().lastEntry().getValue();
    }

    public void addElevUpdate(double timestamp, double extM) {
        elevExtensionM.addSample(timestamp, extM);
    }

    public double getLatestElevPositionMeters() {
        return this.elevExtensionM.getInternalBuffer().lastEntry().getValue();
    }

    public void addWristUpdate(double timestamp, double rots) {
        wristRots.addSample(timestamp, rots);
    }

    public double getLatestWristPositionRotations() {
        return this.wristRots.getInternalBuffer().lastEntry().getValue();
    }

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        return fieldToRobot.getLatest();
    }

    public Map.Entry<Double, Double> getLatestYawRads() {
        return driveYawRads.getLatest();
    }

    public Pose2d getPredictedFieldToRobot(double lookaheadTimeS) {
        var maybeFieldToRobot = getLatestFieldToRobot();
        Pose2d fieldToRobot = maybeFieldToRobot == null ? MathHelpers.kPose2dZero : maybeFieldToRobot.getValue();
        var delta = getLatestRobotRelativeChassisSpeed();
        delta = delta.times(lookaheadTimeS);
        return fieldToRobot
                .exp(new Twist2d(delta.vxMetersPerSecond, delta.vyMetersPerSecond, delta.omegaRadiansPerSecond));
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public Optional<Double> getYawRads(double timestamp) {
        return driveYawRads.getSample(timestamp);
    }

    public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
        return measuredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeed() {
        return desiredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeed() {
        return fusedFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedRobotRelativeChassisSpeed() {
        var speeds = getLatestRobotRelativeChassisSpeed();
        speeds.omegaRadiansPerSecond = getLatestFusedFieldRelativeChassisSpeed().omegaRadiansPerSecond;
        return speeds;
    }

    private Optional<Double> getMaxAbsValueInRange(ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime,
            double maxTime) {
        var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
        var max = submap.stream().max(Double::compare);
        var min = submap.stream().min(Double::compare);
        if (max.isEmpty() || min.isEmpty())
            return Optional.empty();
        if (Math.abs(max.get()) >= Math.abs(min.get()))
            return max;
        else
            return min;
    }

    public Optional<Double> getMaxAbsDriveYawAngularVelocityInRange(double minTime, double maxTime) {
        // Gyro yaw rate not set in sim.
        if (Robot.isReal())
            return getMaxAbsValueInRange(driveYawAngularVelocity, minTime, maxTime);
        return Optional.of(measuredRobotRelativeChassisSpeeds.get().omegaRadiansPerSecond);
    }

    public Optional<Double> getMaxAbsDrivePitchAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsValueInRange(drivePitchAngularVelocity, minTime, maxTime);
    }

    public Optional<Double> getMaxAbsDriveRollAngularVelocityInRange(double minTime, double maxTime) {
        return getMaxAbsValueInRange(driveRollAngularVelocity, minTime, maxTime);
    }

    public void updatePoseObservation(PoseObservation poseObservation, Matrix<N3, N1> visionMeasurementStdDevs) {

        if (poseObservation.type() == PoseObservationType.SOLVE_PNP)
            lastUsedMegatagTimestamp = Timer.getFPGATimestamp();
        RobotContainer.getInstance().getDriveSubsystem().getPoseEstimator().addVisionMeasurement(
                poseObservation.pose().toPose2d(),
                poseObservation.timestamp(), visionMeasurementStdDevs);
    }

    public void updateLastTriggeredIntakeSensorTimestamp(boolean triggered) {
        if (triggered)
            lastTriggeredIntakeSensorTimestamp = RobotTime.getTimestampSeconds();
    }

    public void updateLastTriggeredIntakeLaserTimestamp(boolean triggered) {
        if (triggered)
            lastTriggeredIntakeLaserTimestamp = RobotTime.getTimestampSeconds();
    }

    public double lastUsedMegatagTimestamp() {
        return lastUsedMegatagTimestamp;
    }

    public boolean isRedAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }

    public void updateLogger() {
        if (this.driveYawAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/YawAngularVelocity",
                    this.driveYawAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.driveRollAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/RollAngularVelocity",
                    this.driveRollAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.drivePitchAngularVelocity.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/PitchAngularVelocity",
                    this.drivePitchAngularVelocity.getInternalBuffer().lastEntry().getValue());
        }
        if (this.accelX.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/AccelX", this.accelX.getInternalBuffer().lastEntry().getValue());
        }
        if (this.accelY.getInternalBuffer().lastEntry() != null) {
            Logger.recordOutput("RobotState/AccelY", this.accelY.getInternalBuffer().lastEntry().getValue());
        }
        Logger.recordOutput("RobotState/DesiredChassisSpeedFieldFrame", getLatestDesiredFieldRelativeChassisSpeed());
        Logger.recordOutput("RobotState/MeasuredChassisSpeedFieldFrame", getLatestMeasuredFieldRelativeChassisSpeeds());
        Logger.recordOutput("RobotState/FusedChassisSpeedFieldFrame", getLatestFusedFieldRelativeChassisSpeed());
    }

    public void updateMech2dViz() {
        arm.setAngle(Units.rotationsToDegrees(getLatestArmPositionRotations() - ArmConstants.absEncoderOffset));
        elev.setLength(getLatestElevPositionMeters());
        wrist1.setAngle(Units.rotationsToDegrees(getLatestWristPositionRotations() - WristConstants.absEncoderOffset));

        // arm.setAngle(45);
        // elev.setLength(0);
        // wrist1.setAngle(0);

        Logger.recordOutput("score mech", mechanism);
    }

}