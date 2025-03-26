package com.team1533.lib.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.team1533.frc2025.RobotContainer;
import com.team1533.lib.swerve.HeadingController.HeadingControllerState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AlignController {

    private Pose2d target = Pose2d.kZero;
    private Pose2d current = Pose2d.kZero;
    private final Supplier<Pose2d> currentSupplier;

    private PIDController pidTranslationController;
    private HeadingController headingController;

    public AlignController(double kTranslationP, double kdt, Supplier<Pose2d> current) {
        pidTranslationController = new PIDController(kTranslationP, 0, 0, kdt);
        headingController = new HeadingController();
        this.currentSupplier = current;
    }

    public void setTarget(Pose2d target) {
        this.target = target;
        headingController.setGoal(target.getRotation());
        headingController.setM_HeadingControllerState(HeadingControllerState.SNAP);
        Logger.recordOutput("AlignController/Target", this.target);
    }

// must be called periodically
    public ChassisSpeeds update(ChassisSpeeds input) {
        if (target.equals(Pose2d.kZero)) return input;

        current = currentSupplier.get();
        ChassisSpeeds skewed = new ChassisSpeeds();

        boolean override = !RobotContainer.getInstance().isLeft() ||!RobotContainer.getInstance().isRight();

        // if (current.getTranslation().getDistance(target.getTranslation()) < -1000 || override)   {
        if (override)   {
            skewed.vxMetersPerSecond = MathUtil.clamp(pidTranslationController.calculate(current.getX(), target.getX()), -1., 1.);
            skewed.vyMetersPerSecond = MathUtil.clamp(pidTranslationController.calculate(current.getY(), target.getY()), -1., 1.);
        }

        // if (current.getTranslation().getDistance(target.getTranslation()) < -1000 || override)  {
        if (override)  {
            skewed.omegaRadiansPerSecond = headingController.update();
        }
        
        Logger.recordOutput("AlignController/SkewedSpeeds", skewed);

        return input.plus(ChassisSpeeds.fromFieldRelativeSpeeds(skewed, current.getRotation())); 
    }

}
