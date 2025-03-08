package com.team1533.lib.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AlignController {

    private ChassisSpeeds input = new ChassisSpeeds();
    @AutoLogOutput
    private ChassisSpeeds skewed = new ChassisSpeeds();

    private Pose2d target = Pose2d.kZero;
    private Pose2d current = Pose2d.kZero;

    private ProfiledPIDController pidController;

    public AlignController(double kP) {

    }

    public void setTarget(Pose2d target) {

    }

    public void setTarget(Supplier<Pose2d> targetSupplier) {

    }

    public ChassisSpeeds update() {

    }

}
