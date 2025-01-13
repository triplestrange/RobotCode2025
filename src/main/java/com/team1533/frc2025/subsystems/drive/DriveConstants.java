package com.team1533.frc2025.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.team1533.frc2025.generated.TunerConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class DriveConstants {

        // TunerConstants doesn't include these constants, so they are declared locally
        static final double ODOMETRY_FREQUENCY = new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD()
                        ? 250.0
                        : 100.0;

        public static final double DRIVE_BASE_RADIUS = Math.max(
                        Math.max(
                                        Math.hypot(TunerConstants.FrontLeft.LocationX,
                                                        TunerConstants.FrontRight.LocationY),
                                        Math.hypot(TunerConstants.FrontRight.LocationX,
                                                        TunerConstants.FrontRight.LocationY)),
                        Math.max(
                                        Math.hypot(TunerConstants.BackLeft.LocationX,
                                                        TunerConstants.BackLeft.LocationY),
                                        Math.hypot(TunerConstants.BackRight.LocationX,
                                                        TunerConstants.BackRight.LocationY)));

        // PathPlanner config constants
        public static final double ROBOT_MASS_KG = 54.4311;
        public static final double ROBOT_MOI = 6;
        public static final double WHEEL_COF = 10;
        public static final RobotConfig PP_CONFIG = new RobotConfig(
                        ROBOT_MASS_KG,
                        ROBOT_MOI,
                        new ModuleConfig(
                                        TunerConstants.FrontLeft.WheelRadius,
                                        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                                        WHEEL_COF,
                                        DCMotor.getKrakenX60Foc(1)
                                                        .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                                        TunerConstants.FrontLeft.SlipCurrent,
                                        1),
                        getModuleTranslations());

        public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
                        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
                        .withCustomModuleTranslations(getModuleTranslations())
                        .withGyro(COTS.ofPigeon2())
                        .withSwerveModule(
                                        new SwerveModuleSimulationConfig(
                                                        DCMotor.getKrakenX60Foc(1),
                                                        DCMotor.getKrakenX60(1),
                                                        TunerConstants.FrontLeft.DriveMotorGearRatio,
                                                        TunerConstants.FrontLeft.SteerMotorGearRatio,
                                                        Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                                                        Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                                                        Meters.of(TunerConstants.FrontLeft.WheelRadius),
                                                        KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                                                        WHEEL_COF));

        /** Returns an array of module translations. */
        public static Translation2d[] getModuleTranslations() {
                return new Translation2d[] {
                                new Translation2d(TunerConstants.FrontLeft.LocationX,
                                                TunerConstants.FrontLeft.LocationY),
                                new Translation2d(TunerConstants.FrontRight.LocationX,
                                                TunerConstants.FrontRight.LocationY),
                                new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                                new Translation2d(TunerConstants.BackRight.LocationX,
                                                TunerConstants.BackRight.LocationY)
                };
        }

}
