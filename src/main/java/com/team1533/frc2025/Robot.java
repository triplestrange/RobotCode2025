// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team1533.frc2025;

import java.util.HashMap;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.team1533.lib.loops.Looper;
import com.team1533.lib.util.Alert;
import com.team1533.lib.util.Alert.AlertType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Map;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Looper enabledLoop = new Looper(Constants.LoggerConstants.kSimDt);
  private Looper disabledLoop = new Looper(Constants.LoggerConstants.kSimDt);

  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private int i;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Set a metadata value

    switch (Constants.LoggerConstants.getMode()) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        try {
          Logger.addDataReceiver(new WPILOGWriter());
        }

        catch (Exception e) {
          new Alert("The USB Flash Drive is unplugged.", AlertType.WARNING)
              .set(true);
        }

        try {
          Logger.addDataReceiver(new NT4Publisher());

        }

        catch (Exception e) {
          new Alert("The Logger is not publsihing to Network Tables", AlertType.WARNING).set(true);
        }
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"), 0.01));
        break;
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in
    // the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
                    // be added.
    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
      String name = command.getName();
      int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
      commandCounts.put(name, count);
      Logger.recordOutput(
          "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
      Logger.recordOutput("CommandsAll/" + name, count > 0);
    };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              logCommandFunction.accept(command, true);
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
    RobotController.setBrownoutVoltage(6.0);
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer(this);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (i % 10 == 0) {
    }
    i++;
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    enabledLoop.stop();
    disabledLoop.start();
  }

  @Override
  public void disabledPeriodic() {

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
    enabledLoop.start();
    disabledLoop.stop();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // m_robotContainer.m_robotDrive.m_odometry
    // .setVisionMeasurementStdDevs(Constants.VisionConstants.VISION_MEASUREMENT_STD_DEVS);

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    enabledLoop.start();
    disabledLoop.stop();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    enabledLoop.start();
    disabledLoop.stop();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}