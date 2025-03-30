// Copyright (c) 2025 FRC 1533
// http://github.com/triplestrange
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.loops;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team1533.lib.time.RobotTime;
import edu.wpi.first.wpilibj.Threads;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import org.littletonrobotics.junction.Logger;

/**
 * Thread loop for running a set of callbacks at a Hz driven by the update frequency of CTRE Status
 * Signals.
 */
public class StatusSignalLoop {
  protected List<Runnable> callbacks;
  protected List<BaseStatusSignal> statusSignals;

  protected double updateFrequency;
  protected AtomicBoolean running;

  protected Thread thread;

  public StatusSignalLoop(double updateFrequencyHz) {
    updateFrequency = updateFrequencyHz;
    running = new AtomicBoolean(false);

    thread = new Thread(this::run);
    thread.setDaemon(true);
    callbacks = new ArrayList<>();
    statusSignals = new ArrayList<>();
  }

  public StatusSignalLoop(double updateFrequencyHz, String name) {
    this(updateFrequencyHz);
    thread.setName(name);
  }

  public void register(IStatusSignalLoop loop) {
    addCallback(loop.getStatusSignals(), loop::onLoop);
  }

  public void addCallback(List<BaseStatusSignal> statusSignals, Runnable callback) {
    if (running.get()) {
      throw new IllegalStateException("Unable to add new callback while thread is running.");
    }
    callbacks.add(callback);
    this.statusSignals.addAll(statusSignals);
  }

  public void start() {
    running.set(true);
    thread.start();
  }

  protected void run() {
    BaseStatusSignal[] signals = new BaseStatusSignal[statusSignals.size()];
    signals = statusSignals.toArray(signals);
    BaseStatusSignal.setUpdateFrequencyForAll(updateFrequency, signals);
    // Thread priority taken from SwerveDrive
    Threads.setCurrentThreadPriority(true, 2);

    double lastTime = RobotTime.getTimestampSeconds();
    while (running.get()) {
      try {
        Thread.sleep((long) (1000.0 / updateFrequency));
      } catch (InterruptedException e) {
      }
      for (var runnable : callbacks) {
        runnable.run();
      }

      double currentTime = RobotTime.getTimestampSeconds();
      Logger.recordOutput("FastLooper/loopTime", currentTime - lastTime);
      lastTime = currentTime;
    }
  }

  public void stop(long millis) {
    running.set(false);
    try {
      thread.join(millis);
    } catch (InterruptedException ex) {
      Thread.currentThread().interrupt();
    }
  }
}
