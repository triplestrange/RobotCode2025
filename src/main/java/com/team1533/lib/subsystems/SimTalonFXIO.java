// Copyright (c) 2025 FRC 1533
// 
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team1533.lib.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team1533.lib.time.RobotTime;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;
import org.littletonrobotics.junction.Logger;

public class SimTalonFXIO extends TalonFXIO {
  final GenericMotorController controller = new GenericMotorController(DCMotor.getKrakenX60Foc(1));
  final SimMotorConfigs configMaple;
  final MapleMotorSim mechanismSim;

  private Notifier simNotifier = null;
  private double lastUpdateTimestamp = 0.0;

  public SimTalonFXIO(ServoMotorSubsystemConfig config) {
    super(config);

    configMaple = new SimMotorConfigs(DCMotor.getKrakenX60Foc(1), config.unitToRotorRatio,
        KilogramSquareMeters.of(config.momentOfInertia), Volts.of(0.25));
    mechanismSim = new MapleMotorSim(configMaple);

    // Assume that config is correct (which it might not be)
    talon.getSimState().Orientation = (config.fxConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive)
        ? ChassisReference.Clockwise_Positive
        : ChassisReference.CounterClockwise_Positive;

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier = new Notifier(
        () -> {
          updateSimState();
        });
    simNotifier.startPeriodic(0.005);
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    super.updateInputs(inputs);
  }

  protected void updateSimState() {
    var simState = talon.getSimState();
    simState.setSupplyVoltage(12.0);
    mechanismSim.useMotorController(controller).requestVoltage(simState.getMotorVoltageMeasure());

    double simVoltage = mechanismSim.useMotorController(controller)
        .updateControlSignal(mechanismSim.getAngularPosition(), mechanismSim.getVelocity(),
            mechanismSim.getEncoderPosition(), mechanismSim.getEncoderVelocity())
        .baseUnitMagnitude();

    Logger.recordOutput(config.name + "/Sim/SimulatorVoltage", simVoltage);

    double timestamp = RobotTime.getTimestampSeconds();
    mechanismSim.update(Microseconds.of(timestamp - lastUpdateTimestamp));
    lastUpdateTimestamp = timestamp;

    // Find current state of sim in radians from 0 point
    double simPositionRads = mechanismSim.getAngularPosition().in(Radian);
    Logger.recordOutput(config.name + "/Sim/SimulatorPositionRadians", simPositionRads);

    // Mutate rotor position
    double rotorPosition = Units.radiansToRotations(simPositionRads) / config.unitToRotorRatio;
    simState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput(config.name + "/Sim/setRawRotorPosition", rotorPosition);

    // Mutate rotor vel
    double rotorVel = mechanismSim.getVelocity().baseUnitMagnitude() / config.unitToRotorRatio;
    simState.setRotorVelocity(rotorVel);
    Logger.recordOutput(
        config.name + "/Sim/SimulatorVelocityRadS", mechanismSim.getVelocity().in(RadiansPerSecond));
  }
}
