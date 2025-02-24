package com.team1533.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;

public class ElevatorIOSim extends ElevatorIOReal {
    final GenericMotorController controller = new GenericMotorController(DCMotor.getKrakenX60Foc(2));
    final MomentOfInertia m = KilogramSquareMeters
            .of(Math.pow(ElevatorConstants.drumCircumferenceMeters / (2.0 * Math.PI), 2) * (1. / 16.) * 9.4);
    final Voltage v = Volts.of(ElevatorConstants.frictionVoltage);
    final SimMotorConfigs config = new SimMotorConfigs(DCMotor.getKrakenX60Foc(2), 1, m,
            v);
    final MapleMotorSim mechanismSim = new MapleMotorSim(config);

    private Notifier simNotifier = null;

    private double simPeriodSeconds = 0.005;
    
    public ElevatorIOSim() {
        simNotifier = new Notifier(() -> {
            updateSimState();
        });
        simNotifier.startPeriodic(simPeriodSeconds);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        super.updateInputs(inputs);
    }

    public void updateSimState() {
        var simState = leaderTalon.getSimState();

        simState.setSupplyVoltage(12.0);
        mechanismSim.useMotorController(controller).requestVoltage(simState.getMotorVoltageMeasure());
        Logger.recordOutput("Elevator/Sim/TalonMotorVoltage", simState.getMotorVoltage());

        mechanismSim.update(Seconds.of(simPeriodSeconds));
        Logger.recordOutput("Elevator/Sim/SimulatorVoltage", mechanismSim.getAppliedVoltage());

        double rotorPosition = mechanismSim.getAngularPosition().in(Revolutions);
        simState.setRawRotorPosition(rotorPosition);
        Logger.recordOutput("Elevator/Sim/RawRotorPosition", rotorPosition);

        double rotorVel = mechanismSim.getVelocity().in(RevolutionsPerSecond);
        simState.setRotorVelocity(rotorVel);
        Logger.recordOutput("Elevator/Sim/SimulatorVelocityRPS", rotorVel);
    }
}
