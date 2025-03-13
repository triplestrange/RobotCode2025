package com.team1533.frc2025.subsystems.wrist;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Pound;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;
import org.littletonrobotics.junction.Logger;

import com.team1533.frc2025.Constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;

public class WristIOSim extends WristIOReal {

    final GenericMotorController controller = new GenericMotorController(DCMotor.getKrakenX60Foc(2));
    final MomentOfInertia m = KilogramSquareMeters
            .of((1. / 3.) * Kilogram.convertFrom(7.1342598, Pound) * Math.pow(Meter.convertFrom(25, Inch), 2)
                    * (1. / WristConstants.reduction));
    final Voltage v = Volts.of(WristConstants.frictionVoltage);
    final SimMotorConfigs config = new SimMotorConfigs(DCMotor.getKrakenX60Foc(2), 1, m,
            v);
    final MapleMotorSim mechanismSim = new MapleMotorSim(config);

    private Notifier simNotifier = null;

    public WristIOSim() {
        simNotifier = new Notifier(() -> {
            updateSimState();
        });
        simNotifier.startPeriodic(Constants.kSimDt);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        super.updateInputs(inputs);
    }

    public void updateSimState() {
        var simState = leaderTalon.getSimState();

        simState.setSupplyVoltage(12.0);
        mechanismSim.useMotorController(controller).requestVoltage(simState.getMotorVoltageMeasure());
        Logger.recordOutput("Wrist/Sim/TalonMotorVoltage", simState.getMotorVoltage());

        double simVoltage = mechanismSim.useMotorController(controller)
                .updateControlSignal(mechanismSim.getAngularPosition(), mechanismSim.getVelocity(),
                        mechanismSim.getEncoderPosition(), mechanismSim.getEncoderVelocity())
                .baseUnitMagnitude();

        Logger.recordOutput("Wrist/Sim/SimulatorVoltage", simVoltage);

        double simPositionMeters = mechanismSim.getAngularPosition().baseUnitMagnitude();
        Logger.recordOutput("Wrist/Sim/SimulatorPositionMeters", simPositionMeters);
        mechanismSim.update(Seconds.of(Constants.kSimDt));
        Logger.recordOutput("Wrist/Sim/SimulatorVoltage", mechanismSim.getAppliedVoltage());

        double rotorPosition = mechanismSim.getAngularPosition().in(Revolutions);
        simState.setRawRotorPosition(rotorPosition);
        Logger.recordOutput("Wrist/Sim/RawRotorPosition", rotorPosition);

        double rotorVel = mechanismSim.getVelocity().in(RevolutionsPerSecond);
        simState.setRotorVelocity(rotorVel);
        Logger.recordOutput("Wrist/Sim/SimulatorVelocityRPS", rotorVel);
    }

}
