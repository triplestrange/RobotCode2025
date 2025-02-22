package com.team1533.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;
import org.littletonrobotics.junction.Logger;

import com.team1533.lib.time.RobotTime;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;

public class ElevatorIOSim extends ElevatorIOReal {
    final GenericMotorController controller = new GenericMotorController(DCMotor.getKrakenX60Foc(2));
    final MomentOfInertia m = KilogramSquareMeters
            .of(Math.pow(ElevatorConstants.drumCircumferenceMeters / (2.0 * Math.PI), 2) * (1. / 16.) * 9.4);
    final Voltage v = Volts.of(ElevatorConstants.frictionVoltage);
    final SimMotorConfigs config = new SimMotorConfigs(DCMotor.getKrakenX60Foc(2), ElevatorConstants.reduction, m,
            v);
    final MapleMotorSim mechanismSim = new MapleMotorSim(config);

    private Notifier simNotifier = null;

    private double lastUpdateTimestamp;

    public ElevatorIOSim() {
        simNotifier = new Notifier(() -> {
            updateSimState();
        });
        simNotifier.startPeriodic(0.005);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        super.updateInputs(inputs);
    }

    public void updateSimState() {
        var simState = leaderTalon.getSimState();

        simState.setSupplyVoltage(12.0);
        mechanismSim.useMotorController(controller).requestVoltage(simState.getMotorVoltageMeasure());

        double simVoltage = mechanismSim.useMotorController(controller)
                .updateControlSignal(mechanismSim.getAngularPosition(), mechanismSim.getVelocity(),
                        mechanismSim.getEncoderPosition(), mechanismSim.getEncoderVelocity())
                .baseUnitMagnitude();
        ;
        Logger.recordOutput("Elevator/Sim/SimulatorVoltage", simVoltage);

        double timestamp = RobotTime.getTimestampSeconds();
        mechanismSim.update(Microseconds.of(timestamp - lastUpdateTimestamp));
        lastUpdateTimestamp = timestamp;

        double simPositionMeters = mechanismSim.getAngularPosition().baseUnitMagnitude();
        Logger.recordOutput("Elevator/Sim/SimulatorPositionMeters", simPositionMeters);

        double rotorPosition = simPositionMeters / ElevatorConstants.reduction;
        simState.setRawRotorPosition(rotorPosition);
        Logger.recordOutput("Elevator/Sim/setRawRotorPosition", rotorPosition);

        double rotorVel = mechanismSim.getVelocity().baseUnitMagnitude()
                / ElevatorConstants.reduction;
        simState.setRotorVelocity(rotorVel);
        Logger.recordOutput("Elevator/Sim/SimulatorVelocityRadS", mechanismSim.getVelocity().baseUnitMagnitude());
    }
}
