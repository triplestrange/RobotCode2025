package com.team1533.frc2025.subsystems.intake;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.team1533.frc2025.RobotState;
import com.team1533.lib.loops.IStatusSignalLoop;
import com.team1533.lib.subsystems.MotorIO;
import com.team1533.lib.subsystems.MotorInputsAutoLogged;
import com.team1533.lib.subsystems.ServoMotorSubsystem;
import com.team1533.lib.subsystems.ServoMotorSubsystemConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;

public class IntakeSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> implements IStatusSignalLoop {

    private IntakeSensorInputsAutoLogged inputsSensors = new IntakeSensorInputsAutoLogged();
    private IntakeSensorIO ioSensors;
    private AtomicBoolean laserHasReef = new AtomicBoolean(false);

    private final RobotState state;

    private double timeOfLastLaser = 0.0;
    private Debouncer laserDebouncer = new Debouncer(IntakeConstants.kIntakeLaserDebounceTime, DebounceType.kRising);

    public IntakeSubsystem(
            ServoMotorSubsystemConfig c,
            final MotorIO io, final IntakeSensorIO sensorIO) {
        super(c, new MotorInputsAutoLogged(), io);
        this.ioSensors = sensorIO;
        this.state = RobotState.getInstance();
    }

    @Override
    public void periodic() {
        super.periodic();
        ioSensors.updateInputs(inputsSensors);
        inputsSensors.intakeLaserBlocked = hasReefAtIntake();
        state.updateLastTriggeredIntakeLaserTimestamp(inputsSensors.intakeLaserBlocked);
        Logger.recordOutput("Intake/lastTriggeredLaserTimestamp", state.getLastTriggeredIntakeLaserTimestamp());
        Logger.processInputs(getName() + "/sensors", inputsSensors);
        if (inputsSensors.intakeLaserBlocked) {
            timeOfLastLaser = Timer.getFPGATimestamp();
        }
    }

    public void setTeleopDefaultCommand() {
        setDefaultCommand(dutyCycleCommand(() -> 0.0).withName("Zero intake Duty Cycle"));
    }

    public boolean hasReefAtIntake() {
        return laserHasReef.get();
    }

    public boolean hasReefAtIntake(double secondsAgo) {
        return Timer.getFPGATimestamp() - secondsAgo <= timeOfLastLaser;
    }

    @Override
    public List<BaseStatusSignal> getStatusSignals() {
        return new ArrayList<>();
    }

    @Override
    public void onLoop() {
        laserHasReef.set(laserDebouncer.calculate(ioSensors.getIntakeLaser().get()));
    }
}
