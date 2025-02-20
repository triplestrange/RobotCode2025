package com.team1533.frc2025.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1533.lib.util.CTREUtil;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX leftMotor = new TalonFX(10);
    private final TalonFX rightMotor = new TalonFX(0);
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0.0).withSlot(0);
    private final Follower rightFollower = new Follower(0, false);

    private final StatusSignal<Double> leftPositionSignal = leftMotor.getPosition();
    private final StatusSignal<Velocity> leftVelocitySignal = leftMotor.getVelocity();
    private final StatusSignal<Voltage> leftVoltsSignal = leftMotor.getMotorVoltage();
    private final StatusSignal<Current> leftCurrentStatorSignal = leftMotor.getStatorCurrent();
    private final StatusSignal<Current> leftCurrentSupplySignal = leftMotor.getSupplyCurrent();

    private final StatusSignal<Double> rightPositionSignal = rightMotor.getPosition();
    private final StatusSignal<Double> rightVelocitySignal = rightMotor.getVelocity();
    private final StatusSignal<Double> rightVoltsSignal = rightMotor.getMotorVoltage();
    private final StatusSignal<Current> rightCurrentStatorSignal = rightMotor.getStatorCurrent();
    private final StatusSignal<Current> rightCurrentSupplySignal = rightMotor.getSupplyCurrent();

    public double leftVelocityRadPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double leftCurrentAmps = 0.0;

    public double rightVelocityRadPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double rightCurrentAmps = 0.0;

    public double elevatorPosMeters = 0.0;
    public double secondsSinceReset = 0.0;

    public ElevatorIOReal() {
        var leftConfig = new TalonFXConfiguration();
        var rightConfig = new TalonFXConfiguration();

        leftConfig.Slot0.kP = 1;

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = motor.getPosition().getValueAsDouble();
        inputs.velocityMetersPerSecond = motor.getVelocity().getValueAsDouble();
        inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = new double[] { motor.getSupplyCurrent().getValueAsDouble() };
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setPosition(double positionMeters) {
        motor.setPosition(positionMeters);
    }
}
