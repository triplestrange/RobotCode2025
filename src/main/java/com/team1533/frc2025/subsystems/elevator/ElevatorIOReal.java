package com.team1533.frc2025.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOReal implements ElevatorIO {
    private final TalonFX motor = new TalonFX(10); 

    public ElevatorIOReal() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor.getConfigurator().apply(config);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = motor.getPosition().getValueAsDouble(); 
        inputs.velocityMetersPerSecond = motor.getVelocity().getValueAsDouble();
        inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = new double[]{motor.getSupplyCurrent().getValueAsDouble()};
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
