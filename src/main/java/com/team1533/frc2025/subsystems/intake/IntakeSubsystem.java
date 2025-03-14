package com.team1533.frc2025.subsystems.intake;

import com.team1533.lib.subsystems.MotorIO;
import com.team1533.lib.subsystems.MotorInputsAutoLogged;
import com.team1533.lib.subsystems.ServoMotorSubsystem;
import com.team1533.lib.subsystems.ServoMotorSubsystemConfig;

public class IntakeSubsystem extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {

    public IntakeSubsystem(ServoMotorSubsystemConfig c, final MotorIO io) {
        super(c, new MotorInputsAutoLogged(), io);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

}
