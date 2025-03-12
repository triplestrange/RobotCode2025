package com.team1533.frc2025.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.wpilibj.DigitalInput;

public interface IntakeSensorIO {
    @AutoLog
    public class IntakeSensorInputs {
        public boolean intakeLaserBlocked;
    }

    default void updateInputs(IntakeSensorIO.IntakeSensorInputs inputs) {

    }

    default DigitalInput getIntakeLaser() {
        return null;
    };
}
