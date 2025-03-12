package com.team1533.frc2025.subsystems.intake;

import edu.wpi.first.hal.simulation.DIODataJNI;

public class IntakeSensorIOSim extends IntakeSensorIOReal {
    public IntakeSensorIOSim() {
        super();

    }

    public void setNotBlocked() {
        DIODataJNI.setValue(this.getIntakeLaser().getChannel(), false);
    }

    public void setBlocked() {
        DIODataJNI.setValue(this.getIntakeLaser().getChannel(), true);
    }
}
