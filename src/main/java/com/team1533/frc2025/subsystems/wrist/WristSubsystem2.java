package com.team1533.frc2025.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import com.team1533.frc2025.RobotState;
import com.team1533.frc2025.subsystems.wrist.WristConstants;
import com.team1533.lib.subsystems.CanCoderIO;
import com.team1533.lib.subsystems.CanCoderInputsAutoLogged;
import com.team1533.lib.subsystems.MotorIO;
import com.team1533.lib.subsystems.MotorInputsAutoLogged;
import com.team1533.lib.subsystems.MotorSubsystem;
import com.team1533.lib.subsystems.MotorSubsystemConfig;
import com.team1533.lib.subsystems.MotorSubsystemWithCanCoder;
import com.team1533.lib.subsystems.MotorSubsystemWithCanCoderConfig;
import com.team1533.lib.time.RobotTime;

import edu.wpi.first.math.util.Units;

public class WristSubsystem2
       extends MotorSubsystemWithCanCoder<
                MotorInputsAutoLogged, MotorIO, CanCoderInputsAutoLogged, CanCoderIO> {
    private RobotState state;

    public WristSubsystem2(
            MotorSubsystemWithCanCoderConfig c,
            MotorIO motorIO,
            CanCoderIO cancoderIO,
            RobotState state) {
        super(c, new MotorInputsAutoLogged(), motorIO, new CanCoderInputsAutoLogged(), cancoderIO);
        this.state = RobotState.getInstance();
        setDefaultCommand(dutyCycleCommand(() -> 0.0).withName("Wrist Maintain Setpoint (default)"));

        // Update frequency for feedback.
        cancoderIO.updateFrequency(500);
    }

    // Updates robot state with current wrist angle
    @Override
    public void periodic() {
        super.periodic();
        double timestamp = RobotTime.getTimestampSeconds();
        io.updateInputs(inputs);
        state.addWristUpdate(timestamp, inputs.unitPosition);
        // (
        //     //inputs.unitPosition
        //     timestamp, fastInputs.FusedCANcoderPositionRots
        // );
    }
}
