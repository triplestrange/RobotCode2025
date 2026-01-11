package com.team1533.lib.subsystems;

import org.littletonrobotics.junction.Logger;

public class MotorSubsystemWithFollowersAndCanCoder<
                T extends MotorInputsAutoLogged,
                U extends MotorIO,
                V extends CanCoderInputsAutoLogged,
                W extends CanCoderIO>
        extends MotorSubsystem<T, U> {
    protected MotorSubsystemWithFollowersAndCanCoderConfig conf;
    protected T inputs;
    protected U io;
    protected V cancoderInputs;
    protected W cancoderIO;
    protected boolean hasSetOffset = false;

    public MotorSubsystemWithFollowersAndCanCoder(
            MotorSubsystemWithFollowersAndCanCoderConfig config,
            T inputs,
            U io,
            V cancoderInputs,
            W cancoder) {
        super(config, inputs, io);
        this.conf = config;
        // this.inputs = inputs;
        // this.io = io;
        this.cancoderInputs = cancoderInputs;
        this.cancoderIO = cancoder;
    }

    @Override
    public void periodic() {
        super.periodic();

        cancoderIO.updateInputs(cancoderInputs);
        Logger.processInputs(getName() + "/cancoder", cancoderInputs);

        if (!this.conf.isFusedCancoder
                && !this.hasSetOffset
                && !Double.isNaN(cancoderInputs.absolutePositionRotations)) {
            io.setCurrentPosition(
                    cancoderInputs.absolutePositionRotations * conf.cancoderToUnitsRatio);
            this.hasSetOffset = true;
        }
    }

    public void resetOffset() {
        // Don't set boolean above to left main thread still do it as well.
        io.setCurrentPosition(cancoderInputs.absolutePositionRotations * conf.cancoderToUnitsRatio);
    }
}
