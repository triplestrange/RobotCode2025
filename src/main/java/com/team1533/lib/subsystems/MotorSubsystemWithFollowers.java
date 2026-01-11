package com.team1533.lib.subsystems;

import org.littletonrobotics.junction.Logger;

public class MotorSubsystemWithFollowers<T extends MotorInputsAutoLogged, U extends MotorIO>
        extends MotorSubsystem<T, U> {
    protected MotorSubsystemWithFollowersConfig leaderConfig;
    protected MotorSubsystemWithFollowersConfig.FollowerConfig[] followerConfigs;
    protected T[] followerInputs;
    protected U[] followerIos;

    public MotorSubsystemWithFollowers(
            MotorSubsystemWithFollowersConfig leaderConfig,
            T leaderInputs,
            U leaderIo,
            T[] followerInputs,
            U[] followerIo) {
        super(leaderConfig, leaderInputs, leaderIo);
        this.leaderConfig = leaderConfig;
        this.followerConfigs = leaderConfig.followers;
        this.followerInputs = followerInputs;
        this.followerIos = followerIo;
        assert followerInputs.length == followerIo.length
                : "Length of follower inputs/io not equal";
        for (int i = 0; i < followerConfigs.length; i++) {
            MotorIO followerIO = followerIo[i];
            followerIO.follow(leaderConfig.talonCANID, followerConfigs[i].inverted);
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        for (int i = 0; i < followerConfigs.length; i++) {
            followerIos[i].updateInputs(followerInputs[i]);
            Logger.processInputs(getName() + "/follower" + i, followerInputs[i]);
        }
    }

    @Override
    protected void setCurrentPositionAsZero() {
        super.setCurrentPositionAsZero();
        for (var follower : followerIos) {
            follower.setCurrentPositionAsZero();
        }
    }

    @Override
    public void setCurrentPosition(double positionUnits) {
        super.setCurrentPosition(positionUnits);
        for (var follower : followerIos) {
            follower.setCurrentPosition(positionUnits);
        }
    }

    @Override
    public double getCurrentPosition() {
        double averagePosition = inputs.unitPosition;
        for (var followerInput : followerInputs) {
            averagePosition += followerInput.unitPosition;
        }
        return averagePosition / (followerConfigs.length + 1);
    }

    @Override
    public double getCurrentVelocity() {
        double averagePosition = inputs.velocityUnitsPerSecond;
        for (var followerInput : followerInputs) {
            averagePosition += followerInput.velocityUnitsPerSecond;
        }
        return averagePosition / (followerConfigs.length + 1);
    }
}