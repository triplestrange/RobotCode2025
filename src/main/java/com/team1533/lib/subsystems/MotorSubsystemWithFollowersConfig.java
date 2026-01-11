package com.team1533.lib.subsystems;

public class MotorSubsystemWithFollowersConfig extends MotorSubsystemConfig {
    public static class FollowerConfig {
        public MotorSubsystemConfig config = new MotorSubsystemConfig();
        public boolean inverted = false;
    }

    public FollowerConfig[] followers = new FollowerConfig[] {};
}