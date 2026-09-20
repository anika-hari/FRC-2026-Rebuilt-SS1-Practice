package frc.lib.subsystems.real;

public class ServoMotorSubsystemWithFollowersConfig extends ServoMotorSubsystemConfig {
    public static class FollowerConfig extends ServoMotorSubsystemConfig {
        public boolean inverted = false;
    }

    public FollowerConfig[] followers = new FollowerConfig[] {};
}
