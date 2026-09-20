package frc.lib.subsystems.real;

public class FlywheelSubsystemConfig extends ServoMotorSubsystemWithFollowersConfig {
    /** Velocity tolerance for isAtSetpoint() in subsystem units/sec. */
    public double velocitySetpointTolerance = 60.0;

    // only one should be true at a time on the robot, and it should only be true when tuning.
    public boolean useAutoTuneThread = false;

    /** Returns the total motor count (leader + followers) for simulation. */
    public int getTotalMotorCount() {
        return 1 + (followers != null ? followers.length : 0);
    }

    public static class FlywheelFollowerConfig extends FollowerConfig {
        /** Velocity tolerance for isAtSetpoint() in subsystem units/sec. */
        public double velocitySetpointTolerance = 60.0;

        // only one should be true at a time on the robot, and it should only be true when tuning.
        public boolean useAutoTuneThread = false;
    }

    {
        super.followers = new FlywheelFollowerConfig[] {};
    }
}
