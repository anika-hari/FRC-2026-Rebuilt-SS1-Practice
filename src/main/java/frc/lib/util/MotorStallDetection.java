package frc.lib.util;

public class MotorStallDetection {
    public static boolean isMotorStalled(
            double motorCurrent, double motorVelocity, double currentLimitAmps, double velocityLimitRPS) {
        return Math.abs(motorCurrent) > currentLimitAmps && Math.abs(motorVelocity) < velocityLimitRPS;
    }
}
