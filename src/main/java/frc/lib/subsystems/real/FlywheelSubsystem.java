package frc.lib.subsystems.real;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.subsystems.FlywheelIO;
import frc.robot.commands.FlywheelAutoTunePhoenixThread;
import java.util.Arrays;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

/**
 * Base subsystem for flywheel mechanisms driven by one or more motors.
 * Uses WPILib's FlywheelSim for accurate multi-motor simulation when
 * constructed via IOFactory.flywheelLeaderMotor/flywheelFollowerMotor.
 */
public class FlywheelSubsystem<U extends FlywheelIO> extends ServoMotorSubsystemWithFollowers<U> {
    protected final FlywheelSubsystemConfig flywheelConfig;
    protected double[] velocities = new double[] {};
    protected double[] timestamps = new double[] {};
    protected double smoothedVelocity;
    // inputs from FlywheelAutoTunePhoenix thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> velocityQueue;

    public FlywheelSubsystem(FlywheelSubsystemConfig config, U leaderIo, U[] followerIo) {
        super(config, leaderIo, followerIo);
        System.out.println("========" + config.followers[0].name + "INIT FlywheelSubsystem" + "===============");

        this.flywheelConfig = config;
        if (flywheelConfig.useAutoTuneThread) {
            timestampQueue = FlywheelAutoTunePhoenixThread.getInstance(this).makeTimestampQueue();
            velocityQueue = FlywheelAutoTunePhoenixThread.getInstance(this).registerSignal(getVelocitySignal());
            FlywheelAutoTunePhoenixThread.getInstance(this).start();
        } else {
            timestampQueue = null;
            velocityQueue = null;
        }
    }

    @Override
    public void periodic() {
        super.periodic();

        if (velocityQueue != null && timestampQueue != null) {
            velocityLock.lock();
            inputs.setTimestamps(
                    timestampQueue.stream().mapToDouble((Double val) -> val).toArray());
            inputs.setVelocities(velocityQueue.stream()
                    .mapToDouble((Double val) -> conf.unitToRotorRatio * val)
                    .toArray());
            timestampQueue.clear();
            velocityQueue.clear();
            timestamps = inputs.getTimestamps();
            velocities = inputs.getVelocities();
            velocityLock.unlock();
        }
        OptionalDouble smoothedVelocityOpt = Arrays.stream(velocities).average();
        smoothedVelocity = smoothedVelocityOpt.orElseGet(() -> 0);
        Logger.recordOutput(getName() + "/smoothedVelocityRadiansPerSecond", smoothedVelocity);
    }

    /**
     * Returns the leader motor's velocity only. All motors share one flywheel,
     * so the leader (which drives the FlywheelSim) is the ground truth.
     * Averaging with followers introduces timing-induced discrepancies in sim.
     */
    @Override
    public double getVelocity() {
        return inputs.getVelocity();
    }

    public double[] getVelocityBuffer() {
        return inputs.getVelocities();
    }

    public double getSmoothedVelocity() {
        return smoothedVelocity;
    }

    public double[] getTimestamps() {
        return inputs.getTimestamps();
    }

    public StatusSignal<AngularVelocity> getVelocitySignal() {
        return io.getVelocitySignal();
    }

    /**
     * Returns true if the current velocity is within tolerance of the velocity setpoint.
     */
    public boolean isAtSetpoint() {
        return Math.abs(getVelocitySetpointUnitsPerSecond() - getVelocity()) < flywheelConfig.velocitySetpointTolerance;
    }

    public final Lock velocityLock = new ReentrantLock();
}
