package frc.robot.commands;

import edu.wpi.first.math.util.Units;

/**
 * Configuration for the flywheel auto-tune routine.
 *
 * <p>Motor hardware context: 3× Kraken X60 (FOC), 1.1667× up-speed gear ratio.
 * Mechanism max speed ≈ 5700 × 1.1667 ≈ 6650 RPM.
 *
 * @param maxVelocity High-speed tuning setpoint in subsystem units/sec. Use ~80% of mechanism
 *     max (≈5320 RPM for this system).
 * @param minVelocity Low-speed tuning setpoint in subsystem units/sec. Use ~10% of mechanism
 *     max (≈665 RPM for this system).
 * @param maxCurrentAmps Maximum current to apply during the open-loop kS ramp.
 * @param ksRampStepAmps Current increment per step when ramping to find kS.
 * @param ksRampDwellSeconds Dwell time (seconds) at each current step during kS ramp.
 * @param velocityThreshold Minimum velocity (subsystem units/sec) to consider the motor moving.
 * @param settleVarianceThresholdPct Peak-to-peak velocity variation as a fraction of the mean
 *     below which the motor is considered settled (e.g. 0.01 = 1%). Used by VALIDATE and FIND_KP
 *     phases where precision matters.
 * @param settleConfirmationSeconds Seconds the signal must remain below the variance threshold
 *     before convergence is declared.
 * @param minSettleWaitSeconds Mandatory minimum wait (seconds) before convergence can be
 *     declared, regardless of how stable the signal appears.
 * @param maxSettleWaitSeconds Safety-net maximum wait (seconds). Forces phase advancement if
 *     the signal never converges. Used by VALIDATE and FIND_KP phases.
 * @param tuneVarianceThresholdPct Relaxed variance threshold for TUNE_KV_HIGH and TUNE_KS_LOW
 *     phases where directional accuracy matters more than precision (e.g. 0.05 = 5%).
 * @param tuneMaxSettleWaitSeconds Max-wait for tuning phases. Per CTRE procedure, the motor must
 *     reach true steady state before measuring — the kV/kS transfer formula requires accurate
 *     steady-state error to converge in 2–3 iterations.
 * @param kpStartValue Initial kP for the proportional-gain search. CTRE recommendation:
 *     10 / setpoint_in_mechanism_rotations_per_second (NOT radians/s).
 * @param kpStepMultiplier Multiplier applied to kP each search iteration.
 * @param kpBackoffFactor Fraction of the oscillating kP to keep as the final value.
 * @param oscillationThreshold Peak-to-peak / setpoint ratio that indicates oscillation during
 *     kP search.
 * @param oscillationWindowSeconds Rolling window length (seconds) used for oscillation detection
 *     during kP search.
 * @param validationPoints Number of velocity setpoints to test during final validation.
 * @param phaseTimeoutSeconds Hard per-phase timeout. Sized generously to allow 30–60 s tests
 *     plus multiple adjustment iterations.
 * @param globalTimeoutSeconds Hard total-routine timeout.
 * @param runawayVelocityFactor Abort if velocity exceeds {@code maxVelocity × factor}.
 * @param ksKvIterations Maximum number of outer high-speed/low-speed alternation cycles when
 *     refining kV and kS together.
 * @param tTestAlpha Significance level for the Welch's t-test used by the convergence detector
 *     (e.g. 0.05 for a 95% confidence two-tailed test).
 */
public record FlywheelAutoTuneConfig(
        double maxVelocity,
        double minVelocity,
        double maxCurrentAmps,
        double ksRampStepAmps,
        double ksRampDwellSeconds,
        double velocityThreshold,
        double settleVarianceThresholdPct,
        double settleConfirmationSeconds,
        double minSettleWaitSeconds,
        double maxSettleWaitSeconds,
        double tuneVarianceThresholdPct,
        double tuneNoiseThreshold,
        double tuneMaxSettleWaitSeconds,
        double kpStartValue,
        double kpStepMultiplier,
        double kpBackoffFactor,
        double oscillationThreshold,
        double oscillationWindowSeconds,
        int validationPoints,
        double phaseTimeoutSeconds,
        double globalTimeoutSeconds,
        double runawayVelocityFactor,
        int ksKvIterations,
        double tTestAlpha) {

    /**
     * Default configuration for the Shooter subsystem.
     *
     * <p>Hardware: 3× Kraken X60 (FOC), 28:24 (1.1667×) up-speed gear.
     * Free-speed at 1:1 ≈ 5700 RPM → mechanism max ≈ 6650 RPM.
     */
    public static FlywheelAutoTuneConfig forFeeder() {
        double maxVelRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(7530 * 0.8 / 2); // 80% of 6650 RPM
        return new FlywheelAutoTuneConfig(
                maxVelRadPerSec,
                Units.rotationsPerMinuteToRadiansPerSecond(7530 * 0.1 / 2), // 10% of 6650 RPM
                70.0, // max current during kS ramp (below 80 A supply limit)
                0.05, // kS ramp step amps
                0.500, // kS ramp dwell seconds (longer for reliable detection)
                0.5, // velocity threshold rad/s
                0.01, // settle variance threshold: 1% peak-to-peak
                3.0, // settle confirmation seconds
                10.0, // minimum settle wait seconds
                120.0, // maximum settle wait seconds (VALIDATE / FIND_KP phases)
                0.05, // tune variance threshold: 2% p2p (must reach near-steady-state)
                20,
                60.0, // tune max settle wait seconds (reduced from 120 — motor settles in ~20-30s)
                10.0
                        / Units.radiansToRotations(
                                maxVelRadPerSec), // kP start ≈ 0.113 (CTRE: 10 / setpoint_in_mechanism_rps)
                1.5, // kP step multiplier
                0.70, // kP backoff factor (use 70% of the oscillating kP)
                0.05, // oscillation threshold (5% peak-to-peak / setpoint)
                5.0, // oscillation window seconds
                10, // validation points
                600.0, // phase timeout seconds (5 min per phase)
                1200.0, // global timeout seconds (20 min total)
                1.5, // runaway velocity factor (150% of maxVelocity)
                10, // kS/kV alternation iterations
                0.1); // t-test alpha (significance level)
    }

    /**
     * Default configuration for the Shooter subsystem.
     *
     * <p>Hardware: 3× Kraken X60 (FOC), 28:24 (1.1667×) up-speed gear.
     * Free-speed at 1:1 ≈ 5700 RPM → mechanism max ≈ 6650 RPM.
     */
    public static FlywheelAutoTuneConfig forShooter() {
        double maxVelRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(5320); // 80% of 6650 RPM
        return new FlywheelAutoTuneConfig(
                maxVelRadPerSec,
                Units.rotationsPerMinuteToRadiansPerSecond(665), // 10% of 6650 RPM
                70.0, // max current during kS ramp (below 80 A supply limit)
                0.1, // kS ramp step amps
                0.500, // kS ramp dwell seconds (longer for reliable detection)
                0.5, // velocity threshold rad/s
                0.01, // settle variance threshold: 1% peak-to-peak
                3.0, // settle confirmation seconds
                10.0, // minimum settle wait seconds
                120.0, // maximum settle wait seconds (VALIDATE / FIND_KP phases)
                0.05, // tune variance threshold: 2% p2p (must reach near-steady-state)
                7,
                60.0, // tune max settle wait seconds (reduced from 120 — motor settles in ~20-30s)
                10.0
                        / Units.radiansToRotations(
                                maxVelRadPerSec), // kP start ≈ 0.113 (CTRE: 10 / setpoint_in_mechanism_rps)
                1.5, // kP step multiplier
                0.70, // kP backoff factor (use 70% of the oscillating kP)
                0.05, // oscillation threshold (5% peak-to-peak / setpoint)
                5.0, // oscillation window seconds
                10, // validation points
                600.0, // phase timeout seconds (5 min per phase)
                1200.0, // global timeout seconds (20 min total)
                1.5, // runaway velocity factor (150% of maxVelocity)
                10, // kS/kV alternation iterations
                0.1); // t-test alpha (significance level)
    }

    /**
     * Simulation-tuned configuration for the Shooter subsystem.
     *
     * <p>Uses faster timing parameters suited to the sim's quick flywheel dynamics
     * (J=0.0027 kg·m², time constant &lt;1 s) and relaxed variance thresholds
     * for the low-friction sim environment.
     */
    public static FlywheelAutoTuneConfig forShooterSim() {
        double maxVelRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(5320);
        return new FlywheelAutoTuneConfig(
                maxVelRadPerSec,
                Units.rotationsPerMinuteToRadiansPerSecond(665),
                70.0, // max current during kS ramp
                0.1, // kS ramp step amps
                0.250, // kS ramp dwell seconds (sim responds faster)
                0.5, // velocity threshold rad/s
                0.05, // settle variance threshold: 1%
                1.5, // settle confirmation seconds (was 3.0)
                2.0, // minimum settle wait seconds (was 10.0)
                30.0, // maximum settle wait seconds (was 120.0)
                0.05, // tune variance threshold: 5% (sim has less damping)
                7,
                30.0, // tune max settle wait seconds
                10.0
                        / Units.radiansToRotations(
                                maxVelRadPerSec), // kP start ≈ 0.113 (CTRE: 10 / setpoint_in_mechanism_rps)
                1.5, // kP step multiplier
                0.70, // kP backoff factor
                0.05, // oscillation threshold
                3.0, // oscillation window seconds (was 5.0)
                5, // validation points
                300.0, // phase timeout seconds (5 points × 30s settle + margin)
                600.0, // global timeout seconds (was 300.0)
                1.5, // runaway velocity factor
                10, // kS/kV alternation iterations
                0.1); // t-test alpha (significance level)
    }
}
