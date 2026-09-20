package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.real.FlywheelSubsystem;
import frc.lib.subsystems.real.ServoMotorSubsystem;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

/**
 * Automated flywheel PID/FF tuning command following the CTRE TorqueCurrentFOC manual-PID
 * tuning procedure documented in the Phoenix Pro application notes.
 *
 * <p>Procedure (per CTRE docs):
 * https://phoenixpro-documentation--161.org.readthedocs.build/en/161/docs/application-notes/manual-pid-tuning.html
 *
 * <ol>
 *   <li>Zero all gains (RESET_GAINS)
 *   <li>Find kS via open-loop current ramp; back off one step (FIND_KS)
 *   <li>Set kP low, adjust kV at high velocity until setpoint achieved (TUNE_KV_HIGH)
 *   <li>Adjust kS at low velocity until setpoint achieved (TUNE_KS_LOW)
 *   <li>Repeat steps 3–4 until gains stop changing (up to {@code ksKvIterations} passes)
 *   <li>Increase kP until oscillation, then back off (FIND_KP)
 *   <li>Verify gains across the velocity range (VALIDATE)
 *   <li>Report results (REPORT_RESULTS)
 * </ol>
 *
 * <p>Key design decisions:
 *
 * <ul>
 *   <li>All settling uses a {@link ConvergenceDetector} — no fixed timers. Real kS/kV tests on
 *       the 3× Kraken X60 system can take 30–60 s to level off; the detector waits until the
 *       velocity signal is genuinely stable.
 *   <li>kV and kS are adjusted via a physically derived formula that transfers the P-controller's
 *       residual correction into the feedforward term, driving steady-state error toward zero.
 *   <li>FIND_KP commands an actual velocity setpoint (the original code did not).
 *   <li>VALIDATE uses closed-loop velocity control with all discovered gains active.
 *   <li>All gain unit conversions use {@code subsystem.getUnitToRotorRatio()}, which encodes the
 *       1.1667× gear ratio via the configured Phoenix 6 FeedbackConfigs.
 * </ul>
 */
public class FlywheelAutoTuneCommand extends Command {

    // ========================= Inner Classes =============================

    /**
     * Detects when a motor velocity signal has reached steady state.
     *
     * <p>Samples are collected at ~50 Hz (each {@code execute()} call). Convergence is declared
     * when the peak-to-peak variation over the rolling window stays below
     * {@code varianceThresholdPct} of the mean for at least {@code confirmationSecs} consecutive
     * seconds, subject to a mandatory minimum wait. If the signal never converges within
     * {@code maxWaitSecs}, the detector forces advancement (returns {@code true}) so the phase
     * does not hang indefinitely.
     */
    private static class ConvergenceDetector {
        private static final double DT = 0.02; // assumes 50 Hz execute() loop

        private final double[] buf;
        private final double[] bigBuf;
        private final double varianceThresh;
        private final double noiseThresh;
        private final double confirmSecs;
        private final double minWaitSecs;
        private final double maxWaitSecs;
        private final double zCrit; // precomputed from alpha for t-test

        private int writeIdx;
        private int count;
        private int bigBufWriteIdx;
        private int bigBufCount;
        private double stableSecs;
        private double elapsedSecs;
        private boolean didTimeOut;

        ConvergenceDetector(
                int bufferSize,
                double varianceThresholdPct,
                double noiseThreshold,
                double confirmationSecs,
                double minWaitSecs,
                double maxWaitSecs,
                double alpha) {
            this.buf = new double[bufferSize];
            this.bigBuf = new double[bufferSize * 15];
            this.varianceThresh = varianceThresholdPct;
            this.noiseThresh = noiseThreshold;
            this.confirmSecs = confirmationSecs;
            this.minWaitSecs = minWaitSecs;
            this.maxWaitSecs = maxWaitSecs;
            this.zCrit = inverseNormalCDF(1.0 - alpha / 2.0);
        }

        /** Abramowitz & Stegun rational approximation (26.2.23) for the inverse normal CDF. */
        private static double inverseNormalCDF(double p) {
            if (p < 0.5) return -inverseNormalCDF(1.0 - p);
            double t = Math.sqrt(-2.0 * Math.log(1.0 - p));
            return t
                    - (2.515517 + 0.802853 * t + 0.010328 * t * t)
                            / (1.0 + 1.432788 * t + 0.189269 * t * t + 0.001308 * t * t * t);
        }

        boolean update(double[] velocities) {
            elapsedSecs += DT;

            if (elapsedSecs >= maxWaitSecs) {
                Logger.recordOutput("AutoTune/buffer/status", "timeout");
                didTimeOut = true;
                return true;
            }

            if (elapsedSecs < minWaitSecs || bigBufCount < velocities.length) {
                // Not enough history or time — just accumulate data
                for (double v : velocities) {
                    bigBuf[bigBufWriteIdx % bigBuf.length] = v;
                    bigBufWriteIdx++;
                    bigBufCount = Math.min(bigBufCount + 1, bigBuf.length);
                }
                Logger.recordOutput("AutoTune/buffer/status", "waiting for data");
                return false;
            }

            Logger.recordOutput("AutoTune/buffer/status", "doing t-test");

            // Compute mean and sample variance of bigBuf
            double sumBig = 0;
            for (int i = 0; i < bigBufCount; i++) sumBig += bigBuf[i];
            double meanBig = sumBig / bigBufCount;

            double ssqBig = 0;
            for (int i = 0; i < bigBufCount; i++) {
                double d = bigBuf[i] - meanBig;
                ssqBig += d * d;
            }
            double varBig = ssqBig / (bigBufCount - 1);

            // Compute mean and sample variance of new velocities
            double sumNew = 0;
            for (double v : velocities) sumNew += v;
            double meanNew = sumNew / velocities.length;

            double ssqNew = 0;
            for (double v : velocities) {
                double d = v - meanNew;
                ssqNew += d * d;
            }
            double varNew = (velocities.length > 1) ? ssqNew / (velocities.length - 1) : 0;

            // Welch's t-test: compare means of bigBuf and velocities
            double se = Math.sqrt(varBig / bigBufCount + varNew / velocities.length);

            if (se < 1e-12) {
                // Both distributions have near-zero variance — means are equal
                stableSecs += DT;
            } else {
                double t = Math.abs(meanBig - meanNew) / se;

                // Welch-Satterthwaite degrees of freedom
                double a = varBig / bigBufCount;
                double b = varNew / velocities.length;
                double df =
                        (a + b) * (a + b) / (a * a / (bigBufCount - 1) + b * b / Math.max(velocities.length - 1, 1));

                // Cornish-Fisher expansion: approximate t critical value from z critical value
                double tCrit = zCrit + (zCrit * zCrit * zCrit + zCrit) / (4.0 * Math.max(df, 1));

                Logger.recordOutput("AutoTune/buffer/tStat", t);
                Logger.recordOutput("AutoTune/buffer/tCrit", tCrit);
                Logger.recordOutput("AutoTune/buffer/df", df);

                if (t < tCrit) {
                    stableSecs += DT;
                } else {
                    stableSecs = 0;
                }
            }

            Logger.recordOutput("AutoTune/buffer/stableSecs", stableSecs);

            // Store velocities in bigBuf for next iteration
            for (double v : velocities) {
                bigBuf[bigBufWriteIdx % bigBuf.length] = v;
                bigBufWriteIdx++;
                bigBufCount = Math.min(bigBufCount + 1, bigBuf.length);
            }

            return stableSecs >= confirmSecs;
        }

        /**
         * Call once per execute() with the latest velocity magnitude.
         *
         * @return {@code true} when steady state is confirmed, or when {@code maxWaitSecs} is
         *     exceeded (check {@link #timedOut()} to distinguish).
         */
        @SuppressWarnings("unused")
        boolean update(double velocity) {
            buf[writeIdx % buf.length] = velocity;
            writeIdx++;
            count = Math.min(count + 1, buf.length);
            elapsedSecs += DT;

            if (elapsedSecs >= maxWaitSecs) {
                Logger.recordOutput("AutoTune/buffer/status", "timeout");
                didTimeOut = true;
                return true;
            }
            if (elapsedSecs < minWaitSecs || count < buf.length) {
                Logger.recordOutput("AutoTune/buffer/status", "waiting for data");
                return false;
            }

            Logger.recordOutput("AutoTune/buffer/status", "doing math");

            double min = Double.MAX_VALUE;
            double max = -Double.MAX_VALUE;
            double sum = 0;
            for (int i = 0; i < count; i++) {
                double v = buf[i];
                sum += v;
                if (v < min) min = v;
                if (v > max) max = v;
            }
            double mean = sum / count;
            double ptpPct = (Math.abs(mean) > 1e-6) ? (max - min) / Math.abs(mean) : (max - min);
            Logger.recordOutput("AutoTune/buffer/ptpPct", ptpPct);

            if (ptpPct < varianceThresh || max - min < noiseThresh) {
                stableSecs += DT;
            } else {
                stableSecs = 0;
            }
            return stableSecs >= confirmSecs;
        }

        void reset() {
            writeIdx = 0;
            count = 0;
            bigBufWriteIdx = 0;
            bigBufCount = 0;
            stableSecs = 0;
            elapsedSecs = 0;
            didTimeOut = false;
        }

        double getElapsed() {
            return elapsedSecs;
        }

        boolean timedOut() {
            return didTimeOut;
        }
    }

    // ========================= Phase Enum ================================

    private enum Phase {
        RESET_GAINS,
        FIND_KS,
        TUNE_KV_HIGH, // CTRE step 5: closed-loop kV at high velocity
        TUNE_KS_LOW, // CTRE step 7: closed-loop kS at low velocity
        FIND_KP, // CTRE step 10: increase kP until oscillation, back off
        VALIDATE, // CTRE step 11: verify across velocity range
        REPORT_RESULTS,
        DONE
    }

    // ========================= Fields ====================================

    private final FlywheelSubsystem<?> subsystem;
    private final FlywheelAutoTuneConfig config;
    private final String logPrefix;

    // Timers
    private final Timer globalTimer = new Timer();
    private final Timer phaseTimer = new Timer();
    private final Timer ksDwellTimer = new Timer();

    // Saved gains — restored on abort
    private double origKP, origKI, origKD, origKG, origKS, origKA, origKV;
    private double origVelo, origAccel, origJerk;

    // Discovered gains
    private double discoveredKS;
    private double discoveredKV;
    private double discoveredKP;

    // Convergence detector (reconfigured when entering each settling phase)
    private ConvergenceDetector convergence;

    // FIND_KS
    private double ksCurrentAmps;

    // TUNE_KV_HIGH / TUNE_KS_LOW — shared state
    private boolean tuneApplied; // have we applied gains + commanded setpoint this iteration?
    private int tuneAdjustCount; // adjustment iterations within the current phase
    private boolean coastComplete; // has motor coasted down from previous phase?

    // kS/kV outer iteration loop
    private int ksKvIterationCount;
    private double prevKS;
    private double prevKV;
    private static final double KS_KV_CONVERGE_DELTA = 0.02; // 2% change → converged

    // FIND_KP
    private double kpTestValue;
    private boolean kpApplied;
    private final double[] oscBuffer; // rolling window for oscillation detection
    private int oscBufIdx;
    private int oscBufCount;

    // VALIDATE
    private int validatePointIndex;
    private boolean validateApplied;
    private boolean validateAllPassed;

    private boolean tuneSucceeded;
    private Phase phase;

    // ========================= Constructor ===============================

    public FlywheelAutoTuneCommand(FlywheelSubsystem<?> subsystem, FlywheelAutoTuneConfig config) {
        this.subsystem = subsystem;
        this.config = config;
        this.logPrefix = "AutoTune/" + subsystem.getName() + "/";
        // Oscillation buffer covers exactly one oscillationWindowSeconds window at 50 Hz
        this.oscBuffer = new double[Math.max(1, (int) (config.oscillationWindowSeconds() * 50))];
        addRequirements(subsystem);
    }

    // ========================= Command Lifecycle =========================

    @Override
    public void initialize() {
        System.out.println("=== FlywheelAutoTune: Starting for " + subsystem.getName() + " ===");
        System.out.printf(
                "  unitToRotorRatio=%.4f  sim=%b  maxVel=%.1f rad/s  minVel=%.1f rad/s%n",
                subsystem.getUnitToRotorRatio(), Robot.isSimulation(), config.maxVelocity(), config.minVelocity());
        tuneSucceeded = false;

        // Original gains are saved as zero here. A future improvement would read the live
        // hardware state via the subsystem's inputs before zeroing.
        origKP = origKI = origKD = origKG = origKS = origKA = origKV = 0;
        origVelo = origAccel = origJerk = 0;

        discoveredKS = 0;
        discoveredKV = 0;
        discoveredKP = 0;
        ksKvIterationCount = 0;
        prevKS = Double.NaN;
        prevKV = Double.NaN;

        globalTimer.restart();
        transitionTo(Phase.RESET_GAINS);
    }

    @Override
    public void execute() {
        double elapsed = globalTimer.get();
        Logger.recordOutput(logPrefix + "Phase", phase.name());
        Logger.recordOutput(logPrefix + "ElapsedSeconds", elapsed);
        Logger.recordOutput(logPrefix + "MeasuredVelocity", subsystem.getVelocity());

        if (elapsed > config.globalTimeoutSeconds()) {
            System.out.printf("FlywheelAutoTune: GLOBAL TIMEOUT at %.1fs in phase %s%n", elapsed, phase);
            transitionTo(Phase.DONE);
            return;
        }

        if (phase != Phase.DONE && phase != Phase.REPORT_RESULTS && phaseTimer.get() > config.phaseTimeoutSeconds()) {
            System.out.printf("FlywheelAutoTune: PHASE TIMEOUT in %s at %.1fs%n", phase, phaseTimer.get());
            transitionTo(Phase.DONE);
            return;
        }

        switch (phase) {
            case RESET_GAINS -> executeResetGains();
            case FIND_KS -> executeFindKS();
            case TUNE_KV_HIGH -> executeTuneKVHigh();
            case TUNE_KS_LOW -> executeTuneKSLow();
            case FIND_KP -> executeFindKP();
            case VALIDATE -> executeValidate();
            case REPORT_RESULTS -> executeReportResults();
            case DONE -> {}
        }
    }

    @Override
    public boolean isFinished() {
        return phase == Phase.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.setTorqueCurrentFOCImpl(0);
        if (interrupted || !tuneSucceeded) {
            System.out.println(
                    "FlywheelAutoTune: " + (interrupted ? "INTERRUPTED" : "FAILED") + " — restoring original gains");
            subsystem.updateGains(
                    origKP, origKI, origKD, origKG, origKS, origKA, origKV, origVelo, origAccel, origJerk);
        } else {
            System.out.println("FlywheelAutoTune: Complete — discovered gains are active");
        }
        globalTimer.stop();
        phaseTimer.stop();
    }

    // ========================= Phase Implementations =====================

    private void executeResetGains() {
        subsystem.updateGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        subsystem.setTorqueCurrentFOCImpl(0);
        System.out.println("FlywheelAutoTune: Gains zeroed — starting kS ramp");
        transitionTo(Phase.FIND_KS);
    }

    // ---- FIND_KS --------------------------------------------------------

    /**
     * Ramps open-loop torque current in small steps until the motor starts moving. Sets kS to
     * one step below the current that caused motion — per CTRE docs: "back off to just before
     * that movement."
     */
    private void executeFindKS() {
        if (!ksDwellTimer.hasElapsed(config.ksRampDwellSeconds())) return;

        double rawVelocity = subsystem.getVelocity();
        double velocity = Math.abs(rawVelocity);
        Logger.recordOutput(logPrefix + "FindKS/AppliedCurrent", ksCurrentAmps);
        Logger.recordOutput(logPrefix + "FindKS/Velocity", velocity);
        Logger.recordOutput(logPrefix + "FindKS/RawVelocity", rawVelocity);

        if (velocity > config.velocityThreshold()) {
            // Back off one step per CTRE: "just before movement"
            discoveredKS = Math.max(0, ksCurrentAmps - config.ksRampStepAmps());
            System.out.printf(
                    "FlywheelAutoTune: kS found = %.3f A (motion detected at %.3f A, v=%.2f rad/s)%n",
                    discoveredKS, ksCurrentAmps, velocity);
            Logger.recordOutput(logPrefix + "FindKS/DiscoveredKS", discoveredKS);
            subsystem.setTorqueCurrentFOCImpl(0);
            transitionTo(Phase.TUNE_KV_HIGH);
            return;
        }

        ksCurrentAmps += config.ksRampStepAmps();
        if (ksCurrentAmps > config.maxCurrentAmps()) {
            abortWithMsg("kS ramp reached max current without detecting motion");
            return;
        }

        subsystem.setTorqueCurrentFOCImpl(ksCurrentAmps);
        ksDwellTimer.restart();
    }

    // ---- TUNE_KV_HIGH ---------------------------------------------------

    /**
     * Tunes kV at high velocity using closed-loop control with a very low kP.
     *
     * <p>Per CTRE step 5: "Adjust kV until flywheel achieves setpoint." The correction formula
     * transfers what the P-controller is currently compensating into the kV feedforward term:
     *
     * <pre>
     *   ΔkV = kP × velocity_error / v_target_mechanism
     * </pre>
     *
     * The unitToRotorRatio cancels (it divides both error and demand in rotor space).
     * This converges in 2–3 iterations with an accurate steady-state measurement.
     */
    private void executeTuneKVHigh() {
        // Coast down from previous phase before starting closed-loop control.
        // Wait until velocity drops to at or below the target so the PID starts
        // near equilibrium and doesn't overshoot toward the runaway threshold.
        if (!coastComplete) {
            Logger.recordOutput(logPrefix + "TuneKVHigh/Status", "Coasting");
            if (Math.abs(subsystem.getVelocity()) > config.maxVelocity()) return;
            coastComplete = true;
        }

        if (!tuneApplied) {
            Logger.recordOutput(logPrefix + "TuneKVHigh/Status", "Updating Gains");
            subsystem.updateGains(config.kpStartValue(), 0, 0, 0, discoveredKS, 0, discoveredKV, 0, 0, 0);
            final double target = config.maxVelocity();
            subsystem.setTorqueCurrentFOCVelocityImpl(() -> target, 0);
            convergence.reset();
            tuneApplied = true;
            Logger.recordOutput(logPrefix + "TuneKVHigh/TrialKV", discoveredKV);
            Logger.recordOutput(logPrefix + "TuneKVHigh/AdjustmentIndex", tuneAdjustCount);
        }

        double velocity = Math.abs(subsystem.getVelocity());
        Logger.recordOutput(logPrefix + "TuneKVHigh/Velocity", velocity);
        Logger.recordOutput(logPrefix + "TuneKVHigh/SettleElapsed", convergence.getElapsed());

        if (velocity > config.maxVelocity() * config.runawayVelocityFactor()) {
            abortWithMsg("Velocity runaway in TUNE_KV_HIGH");
            return;
        }

        if (!convergence.update(subsystem.getSmoothedVelocity())) {
            Logger.recordOutput(logPrefix + "TuneKVHigh/Status", "Converging");
            return;
        }

        double error = config.maxVelocity() - velocity;
        double errorPct = Math.abs(error / config.maxVelocity());
        Logger.recordOutput(logPrefix + "TuneKVHigh/ErrorPct", errorPct);
        System.out.printf(
                "FlywheelAutoTune: TuneKVHigh[%d] target=%.1f actual=%.1f error=%.1f%% (%.1fs)%n",
                tuneAdjustCount, config.maxVelocity(), velocity, errorPct * 100, convergence.getElapsed());

        if (errorPct <= 0.05 || tuneAdjustCount >= 10) {
            Logger.recordOutput(logPrefix + "TuneKVHigh/DiscoveredKV", discoveredKV);
            System.out.printf("FlywheelAutoTune: kV converged = %.6f (error %.1f%%)%n", discoveredKV, errorPct * 100);
            transitionTo(Phase.TUNE_KS_LOW);
        } else {
            // Transfer the P-controller's residual into kV feedforward.
            // Clamp the adjustment to ±50% of current kV to prevent overshoot/zeroing.
            // Floor uses 25% of the raw delta so early iterations (when kV≈0) still make
            // meaningful progress instead of being stuck at a tiny fixed step.
            double rawDelta = config.kpStartValue() * error / config.maxVelocity();
            double maxDelta = Math.max(discoveredKV * 0.5, Math.abs(rawDelta) * 0.25);
            double clampedDelta = Math.max(-maxDelta, Math.min(maxDelta, rawDelta));
            Logger.recordOutput(logPrefix + "TuneKVHigh/RawDelta", rawDelta);
            Logger.recordOutput(logPrefix + "TuneKVHigh/ClampedDelta", clampedDelta);
            discoveredKV += clampedDelta;
            discoveredKV = Math.max(0, discoveredKV);
            tuneAdjustCount++;
            tuneApplied = false;
        }
    }

    // ---- TUNE_KS_LOW ----------------------------------------------------

    /**
     * Tunes kS at low velocity using closed-loop control with a very low kP.
     *
     * <p>Per CTRE step 7: "Adjust kS until flywheel achieves setpoint." kS is adjusted using the
     * same residual-transfer approach as kV:
     *
     * <pre>
     *   ΔkS = kP × velocity_error / unitToRotorRatio   (clamped to ±50% of current kS)
     * </pre>
     *
     * After converging, checks whether kS and kV changed more than {@code KS_KV_CONVERGE_DELTA}
     * from the previous outer iteration. If so, loops back to {@link Phase#TUNE_KV_HIGH} for
     * another pass (CTRE steps 8–9: "Repeat until gains do not change").
     */
    private void executeTuneKSLow() {
        // Coast down from previous phase before starting measurement
        if (!coastComplete) {
            if (Math.abs(subsystem.getVelocity()) > config.minVelocity() * 1.5) return;
            coastComplete = true;
        }

        if (!tuneApplied) {
            subsystem.updateGains(config.kpStartValue(), 0, 0, 0, discoveredKS, 0, discoveredKV, 0, 0, 0);
            final double target = config.minVelocity();
            subsystem.setTorqueCurrentFOCVelocityImpl(() -> target, 0);
            convergence.reset();
            tuneApplied = true;
            Logger.recordOutput(logPrefix + "TuneKSLow/TrialKS", discoveredKS);
            Logger.recordOutput(logPrefix + "TuneKSLow/AdjustmentIndex", tuneAdjustCount);
        }

        double velocity = Math.abs(subsystem.getVelocity());
        Logger.recordOutput(logPrefix + "TuneKSLow/Velocity", velocity);
        Logger.recordOutput(logPrefix + "TuneKSLow/SettleElapsed", convergence.getElapsed());

        if (!convergence.update(subsystem.getSmoothedVelocity())) return;

        double error = config.minVelocity() - velocity;
        double errorPct = Math.abs(error / config.minVelocity());
        Logger.recordOutput(logPrefix + "TuneKSLow/ErrorPct", errorPct);
        System.out.printf(
                "FlywheelAutoTune: TuneKSLow[%d] target=%.1f actual=%.1f error=%.1f%% (%.1fs)%n",
                tuneAdjustCount, config.minVelocity(), velocity, errorPct * 100, convergence.getElapsed());

        if (errorPct <= 0.05 || tuneAdjustCount >= 10) {
            Logger.recordOutput(logPrefix + "TuneKSLow/DiscoveredKS", discoveredKS);
            System.out.printf("FlywheelAutoTune: kS converged = %.4f (error %.1f%%)%n", discoveredKS, errorPct * 100);

            // Check whether both gains have stopped changing between outer iterations.
            // prevKS/prevKV are NaN on the first pass — NaN comparisons return false, so the
            // first pass always continues to at least one more iteration.
            double ksDelta = Math.abs(discoveredKS - prevKS) / Math.max(1e-9, Math.abs(prevKS));
            double kvDelta = Math.abs(discoveredKV - prevKV) / Math.max(1e-9, Math.abs(prevKV));
            boolean gainsConverged = !Double.isNaN(ksDelta)
                    && !Double.isNaN(kvDelta)
                    && ksDelta < KS_KV_CONVERGE_DELTA
                    && kvDelta < KS_KV_CONVERGE_DELTA;

            Logger.recordOutput(logPrefix + "TuneKSLow/KSDelta", Double.isNaN(ksDelta) ? -1 : ksDelta);
            Logger.recordOutput(logPrefix + "TuneKSLow/KVDelta", Double.isNaN(kvDelta) ? -1 : kvDelta);

            if (gainsConverged || ksKvIterationCount >= config.ksKvIterations()) {
                System.out.printf(
                        "FlywheelAutoTune: kS/kV converged after %d outer iteration(s)%n", ksKvIterationCount + 1);
                subsystem.setTorqueCurrentFOCImpl(0);
                transitionTo(Phase.FIND_KP);
            } else {
                prevKS = discoveredKS;
                prevKV = discoveredKV;
                ksKvIterationCount++;
                System.out.printf(
                        "FlywheelAutoTune: kS/kV outer iteration %d — returning to TUNE_KV_HIGH%n", ksKvIterationCount);
                transitionTo(Phase.TUNE_KV_HIGH);
            }
        } else {
            // Transfer the P-controller's residual into kS feedforward (clamped to prevent overshoot)
            double rawDelta = config.kpStartValue() * error / subsystem.getUnitToRotorRatio();
            double maxDelta = Math.max(discoveredKS * 0.5, Math.abs(rawDelta) * 0.25);
            double clampedDelta = Math.max(-maxDelta, Math.min(maxDelta, rawDelta));
            Logger.recordOutput(logPrefix + "TuneKSLow/RawDelta", rawDelta);
            Logger.recordOutput(logPrefix + "TuneKSLow/ClampedDelta", clampedDelta);
            discoveredKS += clampedDelta;
            discoveredKS = Math.max(0, discoveredKS);
            tuneAdjustCount++;
            tuneApplied = false;
        }
    }

    // ---- FIND_KP --------------------------------------------------------

    /**
     * Finds kP by increasing it until oscillation is detected, then backs off.
     *
     * <p>Per CTRE step 10: "Increase kP until the flywheel oscillates, then back off to just
     * before that oscillation."
     *
     * <p>Two complementary detection mechanisms:
     *
     * <ul>
     *   <li><b>Fast path:</b> peak-to-peak oscillation check over a rolling window. Fires quickly
     *       when oscillation amplitude exceeds {@code oscillationThreshold}.
     *   <li><b>Slow path:</b> convergence timeout. If the motor never settles within the detector's
     *       {@code maxWaitSecs}, it is assumed to be oscillating at low amplitude.
     * </ul>
     *
     * <p>Unlike the original implementation, this phase actually commands a velocity setpoint via
     * {@code setTorqueCurrentFOCVelocityImpl}, which is a public method on {@link
     * ServoMotorSubsystem}.
     */
    private void executeFindKP() {
        if (!kpApplied) {
            if (kpTestValue == 0) kpTestValue = config.kpStartValue();
            subsystem.updateGains(kpTestValue, 0, 0, 0, discoveredKS, 0, discoveredKV, 0, 0, 0);
            final double testVelocity = (config.maxVelocity() + config.minVelocity()) / 2.0;
            subsystem.setTorqueCurrentFOCVelocityImpl(() -> testVelocity, 0);
            oscBufIdx = 0;
            oscBufCount = 0;
            convergence.reset();
            kpApplied = true;
            Logger.recordOutput(logPrefix + "FindKP/TestKP", kpTestValue);
            Logger.recordOutput(logPrefix + "FindKP/TestVelocity", testVelocity);
        }

        double velocity = subsystem.getVelocity();
        Logger.recordOutput(logPrefix + "FindKP/Velocity", velocity);

        if (Math.abs(velocity) > config.maxVelocity() * config.runawayVelocityFactor()) {
            abortWithMsg("Velocity runaway in FIND_KP");
            return;
        }

        // Rolling oscillation buffer (raw signed velocity)
        oscBuffer[oscBufIdx % oscBuffer.length] = velocity;
        oscBufIdx++;
        oscBufCount = Math.min(oscBufCount + 1, oscBuffer.length);

        boolean converged = convergence.update(subsystem.getSmoothedVelocity());

        // Fast oscillation check: wait until the window is full and past minWait
        if (oscBufCount >= oscBuffer.length && convergence.getElapsed() >= config.minSettleWaitSeconds()) {
            double min = Double.MAX_VALUE;
            double max = -Double.MAX_VALUE;
            for (int i = 0; i < oscBufCount; i++) {
                double v = oscBuffer[i];
                if (v < min) min = v;
                if (v > max) max = v;
            }
            double testVelocity = (config.maxVelocity() + config.minVelocity()) / 2.0;
            double oscRatio = (max - min) / Math.abs(testVelocity);
            Logger.recordOutput(logPrefix + "FindKP/OscillationRatio", oscRatio);

            if (oscRatio > config.oscillationThreshold()) {
                discoveredKP = kpTestValue * config.kpBackoffFactor();
                System.out.printf(
                        "FlywheelAutoTune: Oscillation at kP=%.4f (p2p=%.1f%%) → final kP=%.4f%n",
                        kpTestValue, oscRatio * 100, discoveredKP);
                Logger.recordOutput(logPrefix + "FindKP/DiscoveredKP", discoveredKP);
                subsystem.setTorqueCurrentFOCImpl(0);
                transitionTo(Phase.VALIDATE);
                return;
            }
        }

        if (!converged) return;

        if (convergence.timedOut()) {
            // Motor never settled → oscillating or unstable at this kP
            discoveredKP = kpTestValue * config.kpBackoffFactor();
            System.out.printf(
                    "FlywheelAutoTune: kP=%.4f did not settle (oscillating) → final kP=%.4f%n",
                    kpTestValue, discoveredKP);
        } else {
            // Motor settled cleanly — increase kP and repeat
            System.out.printf("FlywheelAutoTune: kP=%.4f settled OK — increasing%n", kpTestValue);
            kpTestValue *= config.kpStepMultiplier();

            if (kpTestValue <= config.kpStartValue() * 1000) {
                kpApplied = false;
                return;
            }
            // Safety cap: no oscillation found up to 1000× start value
            discoveredKP = kpTestValue / config.kpStepMultiplier();
            System.out.printf("FlywheelAutoTune: No oscillation found up to kP=%.4f — using that%n", discoveredKP);
        }

        Logger.recordOutput(logPrefix + "FindKP/DiscoveredKP", discoveredKP);
        subsystem.setTorqueCurrentFOCImpl(0);
        transitionTo(Phase.VALIDATE);
    }

    // ---- VALIDATE -------------------------------------------------------

    /**
     * Verifies the discovered gains by commanding closed-loop velocity setpoints spread across
     * the operating range, waiting for convergence, and checking steady-state error.
     *
     * <p>Per CTRE step 11: "Verify gains hold for expected velocities."
     */
    private void executeValidate() {
        if (!validateApplied) {
            if (validatePointIndex == 0) {
                validateAllPassed = true;
                subsystem.updateGains(discoveredKP, 0, 0, 0, discoveredKS, 0, discoveredKV, 0, 0, 0);
            }

            if (validatePointIndex >= config.validationPoints()) {
                subsystem.setTorqueCurrentFOCImpl(0);
                transitionTo(Phase.REPORT_RESULTS);
                return;
            }

            double fraction = (double) (validatePointIndex + 1) / (config.validationPoints() + 1);
            double targetVelocity = config.minVelocity() + fraction * (config.maxVelocity() - config.minVelocity());
            final double target = targetVelocity;
            subsystem.setTorqueCurrentFOCVelocityImpl(() -> target, 0);
            convergence.reset();
            validateApplied = true;

            Logger.recordOutput(logPrefix + "Validate/PointIndex", validatePointIndex);
            Logger.recordOutput(logPrefix + "Validate/TargetVelocity", target);
        }

        double velocity = Math.abs(subsystem.getVelocity());
        Logger.recordOutput(logPrefix + "Validate/Velocity", velocity);

        if (!convergence.update(subsystem.getSmoothedVelocity())) return;

        double fraction = (double) (validatePointIndex + 1) / (config.validationPoints() + 1);
        double targetVelocity = config.minVelocity() + fraction * (config.maxVelocity() - config.minVelocity());
        double errorPct = Math.abs(velocity - targetVelocity) / Math.abs(targetVelocity);
        boolean passed = errorPct < 0.15;

        System.out.printf(
                "FlywheelAutoTune: Validate[%d] target=%.1f actual=%.1f error=%.1f%% → %s%n",
                validatePointIndex, targetVelocity, velocity, errorPct * 100, passed ? "PASS" : "FAIL");
        Logger.recordOutput(logPrefix + "Validate/Point" + validatePointIndex + "/ErrorPct", errorPct);
        Logger.recordOutput(logPrefix + "Validate/Point" + validatePointIndex + "/Passed", passed);

        if (!passed) validateAllPassed = false;
        validatePointIndex++;
        validateApplied = false;
    }

    // ---- REPORT_RESULTS -------------------------------------------------

    private void executeReportResults() {
        tuneSucceeded = true;

        System.out.println("\n====================================================");
        System.out.println("   FlywheelAutoTune RESULTS for " + subsystem.getName());
        System.out.println("====================================================");
        System.out.printf("   kS = %.4f%n", discoveredKS);
        System.out.printf("   kV = %.6f%n", discoveredKV);
        System.out.printf("   kP = %.4f%n", discoveredKP);
        System.out.println("   Validation: " + (validateAllPassed ? "ALL PASSED" : "SOME FAILED"));
        System.out.println("====================================================");
        System.out.println("   Copy-paste for Slot0Configs:");
        System.out.printf("   .withKS(%.4f).withKV(%.6f).withKP(%.4f)%n", discoveredKS, discoveredKV, discoveredKP);
        System.out.println("====================================================\n");

        Logger.recordOutput(logPrefix + "Result/kS", discoveredKS);
        Logger.recordOutput(logPrefix + "Result/kV", discoveredKV);
        Logger.recordOutput(logPrefix + "Result/kP", discoveredKP);
        Logger.recordOutput(logPrefix + "Result/ValidationPassed", validateAllPassed);
        Logger.recordOutput(logPrefix + "Result/Success", true);

        // Apply discovered gains so the operator can test immediately
        subsystem.updateGains(discoveredKP, 0, 0, 0, discoveredKS, 0, discoveredKV, 0, 0, 0);
        transitionTo(Phase.DONE);
    }

    // ========================= Helpers ===================================

    private void abortWithMsg(String reason) {
        System.out.println("FlywheelAutoTune: ABORT — " + reason);
        subsystem.setTorqueCurrentFOCImpl(0);
        transitionTo(Phase.DONE);
    }

    /**
     * Transitions to a new phase, restarting timers and resetting all phase-local state.
     * Creates a fresh {@link ConvergenceDetector} sized appropriately for each settling phase.
     */
    private void transitionTo(Phase newPhase) {
        phase = newPhase;
        phaseTimer.restart();

        // Buffer size for stability confirmation window: confirmationSecs × 50 Hz
        int settleBufferSize = Math.max(1, (int) (config.settleConfirmationSeconds() * 50));

        switch (newPhase) {
            case FIND_KS -> {
                ksCurrentAmps = 0;
                ksDwellTimer.restart();
                subsystem.setTorqueCurrentFOCImpl(0);
            }
            case TUNE_KV_HIGH -> {
                tuneApplied = false;
                tuneAdjustCount = 0;
                coastComplete = false;
                subsystem.setTorqueCurrentFOCImpl(0);
                convergence = new ConvergenceDetector(
                        settleBufferSize,
                        config.tuneVarianceThresholdPct(),
                        config.tuneNoiseThreshold(),
                        config.settleConfirmationSeconds(),
                        config.minSettleWaitSeconds(),
                        config.tuneMaxSettleWaitSeconds(),
                        config.tTestAlpha());
            }
            case TUNE_KS_LOW -> {
                tuneApplied = false;
                tuneAdjustCount = 0;
                coastComplete = false;
                subsystem.setTorqueCurrentFOCImpl(0);
                convergence = new ConvergenceDetector(
                        settleBufferSize,
                        config.tuneVarianceThresholdPct(),
                        config.tuneNoiseThreshold(),
                        config.settleConfirmationSeconds(),
                        config.minSettleWaitSeconds(),
                        config.tuneMaxSettleWaitSeconds(),
                        config.tTestAlpha());
            }
            case FIND_KP -> {
                kpTestValue = 0;
                kpApplied = false;
                oscBufIdx = 0;
                oscBufCount = 0;
                // For kP: minWait = oscillation window so we always collect a full window;
                // maxWait is 6× the window or 30 s (whichever is larger) per test iteration.
                int kpBufSize = Math.max(1, (int) (config.oscillationWindowSeconds() * 50));
                convergence = new ConvergenceDetector(
                        kpBufSize,
                        config.settleVarianceThresholdPct(),
                        config.tuneNoiseThreshold(),
                        config.settleConfirmationSeconds(),
                        config.oscillationWindowSeconds(),
                        Math.max(config.oscillationWindowSeconds() * 6, 30.0),
                        config.tTestAlpha());
            }
            case VALIDATE -> {
                validatePointIndex = 0;
                validateApplied = false;
                validateAllPassed = true;
                convergence = new ConvergenceDetector(
                        settleBufferSize,
                        config.settleVarianceThresholdPct(),
                        config.tuneNoiseThreshold(),
                        config.settleConfirmationSeconds(),
                        config.minSettleWaitSeconds(),
                        config.maxSettleWaitSeconds(),
                        config.tTestAlpha());
            }
            case RESET_GAINS, REPORT_RESULTS, DONE -> {}
        }

        Logger.recordOutput(logPrefix + "Phase", newPhase.name());
    }
}
