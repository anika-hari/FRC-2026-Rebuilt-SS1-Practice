package frc.lib.util;

import static frc.config.ConfigConstants.logLoopTimes;

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class for measuring and logging execution time of robot periodic methods. integrates with
 * AdvantageKit Logger for data recording.
 */
public class LoopTimingLogger {
    private static final Map<String, Long> startTimes = new HashMap<>();
    private static final double LOOP_OVERRUN_THRESHOLD_MS = 18.0; // 18ms threshold for 20ms loop
    private static double maxLoopTime = 0.0;
    private static boolean hasLoopOverrun = false;

    private LoopTimingLogger() {
        throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
    }

    /**
     * Start timing measurement for a named method or operation.
     *
     * @param methodName The name of the method/operation being timed
     */
    public static void startTiming(String methodName) {
        if (!logLoopTimes) return;

        startTimes.put(methodName, System.nanoTime());
    }

    /**
     * End timing measurement and log the result using AdvantageKit Logger.
     *
     * @param methodName The name of the method/operation being timed
     * @return The execution time in milliseconds, or -1 if timing was not started
     */
    public static double endTiming(String methodName) {
        if (!logLoopTimes) return 0;

        Long startTime = startTimes.get(methodName);
        if (startTime == null) {
            Logger.recordOutput("LoopTiming/Error", "endTiming called without startTiming for: " + methodName);
            return -1.0;
        }

        long durationNanos = System.nanoTime() - startTime;
        double durationMs = durationNanos / 1_000_000.0;

        Logger.recordOutput("LoopTiming/" + methodName + "MS", durationMs);

        // Track maximum timing and detect loop overruns for robotPeriodic
        if (methodName.equals("RobotPeriodic")) {
            maxLoopTime = Math.max(maxLoopTime, durationMs);
            hasLoopOverrun = durationMs > LOOP_OVERRUN_THRESHOLD_MS;

            Logger.recordOutput("LoopTiming/MaxLoopMS", maxLoopTime);
            Logger.recordOutput("LoopTiming/LoopOverrun", hasLoopOverrun);
            Logger.recordOutput("LoopTiming/LoopUtilizationPercent", (durationMs / 20.0) * 100.0);

            if (hasLoopOverrun) {
                Logger.recordOutput(
                        "LoopTiming/OverrunWarning",
                        "[LoopTiming] WARNING: Loop overrun detected! "
                                + String.format("%.3f", durationMs)
                                + "ms exceeds "
                                + LOOP_OVERRUN_THRESHOLD_MS
                                + "ms threshold");
            }
        }

        startTimes.remove(methodName);

        return durationMs;
    }

    /** Get the current loop overrun threshold in milliseconds. */
    public static double getLoopOverrunThreshold() {
        return LOOP_OVERRUN_THRESHOLD_MS;
    }

    /** Get the maximum recorded loop time since startup. */
    public static double getMaxLoopTime() {
        return maxLoopTime;
    }

    /** Check if there has been a loop overrun since the last robotPeriodic call. */
    public static boolean hasLoopOverrun() {
        return hasLoopOverrun;
    }

    /** Reset the maximum loop time tracking */
    public static void resetMaxLoopTime() {
        maxLoopTime = 0.0;
    }

    /** Clear all active timings */
    public static void clearAllTimings() {
        startTimes.clear();
    }
}
