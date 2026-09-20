package frc.lib.subsystems;

import com.ctre.phoenix6.StatusSignalCollection;
import frc.lib.subsystems.canDevice.CanCoderIO;
import frc.lib.subsystems.canDevice.CanCoderInputs;
import frc.lib.subsystems.canDevice.CanRangeIO;
import frc.lib.subsystems.canDevice.CanRangeInputs;
import frc.lib.subsystems.canDevice.CandiIO;
import frc.lib.subsystems.canDevice.CandiInputs;
import frc.lib.subsystems.motorInputs.MotorInputs;
import frc.lib.util.LoopTimingLogger;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;

/**
 * Central manager to refresh CTRE BaseStatusSignals for lower-priority devices
 * and then delegate to each registered IO to fill its inputs object.
 *
 * IMPORTANT: Drive/pigeon IMU signals are intentionally refreshed locally at a
 * higher frequency inside `DriveIOHardware`. Do NOT register drive pigeon or
 * other high-frequency drivetrain signals with this manager — they must be
 * refreshed at a higher rate to avoid "stale" warnings.
 */
public class CTREStatusSignalManager {
    private static final StatusSignalCollection statusSignals = new StatusSignalCollection();
    private static final List<MotorEntry> motorRegistry = Collections.synchronizedList(new ArrayList<>());
    private static final List<CanCoderEntry> cancoderRegistry = Collections.synchronizedList(new ArrayList<>());
    private static final List<CanRangeEntry> canRangeRegistry = Collections.synchronizedList(new ArrayList<>());
    private static final List<CandiEntry> candiRegistry = Collections.synchronizedList(new ArrayList<>());

    private static record MotorEntry(MotorIO io, MotorInputs inputs) {}

    private static record CanCoderEntry(CanCoderIO io, CanCoderInputs inputs) {}

    private static record CanRangeEntry(CanRangeIO io, CanRangeInputs inputs) {}

    private static record CandiEntry(CandiIO io, CandiInputs inputs) {}

    public static void register(MotorIO io, MotorInputs inputs) {
        statusSignals.addSignals(io.getStatusSignals());
        motorRegistry.add(new MotorEntry(io, inputs));
    }

    public static void register(CanCoderIO io, CanCoderInputs inputs) {
        statusSignals.addSignals(io.getStatusSignals());
        cancoderRegistry.add(new CanCoderEntry(io, inputs));
    }

    public static void register(CanRangeIO io, CanRangeInputs inputs) {
        statusSignals.addSignals(io.getStatusSignals());
        canRangeRegistry.add(new CanRangeEntry(io, inputs));
    }

    public static void register(CandiIO io, CandiInputs inputs) {
        statusSignals.addSignals(io.getStatusSignals());
        candiRegistry.add(new CandiEntry(io, inputs));
    }

    /**
     * Refresh all signals in a single call and then call readInputs on each registered IO.
     */
    public static void refreshAndReadAll() {
        // Refresh all signals
        LoopTimingLogger.startTiming("CTRE/refreshAll");
        statusSignals.refreshAll();
        LoopTimingLogger.endTiming("CTRE/refreshAll");

        // Call readInputs on all registered devices using a small helper to reduce duplication
        safeReadAll(motorRegistry, e -> e.io.readInputs(e.inputs()), "Motor");
        safeReadAll(cancoderRegistry, e -> e.io.readInputs(e.inputs()), "Cancoder");
        safeReadAll(canRangeRegistry, e -> e.io.readInputs(e.inputs()), "CanRange");
        safeReadAll(candiRegistry, e -> e.io.readInputs(e.inputs()), "Candi");
    }

    private static <T> void safeReadAll(List<T> registry, Consumer<T> reader, String label) {
        LoopTimingLogger.startTiming("CTRE/read" + label + "Inputs");
        synchronized (registry) {
            for (T e : registry) {
                try {
                    reader.accept(e);
                } catch (Exception ex) {
                    System.out.println("Error reading " + label + " inputs: " + ex);
                }
            }
        }
        LoopTimingLogger.endTiming("CTRE/read" + label + "Inputs");
    }
}
