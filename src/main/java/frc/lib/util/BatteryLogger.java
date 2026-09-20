package frc.lib.util;

import edu.wpi.first.wpilibj.RobotController;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.regex.Pattern;
import org.littletonrobotics.junction.Logger;

/**
 * Static utility class for logging current, power, and energy usage per subsystem.
 *
 * <p>Subsystems call {@link #reportCurrentUsage} during their periodic methods. After the
 * CommandScheduler runs, {@link #periodicAfterScheduler} aggregates the reports, computes power and
 * energy, and logs everything via AdvantageKit.
 *
 * <p>Keys support hierarchical aggregation using {@code /} and {@code -} delimiters. For example,
 * {@code "Drive/FL-Drive"} automatically aggregates into {@code "Drive/FL"} and {@code "Drive"}.
 */
public class BatteryLogger {
    private static final double LOOP_PERIOD_SECONDS = 0.02;
    private static final Pattern KEY_SPLIT_PATTERN = Pattern.compile("/|-");

    // Controls overhead estimates (amps)
    private static final double RIO_CURRENT_AMPS = 2.0;
    private static final double CANIVORE_CURRENT_AMPS = 0.03;
    private static final double PIGEON_CURRENT_AMPS = 0.04;
    private static final double CANCODER_CURRENT_AMPS = 0.05 * 6; // 4 drive + IntakePivot + Hood
    private static final double RADIO_CURRENT_AMPS = 0.5;

    private static double totalCurrent = 0.0;
    private static double totalPower = 0.0;
    private static double totalEnergy = 0.0;
    private static double batteryVoltage = 12.0;

    private static final Map<String, Double> subsystemCurrents = new LinkedHashMap<>();
    private static final Map<String, Double> subsystemPowers = new LinkedHashMap<>();
    private static final Map<String, Double> subsystemEnergies = new LinkedHashMap<>();

    private BatteryLogger() {}

    /**
     * Report current usage for a subsystem or motor.
     *
     * <p>Called from each subsystem's periodic method. The key supports hierarchical aggregation
     * using {@code /} and {@code -} delimiters.
     *
     * @param key hierarchical key (e.g. "Drive/FL-Drive", "Shooter/Leader")
     * @param amps one or more current values in amps (absolute values are summed)
     */
    public static void reportCurrentUsage(String key, double... amps) {
        double totalAmps = 0.0;
        for (double amp : amps) totalAmps += Math.abs(amp);

        double power = totalAmps * batteryVoltage;
        double energy = power * LOOP_PERIOD_SECONDS;

        totalCurrent += totalAmps;
        totalPower += power;
        totalEnergy += energy;

        subsystemCurrents.put(key, totalAmps);
        subsystemPowers.put(key, power);
        subsystemEnergies.merge(key, energy, Double::sum);

        // Aggregate into parent keys by splitting on "/" and "-"
        String[] keys = KEY_SPLIT_PATTERN.split(key);
        if (keys.length < 2) return;

        String subkey = "";
        for (int i = 0; i < keys.length - 1; i++) {
            subkey += keys[i];
            if (i < keys.length - 2) {
                subkey += "/";
            }
            subsystemCurrents.merge(subkey, totalAmps, Double::sum);
            subsystemPowers.merge(subkey, power, Double::sum);
            subsystemEnergies.merge(subkey, energy, Double::sum);
        }
    }

    /**
     * Called once per loop after the CommandScheduler has run all subsystem periodics. Adds controls
     * overhead, logs all values, and resets per-cycle accumulators.
     */
    public static void periodicAfterScheduler() {
        batteryVoltage = RobotController.getBatteryVoltage();

        // Report fixed controls overhead
        reportCurrentUsage("Controls/RIO", RIO_CURRENT_AMPS);
        reportCurrentUsage("Controls/CANcoders", CANCODER_CURRENT_AMPS);
        reportCurrentUsage("Controls/Pigeon", PIGEON_CURRENT_AMPS);
        reportCurrentUsage("Controls/CANivore", CANIVORE_CURRENT_AMPS);
        reportCurrentUsage("Controls/Radio", RADIO_CURRENT_AMPS);

        // Log totals
        Logger.recordOutput("Power/TotalCurrentAmps", totalCurrent);
        Logger.recordOutput("Power/TotalPowerWatts", totalPower);
        Logger.recordOutput("Power/TotalEnergyWh", joulesToWattHours(totalEnergy));
        Logger.recordOutput("Power/BatteryVoltage", batteryVoltage);

        // Log per-key current (reset each cycle)
        for (var entry : subsystemCurrents.entrySet()) {
            Logger.recordOutput("Power/Current/" + entry.getKey(), entry.getValue());
            subsystemCurrents.put(entry.getKey(), 0.0);
        }

        // Log per-key power (reset each cycle)
        for (var entry : subsystemPowers.entrySet()) {
            Logger.recordOutput("Power/Power/" + entry.getKey(), entry.getValue());
            subsystemPowers.put(entry.getKey(), 0.0);
        }

        // Log per-key energy (cumulative, never reset)
        for (var entry : subsystemEnergies.entrySet()) {
            Logger.recordOutput("Power/Energy/" + entry.getKey(), joulesToWattHours(entry.getValue()));
        }

        // Reset per-cycle totals (energy is cumulative and never resets)
        totalCurrent = 0.0;
        totalPower = 0.0;
    }

    public static double getTotalCurrent() {
        return totalCurrent;
    }

    public static double getTotalPower() {
        return totalPower;
    }

    public static double getTotalEnergy() {
        return totalEnergy;
    }

    private static double joulesToWattHours(double joules) {
        return joules / 3600.0;
    }
}
