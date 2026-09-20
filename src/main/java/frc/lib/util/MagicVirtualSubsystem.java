package frc.lib.util;

import java.util.ArrayList;
import java.util.List;

/** Subsystems that are intended to run periodically, independently of the CommandScheduler. */
public abstract class MagicVirtualSubsystem {
    private static final List<MagicVirtualSubsystem> virtualSubsystems = new ArrayList<>();
    protected boolean disabled = false;

    /**
     * Every subsystem that extends VirtualSubsystem gets added to the list of Subsystems that are
     * called every robot period.
     */
    public MagicVirtualSubsystem() {
        virtualSubsystems.add(this);
    }

    public static void listVirtualSubsystems() {
        for (MagicVirtualSubsystem subsystem : virtualSubsystems) {
            String message = String.format(
                    "[robotzgarage] Connected Virtual Subsystem '%s'",
                    subsystem.getClass().getSimpleName());
            System.out.println(message);
        }
    }

    /** Call {@link #periodic()} on every virtual subsystem. */
    public static void runPeriodically() {
        for (MagicVirtualSubsystem subsystem : virtualSubsystems) {
            subsystem.periodic();
        }
    }

    /** Call {@link #simulationPeriodic()} on every virtual subsystem. */
    public static void runSimulationPeriodically() {
        for (MagicVirtualSubsystem subsystem : virtualSubsystems) {
            subsystem.simulationPeriodic();
        }
    }

    /** Method to run on every thread update. */
    public abstract void periodic();

    /** Method to run on every simulated thread update. */
    public abstract void simulationPeriodic();

    /**
     * Check if this virtual subsystem is disabled in the current robot configuration.
     *
     * @return true if the subsystem is disabled
     */
    public boolean isDisabled() {
        return disabled;
    }

    /**
     * Check if this virtual subsystem is enabled in the current robot configuration.
     *
     * @return true if the subsystem is enabled
     */
    public boolean isEnabled() {
        return !disabled;
    }

    /**
     * Set whether this virtual subsystem is disabled.
     *
     * @param disabled true to disable the subsystem
     */
    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }
}
