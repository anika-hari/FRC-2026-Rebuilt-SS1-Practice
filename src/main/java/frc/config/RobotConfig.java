package frc.config;

import com.ctre.phoenix6.CANBus;
import java.util.EnumSet;
import java.util.Set;

/**
 * Robot configuration enum that defines which subsystems are enabled for different prototype
 * configurations. Change Constants.ROBOT_CONFIG to switch between configurations.
 */
public enum RobotConfig {
    /** Full competition robot with all subsystems enabled. */
    FULL_ROBOT(EnumSet.allOf(Subsystem.class)),

    /** All But Drive */
    ALL_BUT_DRIVE(EnumSet.of(
            Subsystem.SHOOTER, Subsystem.HOOD, Subsystem.HOPPER, Subsystem.INTAKE_ROLLER, Subsystem.INTAKE_PIVOT)),

    /** All But Vision */
    ALL_BUT_VISION(EnumSet.of(
            Subsystem.DRIVE,
            Subsystem.SHOOTER,
            Subsystem.HOOD,
            Subsystem.HOPPER,
            Subsystem.INTAKE_ROLLER,
            Subsystem.INTAKE_PIVOT)),

    /** SANDSPIT2 */
    SANDSPIT2(
            EnumSet.of(
                    Subsystem.DRIVE,
                    Subsystem.SHOOTER,
                    Subsystem.HOOD,
                    Subsystem.HOPPER,
                    Subsystem.INTAKE_ROLLER,
                    Subsystem.INTAKE_PIVOT,
                    Subsystem.BOX,
                    Subsystem.VISION_APRILTAG
                    // ,
                    // Subsystem.LED
                    ),
            new int[] {}),
    SANDSPIT2_NO_CLIMB(
            EnumSet.of(
                    Subsystem.DRIVE,
                    Subsystem.SHOOTER,
                    Subsystem.HOOD,
                    Subsystem.HOPPER,
                    Subsystem.INTAKE_ROLLER,
                    Subsystem.INTAKE_PIVOT,
                    Subsystem.BOX,
                    Subsystem.VISION_APRILTAG,
                    Subsystem.LED),
            new int[] {}),

    /** SANDSPIT2 without drive*/
    SANDSPIT2_NO_DRIVE(
            EnumSet.of(
                    Subsystem.SHOOTER,
                    Subsystem.HOOD,
                    Subsystem.HOPPER,
                    Subsystem.INTAKE_ROLLER,
                    Subsystem.INTAKE_PIVOT,
                    Subsystem.BOX,
                    Subsystem.LED),
            new int[] {}),

    /** SANDSPIT */
    SANDSPIT(EnumSet.of(
            Subsystem.DRIVE,
            Subsystem.SHOOTER,
            Subsystem.HOOD,
            Subsystem.HOPPER,
            Subsystem.INTAKE_ROLLER,
            Subsystem.INTAKE_PIVOT,
            Subsystem.VISION_APRILTAG,
            Subsystem.VISION_OBJECTDETECTION,
            Subsystem.LED)),
    SANDSPIT_NO_VISION_NO_CLIMB_NO_DRIVE(EnumSet.of(
            Subsystem.SHOOTER,
            Subsystem.HOOD,
            Subsystem.HOPPER,
            Subsystem.INTAKE_ROLLER,
            Subsystem.INTAKE_PIVOT,
            Subsystem.LED)),
    /** SANDSPIT */
    SANDSPIT_NO_CLIMB(EnumSet.of(
            Subsystem.DRIVE,
            Subsystem.SHOOTER,
            Subsystem.HOOD,
            Subsystem.HOPPER,
            Subsystem.INTAKE_ROLLER,
            Subsystem.INTAKE_PIVOT,
            Subsystem.VISION_APRILTAG,
            Subsystem.VISION_OBJECTDETECTION,
            Subsystem.VISION_HOPPERDETECTION)),
    /** SANDSPIT without od*/
    SANDSPIT_NO_CLIMB_NO_OD(EnumSet.of(
            Subsystem.DRIVE,
            Subsystem.SHOOTER,
            Subsystem.HOOD,
            Subsystem.HOPPER,
            Subsystem.INTAKE_ROLLER,
            Subsystem.INTAKE_PIVOT,
            Subsystem.VISION_APRILTAG)),
    /** SANDSPIT without drive*/
    SANDSPIT_NO_DRIVE(EnumSet.of(
            Subsystem.SHOOTER,
            Subsystem.HOOD,
            Subsystem.HOPPER,
            Subsystem.INTAKE_ROLLER,
            Subsystem.INTAKE_PIVOT,
            Subsystem.VISION_APRILTAG,
            Subsystem.VISION_OBJECTDETECTION,
            Subsystem.LED)),
    /** All subsystems disabled. */
    ALL_DISABLED(EnumSet.noneOf(Subsystem.class)),

    DRIVE_ONLY(EnumSet.of(Subsystem.DRIVE)),

    HOOD_ONLY(EnumSet.of(Subsystem.HOOD)),

    BOX_ONLY(EnumSet.of(Subsystem.BOX)),

    INTAKE_PIVOT_ONLY(EnumSet.of(Subsystem.INTAKE_PIVOT)),

    NO_CLIMB_OR_INTAKE_OR_CAMERA(EnumSet.of(Subsystem.DRIVE, Subsystem.SHOOTER, Subsystem.HOOD, Subsystem.HOPPER)),

    FLYWHEEL_AND_DRIVE(EnumSet.of(Subsystem.DRIVE, Subsystem.SHOOTER)),

    FLYWHEEL_ONLY(EnumSet.of(Subsystem.SHOOTER)),

    HOOD_AND_SHOOTER(EnumSet.of(Subsystem.HOOD, Subsystem.SHOOTER)),

    /** Drivetrain only - for drive testing and tuning. */
    DRIVETRAIN_AND_VISION_ONLY(EnumSet.of(Subsystem.DRIVE, Subsystem.VISION_APRILTAG)),

    /** Intake prototype - drivetrain + intake roller only. */
    INTAKE_PROTOTYPE(EnumSet.of(Subsystem.DRIVE, Subsystem.INTAKE_ROLLER)),

    /** Full intake prototype - drivetrain + full intake system + hopper. */
    INTAKE_FULL_PROTOTYPE(
            EnumSet.of(Subsystem.DRIVE, Subsystem.INTAKE_ROLLER, Subsystem.INTAKE_PIVOT, Subsystem.HOPPER)),

    /** Shooter prototype - drivetrain + shooter + hood. */
    SHOOTER_PROTOTYPE(EnumSet.of(Subsystem.DRIVE, Subsystem.SHOOTER, Subsystem.HOOD)),

    /** Feeder prototype - drivetrain + feeders + hopper. */
    FEEDER_PROTOTYPE(EnumSet.of(Subsystem.DRIVE, Subsystem.HOPPER));

    /** Enum of all subsystems that can be enabled/disabled. */
    public enum Subsystem {
        DRIVE,
        VISION_APRILTAG,
        VISION_OBJECTDETECTION,
        VISION_HOPPERDETECTION,
        SHOOTER,
        HOOD,
        HOPPER,
        INTAKE_ROLLER,
        INTAKE_PIVOT,
        BOX,
        LED
    }

    private final Set<Subsystem> enabledSubsystems;
    private final CANBus driveCANBus;
    private final CANBus miscCANBus;
    private final CANBus rioCANBus;
    private final int[] disabledCANIds;

    RobotConfig(Set<Subsystem> enabledSubsystems) {
        this(enabledSubsystems, new int[] {});
    }

    RobotConfig(Set<Subsystem> enabledSubsystems, int[] disabledCANIds) {
        this.enabledSubsystems = enabledSubsystems;
        this.driveCANBus = new CANBus(ConfigConstants.DRIVE_CANIVORE_NAME);
        this.miscCANBus = new CANBus(ConfigConstants.MISC_CANIVORE_NAME);
        this.rioCANBus = new CANBus("rio");
        this.disabledCANIds = disabledCANIds;
    }

    RobotConfig(Set<Subsystem> enabledSubsystems, String nonRioCANBusName) {
        this(enabledSubsystems, nonRioCANBusName, nonRioCANBusName);
    }

    RobotConfig(Set<Subsystem> enabledSubsystems, String nonRioCANBusName, int[] disabledCANIds) {
        this(enabledSubsystems, nonRioCANBusName, nonRioCANBusName, disabledCANIds);
    }

    RobotConfig(Set<Subsystem> enabledSubsystems, String driveCANBusName, String miscCANBusName) {
        this(enabledSubsystems, driveCANBusName, miscCANBusName, new int[] {});
    }

    RobotConfig(Set<Subsystem> enabledSubsystems, String driveCANBusName, String miscCANBusName, int[] disabledCANIds) {
        this.enabledSubsystems = enabledSubsystems;
        this.driveCANBus = new CANBus(driveCANBusName);
        if (driveCANBusName.equals(miscCANBusName)) {
            this.miscCANBus = driveCANBus;
        } else {
            this.miscCANBus = new CANBus(miscCANBusName);
        }
        this.rioCANBus = new CANBus("rio");
        this.disabledCANIds = disabledCANIds;
    }

    /**
     * Check if a subsystem is enabled in this configuration.
     *
     * @param subsystem The subsystem to check
     * @return true if the subsystem is enabled
     */
    public boolean isEnabled(Subsystem subsystem) {
        return enabledSubsystems.contains(subsystem);
    }

    public boolean isDisabled(Subsystem subsystem) {
        return !isEnabled(subsystem);
    }

    public boolean isCANIDDisabled(int canID) {
        for (Integer disabledID : disabledCANIds) {
            if (disabledID == canID) {
                return true;
            }
        }
        return false;
    }

    /**
     * Get all enabled subsystems for this configuration.
     *
     * @return A copy of the enabled subsystems set
     */
    public Set<Subsystem> getEnabledSubsystems() {
        return EnumSet.copyOf(enabledSubsystems);
    }

    public CANBus driveCANBus() {
        return driveCANBus;
    }

    public CANBus miscCANBus() {
        return miscCANBus;
    }

    public CANBus rioCANBus() {
        return rioCANBus;
    }
}
