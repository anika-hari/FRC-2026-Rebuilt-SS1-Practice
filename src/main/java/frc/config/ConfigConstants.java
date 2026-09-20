package frc.config;

import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.util.MacAddressUtil;
import java.net.SocketException;

public class ConfigConstants {
    public static boolean logLoopTimes = false;
    public static byte[] mac;

    static {
        try {
            ConfigConstants.mac = MacAddressUtil.getMacAddress();
        } catch (SocketException e) {
            System.out.println("Failed to get MAC address");
            ConfigConstants.mac = new byte[6];
        }
    }

    public static MacAddressUtil.RobotIdentity simRobotIdentity = MacAddressUtil.RobotIdentity.SANDSPIT2;
    public static MacAddressUtil.RobotIdentity robotIdentity =
            RobotBase.isSimulation() ? simRobotIdentity : MacAddressUtil.RobotIdentity.SANDSPIT2;

    public static String DRIVE_CANIVORE_NAME = "DRIVE";
    public static String MISC_CANIVORE_NAME = "MISC";

    // ROBOT CONFIG MUST COME AT END OF THIS FILE BECAUSE IT DEPENDS ON OTHER CONSTANTS IN THIS FILE
    /**
     * Robot configuration - change this to run different prototypes. Options: - FULL_ROBOT: All
     * subsystems enabled -
     * DRIVETRAIN_ONLY: Drive + Vision + LED
     * INTAKE_PROTOTYPE: Drive + Intake + Roller + LED
     * INTAKE_FULL_PROTOTYPE: Drive + Full Intake + Hopper + LED
     * SHOOTER_PROTOTYPE: Drive + Shooter + Hood + LED
     * FEEDER_PROTOTYPE: Drive + Feeders + Hopper + LED
     * LITERALLY_ONLY_DRIVE: Drive + LED
     */
    public static final RobotConfig ROBOT_CONFIG = RobotConfig.SANDSPIT2;
}
