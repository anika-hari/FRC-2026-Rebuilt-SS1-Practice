package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static final boolean tuningMode = true;
    public static final boolean elasticButtonsEnabled = true;
    public static final boolean fullLogging = true;
    public static boolean disableHAL = false;

    public static void disableHAL() {
        disableHAL = true;
    }

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final Mode simMode = Mode.SIM;

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final double robotWidthWithBumpers = Units.inchesToMeters(34.25); // TODO: calibrate on Sandspit
}
