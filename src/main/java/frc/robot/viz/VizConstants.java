package frc.robot.viz;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;

public class VizConstants {

    public static Translation2d viz_dimensions = new Translation2d(2, 3);

    public static double drivebase_x =
            (viz_dimensions.getX() - Units.inchesToMeters(DriveConstants.kBumperLengthInches)) / 2;
    public static double drivebase_y = 0.05;
    public static double drivebase_top_y = drivebase_y + Units.inchesToMeters(4.25);

    public static double intake_x = drivebase_x + Units.inchesToMeters(7.484);
    public static double intake_y = drivebase_top_y + 0.05;

    public static double box_x = drivebase_x + Units.inchesToMeters(12);
    public static double box_y = drivebase_top_y + Units.inchesToMeters(20);

    public static double hopper_hotdog_1_x = drivebase_x + 0.3;
    public static double hopper_hotdogs_y = drivebase_top_y + 0.05;
    public static double hopper_hotdog_2_x = drivebase_x + Units.inchesToMeters(13.5528);

    public static double feeder_x = drivebase_x + 0.6 + Units.inchesToMeters(1.9685);
    public static double feederbot_y = drivebase_y + Units.inchesToMeters(11.2365);
    public static double feedertop_y = feederbot_y + Units.inchesToMeters(5.199);

    public static double shooter_x = feeder_x + Units.inchesToMeters(10);
    public static double shooter_y = feedertop_y + Units.inchesToMeters(4);

    public static double left_led_x = drivebase_x + 0.4;
    public static double right_led_x = drivebase_x + 0.5;

    public static double led_y = drivebase_y + 0.5;
}
