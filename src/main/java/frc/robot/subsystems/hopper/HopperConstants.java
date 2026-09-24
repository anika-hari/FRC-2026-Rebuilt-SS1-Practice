package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.config.ConfigConstants;
import frc.lib.drivers.CANDeviceId;
import frc.lib.subsystems.SubsystemUnit;
import frc.lib.subsystems.TalonFXMotorType;
import frc.lib.subsystems.real.ServoMotorSubsystemConfig;

public class HopperConstants {
    public static final ServoMotorSubsystemConfig kHopperConfig = new ServoMotorSubsystemConfig();

    public enum HopperState {
        IDLE,
        FEEDING,
        REVERSE,
        JAMMED
    }


    // ====CAN IDs====
    public static final int kHopperID = 51;
    // ====Tunable Constants====
    private static final double Tunable_Hopper_kP = 2.0; // TODO: calibrate on Sandspit
    private static final double Tunable_Hopper_kI = 0.0; // TODO: calibrate on Sandspit
    private static final double Tunable_Hopper_kD = 0.1; // TODO: calibrate on Sandspit
    private static final double Tunable_Hopper_kG = 0.04; // TODO: calibrate on Sandspit
    private static final double Tunable_Hopper_kS = 0.2; // TODO: calibrate on Sandspit
    private static final double Tunable_Hopper_kV = 0.0; // TODO: calibrate on Sandspit
    private static final double Tunable_Hopper_kA = 0.0; // TODO: calibrate on Sandspit
    // ====Motion Magic====
    private static final double Tunable_Hopper_Velo = 300;
    private static final double Tunable_Hopper_Accel = 9999; // 9999 is the maximum value for this
    private static final double Tunable_Hopper_Jerk = 0; // 0 turns off jerk control. Jerk control evidently has issues.
    // ====MISC====
    public static final double kHopperMomentOfInertia = 0.0001;
    public static final double kHopperCurrentLimitAmps = 80;
    // ====GEAR RATIOS====
    private static final double kRotorToHopperGearRatio = (12.0 / 20.0);
    private static final double kSensorToHopperGearRatio =
            kRotorToHopperGearRatio; // Sensor is the motor encoder, which is on the rotor
    private static final double kUnitsToHopperRatio =
            Units.rotationsToRadians(1); // CTRE stuff reports in mechanism rotations, we want radians
    // ====CANRANGE CONSTANTS====
    private static final double kFOVDegrees = 12.75;
    private static final double kMinSignalStrengthForValidMeasurement = 40000;
    private static final double kProximityThresholdMeters = Units.inchesToMeters(3);
    private static final double kProximityHysteresis = Units.inchesToMeters(0.5);
    // ====OTHER====
    public static final double kHopperVoltage = 12;
    public static final double hopperRevTime = 0.2;
    // ====MOTOR CONFIGS====

    //Create a TalonFXConfiguration with the constants above

    TalonFXConfiguration config = new TalonFXConfiguration();
    //create a new TalonFXConfiguration() and use .with to set the config
    //Example: new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(Tunable_Hopper_kP))
    //RotorToSensorRatio is 1 / (kRotorToHopperGearRatio / kSensorToHopperGearRatio)
    //SensorToMechanism is (1 / kSensorToHopperGearRatio)
    //NeutralMode should be set to Brake, and Inverted should be Clockwise_Positive

    // ====CANRANGE CONFIGS====
    //create a CANrangeConfiguration object with the constants above

    static {
        // ====MOTOR=====
        kHopperConfig.name = "Hopper";
        kHopperConfig.unit = SubsystemUnit.RADIANS;
        kHopperConfig.talonCANID = new CANDeviceId(kHopperID, ConfigConstants.ROBOT_CONFIG.miscCANBus());
        kHopperConfig.fxConfig = hopperFXConfig;
        kHopperConfig.motorType = TalonFXMotorType.KRAKEN_X60_FOC;
        kHopperConfig.momentOfInertia = kHopperMomentOfInertia;
        kHopperConfig.unitToRotorRatio = kUnitsToHopperRatio;
    }
}
