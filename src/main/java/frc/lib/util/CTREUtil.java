package frc.lib.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class CTREUtil {
    public static final int MAX_RETRIES = 10;

    public static StatusCode tryUntilOK(Supplier<StatusCode> function, int deviceId) {
        final int max_num_retries = 10;
        StatusCode statusCode = StatusCode.OK;
        for (int i = 0; i < max_num_retries; ++i) {
            statusCode = function.get();
            if (statusCode == StatusCode.OK) {
                break;
            }
        }
        if (statusCode != StatusCode.OK) {
            DriverStation.reportError(
                    "Error calling " + function + " on ctre device id " + deviceId + ": " + statusCode, true);
        }
        return statusCode;
    }

    public static StatusCode applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(TalonFX motor, VoltageConfigs config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfigurationNonBlocking(TalonFX motor, VoltageConfigs config) {
        return motor.getConfigurator().apply(config, 0.01);
    }

    public static StatusCode applyConfiguration(TalonFX motor, HardwareLimitSwitchConfigs config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(TalonFX motor, MotionMagicConfigs config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(TalonFX motor, CurrentLimitsConfigs config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode refreshConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(CANcoder cancoder, CANcoderConfiguration config) {
        return tryUntilOK(() -> cancoder.getConfigurator().apply(config), cancoder.getDeviceID());
    }

    public static StatusCode applyConfiguration(CANrange canrange, CANrangeConfiguration config) {
        return tryUntilOK(() -> canrange.getConfigurator().apply(config), canrange.getDeviceID());
    }

    public static StatusCode applyConfiguration(CANdi candi, CANdiConfiguration config) {
        return tryUntilOK(() -> candi.getConfigurator().apply(config), candi.getDeviceID());
    }

    public static CANdiConfiguration createCandiConfiguration() {
        CANdiConfiguration candiConfiguration = new CANdiConfiguration();
        candiConfiguration.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenNotHigh;
        candiConfiguration.DigitalInputs.S1FloatState = S1FloatStateValue.PullLow;
        candiConfiguration.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenNotHigh;
        candiConfiguration.DigitalInputs.S2FloatState = S2FloatStateValue.PullLow;
        return candiConfiguration;
    }

    public static CANdiConfiguration createCustomCandiConfiguration(
            S1CloseStateValue s1CloseState,
            S1FloatStateValue s1FloatState,
            S2CloseStateValue s2CloseState,
            S2FloatStateValue s2FloatState) {
        CANdiConfiguration candiConfiguration = new CANdiConfiguration();
        candiConfiguration.DigitalInputs.S1CloseState = s1CloseState;
        candiConfiguration.DigitalInputs.S1FloatState = s1FloatState;
        candiConfiguration.DigitalInputs.S2CloseState = s2CloseState;
        candiConfiguration.DigitalInputs.S2FloatState = s2FloatState;
        return candiConfiguration;
    }

    /**
     * Calculates the optimal location for the Cancoder discontinuity point based on the mechanism's range of motion and center position. The discontinuity point is placed on the opposite side of the allowed range of motion from the center position,
     *
     * @param mechanismMinimumUnits the minimum position of the mechanism in the units that the mechanism is controlled in (e.g. degrees, meters, etc.)
     * @param mechanismMaximumUnits the maximum position of the mechanism in the units that the mechanism is controlled in (e.g. degrees, meters, etc.)
     * @param mechanismCenterUnits the center position of the mechanism in the units that the mechanism is controlled in (e.g. degrees, meters, etc.). Required to be between the minimum and maximum positions. Used to set the discontinuity point on the opposite side of the range of motion.
     * @param sensorToMechanismGearRatio the gear ratio between the sensor and the mechanism, defined as the number of rotations the mechanism makes for one rotation of the sensor. A reduction would have a ratio of less than 1.
     * @param unitsToMechanismRatio the ratio between the units that the mechanism is controlled in and rotations of the mechanism's final component, defined as the number of units the mechanism moves for one rotation of the mechanism's final component. For example, if the mechanism is an arm that rotates and the position is controlled in radians, this would be 2*pi. If the mechanism is a linear elevator controlled in meters and it moves 0.05 meters for one rotation of the spool, this would be 0.05.
     * @return the optimal location for the Cancoder discontinuity point in rotations of the sensor, between 0 and 1. This is calculated to be the center of the two limits on the opposite side of the allowed range of motion.
     */
    public static double calculateCanCoderDiscontinuityPoint(
            double mechanismMinimumUnits,
            double mechanismMaximumUnits,
            double mechanismCenterUnits,
            double sensorToMechanismGearRatio,
            double unitsToMechanismRatio) {
        double cancoderMaximumRotations = mechanismMaximumUnits / unitsToMechanismRatio / sensorToMechanismGearRatio;
        double cancoderMinimumRotations = mechanismMinimumUnits / unitsToMechanismRatio / sensorToMechanismGearRatio;
        if (cancoderMaximumRotations - cancoderMinimumRotations > 1) {
            // If the allowed range of motion is greater than one rotation, then the discontinuity point shouldn't be
            // set automatically,
            // it should be manually set to a convenient location and zeroing must be done by other means.
            System.out.println(
                    "Warning: The mechanism range of motion is greater than one rotation, so the automatic calculation of the "
                            + "cancoder discontinuity point may not work correctly. Please set the discontinuity point manually and ensure that zeroing is done by other means.");
            return 0;
        }
        if (cancoderMaximumRotations < cancoderMinimumRotations) {
            // If the maximum rotation is less than the minimum rotation, print a warning that the discontinuity point
            // may be calculated incorrectly and return 0
            System.out.println(
                    "Warning: The calculated cancoder maximum rotations is less than the minimum rotations, which may indicate an issue with the input parameters. "
                            + "The automatic calculation of the cancoder discontinuity point may not work correctly. Please check the input parameters and ensure they are correct.");
            return 0;
        }
        double cancoderMiddleRotations = mechanismCenterUnits / unitsToMechanismRatio / sensorToMechanismGearRatio;
        Rotation2d cancoderMaxmiumRotation2d = new Rotation2d(Units.rotationsToRadians(cancoderMaximumRotations));
        Rotation2d cancoderMinimumRotation2d = new Rotation2d(Units.rotationsToRadians(cancoderMinimumRotations));
        Rotation2d cancoderMiddleRotation2d = new Rotation2d(Units.rotationsToRadians(cancoderMiddleRotations));
        // We want the discontinuity to be on the opposite side of the limits from where the
        // mechanism can actually reach, so we add pi to the allowed position between limits to
        // flip it around.
        return Util.rangeModulo(
                Util.getMiddleRotationChoosingSide(
                                cancoderMinimumRotation2d,
                                cancoderMaxmiumRotation2d,
                                cancoderMiddleRotation2d.plus(Rotation2d.kPi))
                        .getRotations(),
                0,
                1);
    }
}
