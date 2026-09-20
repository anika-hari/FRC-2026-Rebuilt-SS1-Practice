package frc.lib.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.drivers.CANDeviceId;
import frc.lib.subsystems.motorInputs.MotorInputs;

/**
 * MotorIO interface for hardware abstraction. Implementations may expose status signals
 * so a central refresher can update all signals once per robot loop.
 */
public interface MotorIO {
    void readInputs(MotorInputs inputs);

    void setOpenLoopDutyCycle(double dutyCycle);

    // These are in the "units" of the subsystem (rad, m).
    void setPositionSetpoint(double units);

    default void setMotionMagicSetpoint(double units) {
        setMotionMagicSetpoint(units, 0);
    }

    void setMotionMagicSetpoint(double units, int slot);

    default void setMotionMagicSetpoint(double units, double velocity, double acceleration, double jerk) {
        setMotionMagicSetpoint(units, velocity, acceleration, jerk, 0);
    }

    default void setMotionMagicSetpoint(double units, double velocity, double acceleration, double jerk, int slot) {
        setMotionMagicSetpoint(units, velocity, acceleration, jerk, slot, 0.0);
    }

    void setMotionMagicSetpoint(
            double units, double velocity, double acceleration, double jerk, int slot, double feedforward);

    void updatePIDFFMM(
            double kP,
            double kI,
            double kD,
            double kG,
            double kS,
            double kA,
            double kV,
            double velo,
            double accel,
            double jerk);

    void setNeutralMode(NeutralModeValue mode);

    default void setVelocitySetpoint(double unitsPerSecond) {
        setVelocitySetpoint(unitsPerSecond, 0);
    }

    void setVelocitySetpoint(double unitsPerSecond, int slot);

    void setVoltageOutput(double voltage);

    void setVoltageOutputIgnoringLimits(double voltage);

    void setCurrentPositionAsZero();

    void setCurrentPosition(double positionUnits);

    void setEnableSoftLimits(boolean forward, boolean reverse);

    void setEnableHardLimits(boolean forward, boolean reverse);

    void setEnableAutosetPositionValue(boolean forward, boolean reverse);

    void follow(CANDeviceId masterId, boolean opposeMasterDirection);

    void setTorqueCurrentFOC(double current);

    void setTorqueCurrentFOCVelocity(double velocity, int slot);

    void setMotionMagicConfig(MotionMagicConfigs config);

    void setVoltageConfig(VoltageConfigs config);

    void setVoltageOutputFOCOff(double voltage);

    void setVelocitySetpointNOFOC(double unitsPerSecond, int slot);

    /**
     * Return an array of BaseStatusSignal objects used by this MotorIO implementation.
     * The default implementation returns an empty array so existing implementations
     * that don't expose signals don't need to be changed.
     */
    default BaseStatusSignal[] getStatusSignals() {
        return new BaseStatusSignal[0];
    }
}
