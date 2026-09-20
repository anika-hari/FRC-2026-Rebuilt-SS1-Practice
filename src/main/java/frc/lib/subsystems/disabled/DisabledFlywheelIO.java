package frc.lib.subsystems.disabled;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.drivers.CANDeviceId;
import frc.lib.subsystems.FlywheelIO;
import frc.lib.subsystems.motorInputs.MotorInputs;

/**
 * A no-op implementation of MotorIO for disabled subsystems. Does not initialize any hardware,
 * preventing CAN errors when hardware is not present.
 */
public class DisabledFlywheelIO implements FlywheelIO {

    @Override
    public void readInputs(MotorInputs inputs) {
        // No-op: inputs remain at default values (0)
    }

    @Override
    public StatusSignal<AngularVelocity> getVelocitySignal() {
        return new StatusSignal<AngularVelocity>(null, null, null);
    }

    @Override
    public void setOpenLoopDutyCycle(double dutyCycle) {
        // System.out.println("DisabledMotorIO: setOpenLoopDutyCycle called with " + dutyCycle);
    }

    @Override
    public void setPositionSetpoint(double units) {
        // System.out.println("DisabledMotorIO: setPositionSetpoint called with " + units);
    }

    @Override
    public void setMotionMagicSetpoint(double units, int slot) {
        // System.out.println("DisabledMotorIO: setMotionMagicSetpoint called with units=" + units + ", slot=" + slot);
    }

    @Override
    public void setMotionMagicSetpoint(
            double units, double velocity, double acceleration, double jerk, int slot, double feedforward) {
        // System.out.println("DisabledMotorIO: setMotionMagicSetpoint called with units=" + units + ", velocity="
        //         + velocity + ", acceleration=" + acceleration + ", jerk=" + jerk + ", slot=" + slot + ",
        // feedforward="
        //         + feedforward);
    }

    @Override
    public void updatePIDFFMM(
            double kP,
            double kI,
            double kD,
            double kG,
            double kS,
            double kA,
            double kV,
            double velo,
            double accel,
            double jerk) {
        // System.out.println("DisabledMotorIO: updatePIDFFMM called with kP=" + kP + ", kI=" + kI + ", kD=" + kD + ",
        // kG="
        //         + kG + ", kS=" + kS + ", kA=" + kA + ", kV=" + kV + ", velo=" + velo + ", accel=" + accel + ", jerk="
        //         + jerk);
    }

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        // System.out.println("DisabledMotorIO: setNeutralMode called with mode=" + mode);
    }

    @Override
    public void setVelocitySetpoint(double unitsPerSecond, int slot) {
        // System.out.println(
        //         "DisabledMotorIO: setVelocitySetpoint called with unitsPerSecond=" + unitsPerSecond + ", slot=" +
        // slot);
    }

    @Override
    public void setVoltageOutput(double voltage) {
        // System.out.println("DisabledMotorIO: setVoltageOutput called with voltage=" + voltage);
    }

    @Override
    public void setVoltageOutputIgnoringLimits(double voltage) {
        // System.out.println("DisabledMotorIO: setVoltageOutput called with voltage=" + voltage);
    }

    @Override
    public void setVoltageOutputFOCOff(double units) {
        // System.out.println("DisabledMotorIO: setVoltageOutputFOCOff called with " + units);
    }

    @Override
    public void setCurrentPositionAsZero() {
        // System.out.println("DisabledMotorIO: setCurrentPositionAsZero called");
    }

    @Override
    public void setCurrentPosition(double positionUnits) {
        // System.out.println("DisabledMotorIO: setCurrentPosition called with positionUnits=" + positionUnits);
    }

    @Override
    public void setEnableSoftLimits(boolean forward, boolean reverse) {
        // System.out.println(
        //         "DisabledMotorIO: setEnableSoftLimits called with forward=" + forward + ", reverse=" + reverse);
    }

    @Override
    public void setVelocitySetpointNOFOC(double unitsPerSecond, int slot) {
        // System.out.println(
        //         "DisabledMotorIO: setVelocitySetpoint called with unitsPerSecond=" + unitsPerSecond + ", slot=" +
        // slot);
    }

    @Override
    public void setEnableHardLimits(boolean forward, boolean reverse) {
        // System.out.println(
        //         "DisabledMotorIO: setEnableHardLimits called with forward=" + forward + ", reverse=" + reverse);
    }

    @Override
    public void setEnableAutosetPositionValue(boolean forward, boolean reverse) {
        // System.out.println("DisabledMotorIO: setEnableAutosetPositionValue called with forward=" + forward
        //         + ", reverse=" + reverse);
    }

    @Override
    public void follow(CANDeviceId masterId, boolean opposeMasterDirection) {
        // System.out.println("DisabledMotorIO: follow called with masterId=" + masterId + ", opposeMasterDirection="
        //         + opposeMasterDirection);
    }

    @Override
    public void setTorqueCurrentFOC(double current) {
        // System.out.println("DisabledMotorIO: setTorqueCurrentFOC called with current=" + current);
    }

    @Override
    public void setTorqueCurrentFOCVelocity(double velocity, int slot) {
        // No-op
    }

    @Override
    public void setMotionMagicConfig(MotionMagicConfigs config) {
        // System.out.println("DisabledMotorIO: setMotionMagicConfig called with config=" + config);
    }

    @Override
    public void setVoltageConfig(VoltageConfigs config) {
        // System.out.println("DisabledMotorIO: setVoltageConfig called with config=" + config);
    }
}
