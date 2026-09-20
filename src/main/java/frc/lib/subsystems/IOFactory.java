package frc.lib.subsystems;

import edu.wpi.first.wpilibj.RobotBase;
import frc.config.ConfigConstants;
import frc.lib.drivers.CANDeviceId;
import frc.lib.subsystems.canDevice.CanCoderConfig;
import frc.lib.subsystems.canDevice.CanCoderIO;
import frc.lib.subsystems.canDevice.CanCoderIOHardware;
import frc.lib.subsystems.canDevice.CanRangeConfig;
import frc.lib.subsystems.canDevice.CanRangeIO;
import frc.lib.subsystems.canDevice.CanRangeIOHardware;
import frc.lib.subsystems.canDevice.CandiConfig;
import frc.lib.subsystems.canDevice.CandiIO;
import frc.lib.subsystems.canDevice.CandiIOHardware;
import frc.lib.subsystems.disabled.DisabledCanCoderIO;
import frc.lib.subsystems.disabled.DisabledCanRangeIO;
import frc.lib.subsystems.disabled.DisabledCandiIO;
import frc.lib.subsystems.disabled.DisabledFlywheelIO;
import frc.lib.subsystems.disabled.DisabledMotorIO;
import frc.lib.subsystems.real.FlywheelSubsystemConfig;
import frc.lib.subsystems.real.ServoMotorSubsystemConfig;
import frc.lib.subsystems.real.ServoMotorSubsystemWithCanCoderConfig;
import frc.lib.subsystems.real.ServoMotorSubsystemWithFollowersConfig.FollowerConfig;
import frc.lib.subsystems.simulation.SimCanCoderIO;
import frc.lib.subsystems.simulation.SimCanCoderIO.SimCanCoderState;
import frc.lib.subsystems.simulation.SimCanRangeIO;
import frc.lib.subsystems.simulation.SimCanRangeStateInput;
import frc.lib.subsystems.simulation.SimCandiIO;
import frc.lib.subsystems.simulation.SimCandiIO.SimCandiState;
import frc.lib.subsystems.simulation.SimFlywheelFollowerIO;
import frc.lib.subsystems.simulation.SimFlywheelIO;
import frc.lib.subsystems.simulation.SimTalonFXIO;
import frc.lib.subsystems.simulation.SimTalonFXWithCancoder;

/** Contains basic functions that are used often. */
public class IOFactory {
    private static boolean canIdDisabled(CANDeviceId canDeviceId) {
        return ConfigConstants.ROBOT_CONFIG.isCANIDDisabled(canDeviceId.getDeviceNumber());
    }

    public static <T extends ServoMotorSubsystemConfig> MotorIO motor(T config, boolean disabled) {
        if (disabled || canIdDisabled(config.talonCANID)) {
            return new DisabledMotorIO();
        }
        if (RobotBase.isSimulation()) {
            if (config instanceof ServoMotorSubsystemWithCanCoderConfig canCoderConfig) {
                return new SimTalonFXWithCancoder(canCoderConfig);
            }
            return new SimTalonFXIO(config);
        }
        return new TalonFXIO(config);
    }

    public static FlywheelIO flywheelLeaderMotor(FlywheelSubsystemConfig config, boolean disabled) {
        if (disabled || canIdDisabled(config.talonCANID)) {
            return new DisabledFlywheelIO();
        }
        if (RobotBase.isSimulation()) {
            return new SimFlywheelIO(config);
        }
        return new TalonFXFlywheelIO(config);
    }

    public static FlywheelIO flywheelFollowerMotor(FollowerConfig followers, boolean disabled, MotorIO leaderIO) {
        if (disabled || canIdDisabled(followers.talonCANID)) {
            return new DisabledFlywheelIO();
        }
        if (RobotBase.isSimulation() && leaderIO instanceof SimFlywheelIO simLeader) {
            return new SimFlywheelFollowerIO(followers, simLeader);
        }
        return new TalonFXFlywheelIO(followers);
    }

    public static <T extends CanCoderConfig> CanCoderIO canCoder(T config, boolean disabled) {
        if (disabled || canIdDisabled(config.CANID)) {
            return new DisabledCanCoderIO();
        }
        if (RobotBase.isSimulation()) {
            return new SimCanCoderIO(config, () -> new SimCanCoderState());
        }
        return new CanCoderIOHardware(config);
    }

    public static CanCoderIO canCoder(
            ServoMotorSubsystemWithCanCoderConfig motorConfig, boolean disabled, MotorIO motorIO) {
        if (disabled || canIdDisabled(motorConfig.canCoderConfig.CANID)) {
            return new DisabledCanCoderIO();
        }
        if (RobotBase.isSimulation() && motorIO instanceof SimTalonFXIO simMotorIO) {
            return new SimCanCoderIO(motorConfig.canCoderConfig, simMotorIO.getSupplierForCancoder(motorConfig));
        }
        return new CanCoderIOHardware(motorConfig.canCoderConfig);
    }

    public static <T extends CanRangeConfig> CanRangeIO canRange(T config, boolean disabled) {
        if (disabled || canIdDisabled(config.CANID)) {
            return new DisabledCanRangeIO();
        }
        if (RobotBase.isSimulation()) {
            return new SimCanRangeIO(config, () -> new SimCanRangeStateInput());
        }
        return new CanRangeIOHardware(config);
    }

    public static <T extends CandiConfig> CandiIO candi(T config, boolean disabled) {
        if (disabled || canIdDisabled(config.CANID)) {
            return new DisabledCandiIO();
        }
        if (RobotBase.isSimulation()) {
            return new SimCandiIO(config, () -> new SimCandiState());
        }
        return new CandiIOHardware(config);
    }
}
