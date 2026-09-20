package frc.lib.subsystems.real;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.drivers.CANDeviceId;
import frc.lib.subsystems.CTREStatusSignalManager;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.motorInputs.MotorInputsAutoLogged;
import frc.lib.util.BatteryLogger;
import frc.lib.util.LoggedTunableNumber;
import frc.lib.util.LoopTimingLogger;
import frc.lib.util.MotorStallDetection;
import frc.lib.util.Util;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ServoMotorSubsystem<U extends MotorIO> extends SubsystemBase {
    protected U io;
    protected MotorInputsAutoLogged inputs;
    protected double positionSetpointUnits = 0.0;
    protected double velocitySetpointUnits = 0.0;
    protected boolean disabled = false;
    protected final String unitName;

    // Tunable numbers for manual testing
    private LoggedTunableNumber subsystemkP;
    private LoggedTunableNumber subsystemkI;
    private LoggedTunableNumber subsystemkD;
    private LoggedTunableNumber subsystemkG;
    private LoggedTunableNumber subsystemkS;
    private LoggedTunableNumber subsystemkA;
    private LoggedTunableNumber subsystemkV;
    private LoggedTunableNumber subsystemVelo;
    private LoggedTunableNumber subsystemAccel;
    private LoggedTunableNumber subsystemJerk;
    private boolean firstPeriodicLoop = true;
    private final String loggingPrefixCommand;

    protected ServoMotorSubsystemConfig conf;

    public ServoMotorSubsystem(ServoMotorSubsystemConfig config, U io) {
        super(config.name);
        this.conf = config;
        this.io = io;
        this.inputs = config.unit.getInputs();
        loggingPrefixCommand = getName() + "/currentCommand";
        unitName = config.unit.getUnitName();

        subsystemkP = new LoggedTunableNumber(config.name + "/kP", config.fxConfig.Slot0.kP);
        subsystemkI = new LoggedTunableNumber(config.name + "/kI", config.fxConfig.Slot0.kI);
        subsystemkD = new LoggedTunableNumber(config.name + "/kD", config.fxConfig.Slot0.kD);
        subsystemkG = new LoggedTunableNumber(config.name + "/kG", config.fxConfig.Slot0.kG);
        subsystemkS = new LoggedTunableNumber(config.name + "/kS", config.fxConfig.Slot0.kS);
        subsystemkA = new LoggedTunableNumber(config.name + "/kA", config.fxConfig.Slot0.kA);
        subsystemkV = new LoggedTunableNumber(config.name + "/kV", config.fxConfig.Slot0.kV);
        subsystemVelo =
                new LoggedTunableNumber(config.name + "/Velo", config.fxConfig.MotionMagic.MotionMagicCruiseVelocity);
        subsystemAccel =
                new LoggedTunableNumber(config.name + "/Accel", config.fxConfig.MotionMagic.MotionMagicAcceleration);
        subsystemJerk = new LoggedTunableNumber(config.name + "/Jerk", config.fxConfig.MotionMagic.MotionMagicJerk);

        setDefaultCommand(dutyCycleCommand(() -> 0.0)
                .withName(getName() + " Default Command Neutral")
                .ignoringDisable(true));

        // Register this MotorIO + inputs with the central refresher so all CTRE signals
        // can be refreshed once per robot loop and then this.io.readInputs will be
        // called from the manager. Implementations of MotorIO should expose their
        // BaseStatusSignal[] via getStatusSignals().
        CTREStatusSignalManager.register(this.io, this.inputs);

        System.out.println("======== " + getName() + " Online ========");
    }

    @Override
    public void periodic() {
        // double timestamp = RobotTime.getTimestampSeconds();
        LoopTimingLogger.startTiming(getName());

        // IO refresh is handled centrally by CTREStatusSignalManager.refreshAndReadAll()
        // which is called once per robot loop from Robot.robotPeriodic. Here we only
        // process the inputs that have already been populated by the manager.
        Logger.processInputs(getName(), inputs);

        boolean subsystemkPChanged = subsystemkP.hasChanged(hashCode());
        boolean subsystemkIChanged = subsystemkI.hasChanged(hashCode());
        boolean subsystemkDChanged = subsystemkD.hasChanged(hashCode());
        boolean subsystemkGChanged = subsystemkG.hasChanged(hashCode());
        boolean subsystemkSChanged = subsystemkS.hasChanged(hashCode());
        boolean subsystemkAChanged = subsystemkA.hasChanged(hashCode());
        boolean subsystemkVChanged = subsystemkV.hasChanged(hashCode());
        boolean subsystemVeloChanged = subsystemVelo.hasChanged(hashCode());
        boolean subsystemAccelChanged = subsystemAccel.hasChanged(hashCode());
        boolean subsystemJerkChanged = subsystemJerk.hasChanged(hashCode());

        if (firstPeriodicLoop) {
            firstPeriodicLoop = false;
        } else if (subsystemkPChanged
                || subsystemkIChanged
                || subsystemkDChanged
                || subsystemkGChanged
                || subsystemkSChanged
                || subsystemkAChanged
                || subsystemkVChanged
                || subsystemVeloChanged
                || subsystemAccelChanged
                || subsystemJerkChanged) {
            System.out.println("Updating PIDFFMM for " + getName() + " because:\n"
                    + (subsystemkPChanged ? " kP changed\n" : "")
                    + (subsystemkIChanged ? " kI changed\n" : "")
                    + (subsystemkDChanged ? " kD changed\n" : "")
                    + (subsystemkGChanged ? " kG changed\n" : "")
                    + (subsystemkSChanged ? " kS changed\n" : "")
                    + (subsystemkAChanged ? " kA changed\n" : "")
                    + (subsystemkVChanged ? " kV changed\n" : "")
                    + (subsystemVeloChanged ? " Velo changed\n" : "")
                    + (subsystemAccelChanged ? " Accel changed\n" : "")
                    + (subsystemJerkChanged ? " Jerk changed\n" : ""));
            io.updatePIDFFMM(
                    subsystemkP.get(),
                    subsystemkI.get(),
                    subsystemkD.get(),
                    subsystemkG.get(),
                    subsystemkS.get(),
                    subsystemkA.get(),
                    subsystemkV.get(),
                    subsystemVelo.get(),
                    subsystemAccel.get(),
                    subsystemJerk.get());
        }

        reportPowerUsage();

        // Logger.recordOutput(getName() + "/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
        Logger.recordOutput(
                loggingPrefixCommand,
                (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
        LoopTimingLogger.endTiming(getName());
    }

    public CANDeviceId getCANDeviceID() {
        return conf.talonCANID;
    }

    protected void setMotionMagicConfig(MotionMagicConfigs config) {
        io.setMotionMagicConfig(config);
    }

    protected void setOpenLoopDutyCycleImpl(double dutyCycle) {
        Logger.recordOutput(getName() + "/API/setOpenLoopDutyCycle/dutyCycle", dutyCycle);
        io.setOpenLoopDutyCycle(dutyCycle);
    }

    protected void setVoltageImpl(double voltage) {
        Logger.recordOutput(getName() + "/API/setVoltageImpl/SetVolts", voltage);
        io.setVoltageOutput(voltage);
    }

    protected void setVoltageIgnoringLimitsImpl(double voltage) {
        Logger.recordOutput(getName() + "/API/setVoltageImpl/SetVolts", voltage);
        io.setVoltageOutputIgnoringLimits(voltage);
    }

    protected void setVoltageOutputFOCOffImpl(double voltage) {
        Logger.recordOutput(getName() + "/API/setVoltageImpl/SetVoltsFOCOff", voltage);
        io.setVoltageOutputFOCOff(voltage);
    }

    protected void setPositionSetpointImpl(double units) {
        positionSetpointUnits = units;
        Logger.recordOutput(getName() + "/API/setPositionSetpointImp/Set" + unitName, units);
        io.setPositionSetpoint(units);
    }

    protected void setNeutralModeImpl(NeutralModeValue mode) {
        Logger.recordOutput(getName() + "/API/setNeutralModeImpl/Mode", mode);
        io.setNeutralMode(mode);
    }

    protected void setMotionMagicSetpointImpl(double units, int slot) {
        positionSetpointUnits = units;
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImp/Set" + unitName, units);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImp/Slot", slot);
        io.setMotionMagicSetpoint(units, slot);
    }

    protected void setMotionMagicSetpointImpl(double units, MotionMagicConfigs config, int slot) {
        positionSetpointUnits = units;
        double velocity = config.MotionMagicCruiseVelocity;
        double acceleration = config.MotionMagicAcceleration;
        double jerk = config.MotionMagicJerk;

        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Set" + unitName, units);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Velocity", velocity);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Accel", acceleration);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Jerk", jerk);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Slot", slot);
        io.setMotionMagicSetpoint(units, velocity, acceleration, jerk, slot);
    }

    protected void setMotionMagicSetpointImpl(double units, MotionMagicConfigs config, double feedfowards, int slot) {
        positionSetpointUnits = units;
        double velocity = config.MotionMagicCruiseVelocity;
        double acceleration = config.MotionMagicAcceleration;
        double jerk = config.MotionMagicJerk;

        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Set" + unitName, units);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Velocity", velocity);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Accel", acceleration);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Jerk", jerk);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Slot", slot);
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImpDynamic/Feedforwards", feedfowards);
        io.setMotionMagicSetpoint(units, velocity, acceleration, jerk, slot, feedfowards);
    }

    protected void setVelocitySetpointImpl(double unitsPerSecond, int slot) {
        velocitySetpointUnits = unitsPerSecond;
        Logger.recordOutput(getName() + "/API/setVelocitySetpointImpl/Set" + unitName + "PerSecond", unitsPerSecond);
        io.setVelocitySetpoint(unitsPerSecond, slot);
    }

    protected void setVelocitySetpointNOFOCImpl(double unitsPerSecond, int slot) {
        velocitySetpointUnits = unitsPerSecond;
        Logger.recordOutput(
                getName() + "/API/setVelocitySetpointImpl/SetNOFOC" + unitName + "PerSecond", unitsPerSecond);
        io.setVelocitySetpointNOFOC(unitsPerSecond, slot);
    }

    public double getPosition() {
        return inputs.getPosition();
    }

    public double getVelocity() {
        return inputs.getVelocity();
    }

    public double getCurrentStator() {
        return inputs.getCurrentStator();
    }

    public double getPositionSetpointUnits() {
        return positionSetpointUnits;
    }

    public double getVelocitySetpointUnitsPerSecond() {
        return velocitySetpointUnits;
    }

    public Command setMotionMagicConfigCommand(MotionMagicConfigs configs) {
        // Not taking requirements is intentional here.
        return new InstantCommand(() -> setMotionMagicConfig(configs));
    }

    public Command dutyCycleCommand(DoubleSupplier dutyCycle) {
        return runEnd(
                        () -> {
                            setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
                        },
                        () -> {
                            setOpenLoopDutyCycleImpl(0.0);
                        })
                .withName(getName() + " DutyCycleControl");
    }

    public Command dutyCycleCommandNoEnd(DoubleSupplier dutyCycle) {
        return runEnd(
                        () -> {
                            setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
                        },
                        () -> {})
                .withName(getName() + " DutyCycleControl");
    }

    public Command voltageCommand(DoubleSupplier voltage) {
        return runEnd(
                        () -> {
                            setVoltageImpl(voltage.getAsDouble());
                        },
                        () -> {
                            setVoltageImpl(0.0);
                        })
                .withName(getName() + " VoltageControl");
    }

    public Command voltageIgnoringLimitsCommand(DoubleSupplier voltage) {
        return runEnd(
                        () -> {
                            setVoltageIgnoringLimitsImpl(voltage.getAsDouble());
                        },
                        () -> {
                            setVoltageIgnoringLimitsImpl(0.0);
                        })
                .withName(getName() + " VoltageControl");
    }

    public Command voltageCommandFOCOff(DoubleSupplier voltage) {
        return runEnd(
                        () -> {
                            setVoltageOutputFOCOffImpl(voltage.getAsDouble());
                        },
                        () -> {
                            setVoltageOutputFOCOffImpl(0.0);
                        })
                .withName(getName() + " VoltageControlFOCOff");
    }

    public Command velocitySetpointCommand(DoubleSupplier velocitySupplier) {
        return velocitySetpointCommand(velocitySupplier, 0);
    }

    public Command velocitySetpointCommand(DoubleSupplier velocitySupplier, int slot) {
        return runEnd(
                        () -> {
                            setVelocitySetpointImpl(velocitySupplier.getAsDouble(), slot);
                        },
                        () -> {})
                .withName(getName() + " VelocityControl");
    }

    public Command velocitySetpointNOFOCCommand(DoubleSupplier velocitySupplier, int slot) {
        return runEnd(
                        () -> {
                            setVelocitySetpointNOFOCImpl(velocitySupplier.getAsDouble(), slot);
                        },
                        () -> {})
                .withName(getName() + " VelocityControl");
    }

    public Command setCoast() {
        return startEnd(
                        () -> setNeutralModeImpl(NeutralModeValue.Coast),
                        () -> setNeutralModeImpl(NeutralModeValue.Brake))
                .withName(getName() + "CoastMode")
                .ignoringDisable(true);
    }

    public Command positionSetpointCommand(DoubleSupplier unitSupplier) {
        return runEnd(
                        () -> {
                            setPositionSetpointImpl(unitSupplier.getAsDouble());
                        },
                        () -> {})
                .withName(getName() + " positionSetpointCommand");
    }

    public Command positionSetpointUntilOnTargetCommand(DoubleSupplier unitSupplier, DoubleSupplier epsilon) {
        return new ParallelDeadlineGroup(
                new WaitUntilCommand(() ->
                        Util.epsilonEquals(unitSupplier.getAsDouble(), inputs.getPosition(), epsilon.getAsDouble())),
                positionSetpointCommand(unitSupplier));
    }

    public Command motionMagicSetpointCommand(DoubleSupplier unitSupplier, int slot) {
        return runEnd(
                        () -> {
                            setMotionMagicSetpointImpl(unitSupplier.getAsDouble(), slot);
                        },
                        () -> {})
                .withName(getName() + " motionMagicSetpointCommand");
    }

    public Command motionMagicSetpointCommand(
            DoubleSupplier unitSupplier, Supplier<MotionMagicConfigs> configSupplier, int slot) {
        return runEnd(
                        () -> {
                            setMotionMagicSetpointImpl(unitSupplier.getAsDouble(), configSupplier.get(), slot);
                        },
                        () -> {})
                .withName(getName() + " dynamicMotionMagicSetpointCommand");
    }

    public Command motionMagicSetpointCommand(
            DoubleSupplier unitSupplier,
            Supplier<MotionMagicConfigs> configSupplier,
            Supplier<Double> feedforwards,
            int slot) {
        return runEnd(
                        () -> {
                            setMotionMagicSetpointImpl(
                                    unitSupplier.getAsDouble(), configSupplier.get(), feedforwards.get(), slot);
                        },
                        () -> {})
                .withName(getName() + " dynamicMotionMagicSetpointCommand");
    }

    public Command motionMagicSetpointCommandBlocking(DoubleSupplier setpoint, double tolerance, int slot) {
        return motionMagicSetpointCommand(() -> setpoint.getAsDouble(), slot)
                .until(() -> Util.epsilonEquals(getPosition(), setpoint.getAsDouble(), tolerance));
    }

    public Command motionMagicSetpointCommandBlocking(
            DoubleSupplier setpoint, Supplier<MotionMagicConfigs> configSupplier, double tolerance, int slot) {
        return motionMagicSetpointCommand(setpoint, configSupplier, slot)
                .until(() -> Util.epsilonEquals(getPosition(), setpoint.getAsDouble(), tolerance));
    }

    public Command motionMagicSetpointCommandBlocking(
            DoubleSupplier setpoint,
            Supplier<MotionMagicConfigs> configSupplier,
            Supplier<Double> feedforwards,
            double tolerance,
            int slot) {
        return motionMagicSetpointCommand(setpoint, configSupplier, feedforwards, slot)
                .until(() -> Util.epsilonEquals(getPosition(), setpoint.getAsDouble(), tolerance));
    }

    public Command motionMagicSetpointCommand(DoubleSupplier unitSupplier) {
        return motionMagicSetpointCommand(unitSupplier, 0);
    }

    public Command motionMagicSetpointCommand(
            DoubleSupplier unitSupplier, Supplier<MotionMagicConfigs> configSupplier) {
        return motionMagicSetpointCommand(unitSupplier, configSupplier, 0);
    }

    public Command motionMagicSetpointCommandBlocking(DoubleSupplier setpoint, double tolerance) {
        return motionMagicSetpointCommandBlocking(setpoint, tolerance, 0);
    }

    public Command motionMagicSetpointCommandBlocking(
            DoubleSupplier setpoint, double tolerance, Supplier<Integer> slot) {
        return motionMagicSetpointCommandBlocking(setpoint, tolerance, slot.get());
    }

    public Command motionMagicSetpointCommandBlocking(
            DoubleSupplier setpoint, Supplier<MotionMagicConfigs> configSupplier, double tolerance) {
        return motionMagicSetpointCommandBlocking(setpoint, configSupplier, tolerance, 0);
    }

    public void setTorqueCurrentFOCImpl(double current) {
        Logger.recordOutput(getName() + "/API/setTorqueCurrentFoC/SetAmps", current);
        io.setTorqueCurrentFOC(current);
    }

    public Command setTorqueCurrentFOC(DoubleSupplier current) {
        return runEnd(
                        () -> {
                            setTorqueCurrentFOCImpl(current.getAsDouble());
                        },
                        () -> {})
                .withName(getName() + " torqueCurrentFOCCommand");
    }

    public void setTorqueCurrentFOCVelocityImpl(DoubleSupplier velocity, int slot) {
        velocitySetpointUnits = velocity.getAsDouble();
        Logger.recordOutput(
                getName() + "/API/setTorqueCurrentFoCVelocity/Set" + unitName + "PerSecond", velocitySetpointUnits);
        io.setTorqueCurrentFOCVelocity(velocitySetpointUnits, slot);
    }

    public Command setTorqueCurrentFOCVelocity(DoubleSupplier velocity) {
        return run(() -> setTorqueCurrentFOCVelocityImpl(velocity, 0));
    }

    public Command setTorqueCurrentFOCVelocity(DoubleSupplier velocity, int slot) {
        return run(() -> setTorqueCurrentFOCVelocityImpl(velocity, slot));
    }

    protected void setCurrentPositionAsZero() {
        io.setCurrentPositionAsZero();
    }

    public void setCurrentPosition(double positionUnits) {
        io.setCurrentPosition(positionUnits);
    }

    public void updateGains(
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
        io.updatePIDFFMM(kP, kI, kD, kG, kS, kA, kV, velo, accel, jerk);
    }

    public double getUnitToRotorRatio() {
        return conf.unitToRotorRatio;
    }

    public double getCurrentStatorAmps() {
        return inputs.getCurrentStator();
    }

    public double getCurrentSupplyAmps() {
        return inputs.getCurrentSupply();
    }

    protected void reportPowerUsage() {
        BatteryLogger.reportCurrentUsage(getName(), inputs.getCurrentSupply());
    }

    public boolean isMotorStalled(double currentLimitAmps, double velocityLimitUnitsPerSecond) {
        return MotorStallDetection.isMotorStalled(
                inputs.getCurrentStator(), inputs.getVelocity(), currentLimitAmps, velocityLimitUnitsPerSecond);
    }

    /**
     * Check if this subsystem is disabled in the current robot configuration.
     *
     * @return true if the subsystem is disabled
     */
    public boolean isDisabled() {
        return disabled;
    }

    /**
     * Check if this subsystem is enabled in the current robot configuration.
     *
     * @return true if the subsystem is enabled
     */
    public boolean isEnabled() {
        return !disabled;
    }

    /**
     * Set whether this subsystem is disabled. Called by RobotContainer during construction.
     *
     * @param disabled true to disable the subsystem
     */
    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
        if (disabled) {
            removeDefaultCommand();
        }
    }

    public LoggedTunableNumber[] getTunables() {
        return new LoggedTunableNumber[] {};
    }

    protected Command withoutLimitsTemporarily() {
        var prev = new Object() {
            boolean fwd = false;
            boolean rev = false;
        };
        return Commands.startEnd(
                () -> {
                    prev.fwd = conf.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable;
                    prev.rev = conf.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable;
                    io.setEnableSoftLimits(false, false);
                },
                () -> {
                    io.setEnableSoftLimits(prev.fwd, prev.rev);
                });
    }
}
