package frc.lib.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.drivers.CANDeviceId;
import frc.lib.subsystems.motorInputs.MotorInputs;
import frc.lib.subsystems.real.ServoMotorSubsystemConfig;
import frc.lib.util.CTREUtil;
import frc.lib.util.PhoenixUtil;
import frc.robot.Robot;

public class TalonFXIO implements MotorIO {
    protected final TalonFX talon;
    protected final ServoMotorSubsystemConfig config;

    protected final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
    private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0);
    private final VoltageOut voltageControl = new VoltageOut(0.0);
    private final VoltageOut voltageIgnoringLimitsControl = new VoltageOut(0.0).withIgnoreSoftwareLimits(true);
    private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);
    private final MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0.0);
    private final DynamicMotionMagicVoltage dynamicMotionMagicVoltage = new DynamicMotionMagicVoltage(0.0, 0.0, 0.0);
    private final Follower followerControl = new Follower(0, MotorAlignmentValue.Aligned);
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0.0);
    private final VoltageOut voltageControlFOCOff = new VoltageOut(0.0).withEnableFOC(false);
    private final VelocityVoltage velocityVoltageControlNOFOC = new VelocityVoltage(0.0).withEnableFOC(false);

    private final StatusSignal<Angle> positionSignal;
    protected final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentStatorSignal;
    private final StatusSignal<Current> currentSupplySignal;
    private final StatusSignal<Angle> rawRotorPositionSignal;
    private final StatusSignal<Double> closedLoopErrorSignal;

    protected final BaseStatusSignal[] signals;

    public TalonFXIO(ServoMotorSubsystemConfig config) {
        this.config = config;
        talon = new TalonFX(config.talonCANID.getDeviceNumber(), config.talonCANID.getBus());

        // Current limits and ramp rates do not perform well in sim.
        if (Robot.isSimulation()) {
            this.config.fxConfig.CurrentLimits = new CurrentLimitsConfigs();
            this.config.fxConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs();
            this.config.fxConfig.OpenLoopRamps = new OpenLoopRampsConfigs();
        }

        CTREUtil.applyConfiguration(talon, this.config.fxConfig);

        positionSignal = talon.getPosition();
        velocitySignal = talon.getVelocity();
        voltageSignal = talon.getMotorVoltage();
        currentStatorSignal = talon.getStatorCurrent();
        currentSupplySignal = talon.getSupplyCurrent();
        rawRotorPositionSignal = talon.getRotorPosition();
        closedLoopErrorSignal = talon.getClosedLoopError();

        signals = new BaseStatusSignal[] {
            positionSignal,
            velocitySignal,
            voltageSignal,
            currentStatorSignal,
            currentSupplySignal,
            rawRotorPositionSignal,
            closedLoopErrorSignal
        };

        CTREUtil.tryUntilOK(
                () -> BaseStatusSignal.setUpdateFrequencyForAll(this.config.kSignalUpdateHertz, signals),
                talon.getDeviceID());
        CTREUtil.tryUntilOK(() -> talon.optimizeBusUtilization(), talon.getDeviceID());
    }

    /**
     * Expose status signals for centralized refresh.
     */
    @Override
    public BaseStatusSignal[] getStatusSignals() {
        return signals;
    }

    private double rotorToUnits(double rotor) {
        return (rotor * config.unitToRotorRatio) + config.unitToRotorOffsetUnits;
    }

    private double rotorToUnitsRaw(double rotor) {
        return (rotor * config.unitToRotorRatio);
    }

    private double clampPosition(double units) {
        return unitsToRotor(MathUtil.clamp(units, config.kMinPositionUnits, config.kMaxPositionUnits));
    }

    public double unitsToRotor(double units) {
        return (units - config.unitToRotorOffsetUnits) / config.unitToRotorRatio;
    }

    public double unitsToRotorRaw(double units) {
        return (units) / config.unitToRotorRatio;
    }

    @Override
    public void readInputs(MotorInputs inputs) {
        // NOTE: refreshAll is now handled centrally by CTREStatusSignalManager to
        // allow a single refresh call per robot loop for all devices.

        inputs.setPosition(rotorToUnits(positionSignal.getValueAsDouble()));
        inputs.setVelocity(rotorToUnitsRaw(velocitySignal.getValueAsDouble()));
        inputs.setVolts(voltageSignal.getValueAsDouble());
        inputs.setCurrentStator(currentStatorSignal.getValueAsDouble());
        inputs.setCurrentSupply(currentSupplySignal.getValueAsDouble());
        inputs.setRotorPosition(rawRotorPositionSignal.getValueAsDouble());
        inputs.setClosedLoopError(closedLoopErrorSignal.getValueAsDouble());
    }

    @Override
    public void setOpenLoopDutyCycle(double dutyCycle) {
        talon.setControl(dutyCycleControl.withOutput(dutyCycle));
    }

    @Override
    public void setPositionSetpoint(double units) {
        talon.setControl(positionVoltageControl.withPosition(clampPosition(units)));
    }

    @Override
    public void setMotionMagicConfig(MotionMagicConfigs config) {
        this.config.fxConfig.MotionMagic = config;
        CTREUtil.applyConfiguration(talon, this.config.fxConfig.MotionMagic);
    }

    @Override
    public void setMotionMagicSetpoint(double units, int slot) {
        talon.setControl(
                motionMagicPositionControl.withPosition(clampPosition(units)).withSlot(slot));
    }

    @Override
    public void setMotionMagicSetpoint(
            double units, double velocity, double acceleration, double jerk, int slot, double feedfowards) {
        talon.setControl(dynamicMotionMagicVoltage
                .withPosition(clampPosition(units))
                .withAcceleration(acceleration)
                .withJerk(jerk)
                .withVelocity(velocity)
                .withSlot(slot)
                .withFeedForward(feedfowards));
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
        var motorConfig = new TalonFXConfiguration();
        talon.getConfigurator().refresh(motorConfig);
        motorConfig.Slot0.kP = kP;
        motorConfig.Slot0.kI = kI;
        motorConfig.Slot0.kD = kD;
        motorConfig.Slot0.kG = kG;
        motorConfig.Slot0.kS = kS;
        motorConfig.Slot0.kA = kA;
        motorConfig.Slot0.kV = kV;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = velo;
        motorConfig.MotionMagic.MotionMagicAcceleration = accel;
        motorConfig.MotionMagic.MotionMagicJerk = jerk;
        PhoenixUtil.tryUntilOk(5, () -> talon.getConfigurator().apply(motorConfig, 0.050));
    }

    @Override
    public void setNeutralMode(NeutralModeValue mode) {
        config.fxConfig.MotorOutput.NeutralMode = mode;
        CTREUtil.applyConfiguration(talon, config.fxConfig);
    }

    @Override
    public void setEnableSoftLimits(boolean fwd, boolean rev) {
        config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = fwd;
        config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = rev;
        CTREUtil.applyConfiguration(talon, config.fxConfig);
    }

    @Override
    public void setEnableHardLimits(boolean fwd, boolean rev) {
        config.fxConfig.HardwareLimitSwitch.ForwardLimitEnable = fwd;
        config.fxConfig.HardwareLimitSwitch.ReverseLimitEnable = rev;
        CTREUtil.applyConfiguration(talon, config.fxConfig.HardwareLimitSwitch);
    }

    @Override
    public void setEnableAutosetPositionValue(boolean fwd, boolean rev) {
        config.fxConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = fwd;
        config.fxConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = rev;
        config.fxConfig.HardwareLimitSwitch.ForwardLimitEnable = fwd;
        config.fxConfig.HardwareLimitSwitch.ReverseLimitEnable = rev;
        CTREUtil.applyConfiguration(talon, config.fxConfig.HardwareLimitSwitch);
    }

    @Override
    public void setVelocitySetpoint(double unitsPerSecond, int slot) {
        talon.setControl(velocityVoltageControl
                .withVelocity(unitsToRotorRaw(unitsPerSecond))
                .withSlot(slot));
    }

    @Override
    public void setVelocitySetpointNOFOC(double unitsPerSecond, int slot) {
        talon.setControl(velocityVoltageControlNOFOC
                .withVelocity(unitsToRotorRaw(unitsPerSecond))
                .withSlot(slot));
    }

    @Override
    public void setCurrentPositionAsZero() {
        setCurrentPosition(0.0);
    }

    @Override
    public void setCurrentPosition(double positionUnits) {
        talon.setPosition(unitsToRotor(positionUnits));
    }

    @Override
    public void setVoltageOutput(double voltage) {
        talon.setControl(voltageControl.withOutput(voltage));
    }

    @Override
    public void setVoltageOutputIgnoringLimits(double voltage) {
        talon.setControl(voltageIgnoringLimitsControl.withOutput(voltage));
    }

    @Override
    public void setVoltageOutputFOCOff(double voltage) {
        talon.setControl(voltageControlFOCOff.withOutput(voltage));
    }

    @Override
    public void follow(CANDeviceId masterId, boolean opposeMasterDirection) {
        CTREUtil.tryUntilOK(
                () -> talon.setControl(followerControl
                        .withLeaderID(masterId.getDeviceNumber())
                        .withMotorAlignment(
                                opposeMasterDirection ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned)),
                this.config.talonCANID.getDeviceNumber());
    }

    @Override
    public void setTorqueCurrentFOC(double current) {
        talon.setControl(torqueCurrentFOC.withOutput(current));
    }

    @Override
    public void setTorqueCurrentFOCVelocity(double unitsPerSecond, int slot) {
        talon.setControl(velocityTorqueCurrentFOC
                .withVelocity(unitsToRotorRaw(unitsPerSecond))
                .withSlot(slot));
    }

    @Override
    public void setVoltageConfig(VoltageConfigs inputConfig) {
        CTREUtil.applyConfigurationNonBlocking(talon, inputConfig);
    }
}
