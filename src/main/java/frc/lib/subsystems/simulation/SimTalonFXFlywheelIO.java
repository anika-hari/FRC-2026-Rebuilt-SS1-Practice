package frc.lib.subsystems.simulation;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.lib.subsystems.TalonFXFlywheelIO;
import frc.lib.subsystems.real.FlywheelSubsystemConfig;
import frc.lib.subsystems.real.ServoMotorSubsystemWithCanCoderConfig;
import frc.lib.subsystems.real.ServoMotorSubsystemWithFollowersConfig.FollowerConfig;
import frc.lib.time.RobotTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SimTalonFXFlywheelIO extends TalonFXFlywheelIO {
    // Single shared Notifier for all SimTalonFXIO instances to prevent thread starvation
    private static final List<SimTalonFXFlywheelIO> instances = new ArrayList<>();
    private static Notifier sharedSimNotifier = null;

    protected DCMotorSim sim;
    protected double lastUpdateTimestamp = 0.0;
    protected Optional<Double> overrideRPS = Optional.empty();
    protected Optional<Double> overridePos = Optional.empty();

    // Used to handle mechanisms that wrap.
    protected boolean invertVoltage = false;

    protected AtomicReference<Double> lastRotations = new AtomicReference<>((double) 0.0);
    protected AtomicReference<Double> lastRPS = new AtomicReference<>((double) 0.0);

    protected double getSimRatio() {
        return config.unitToRotorRatio;
    }

    public SimTalonFXFlywheelIO(FlywheelSubsystemConfig config) {
        this(
                config,
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                config.motorType.getDCMotor(1), config.momentOfInertia, 1.0 / config.unitToRotorRatio),
                        config.motorType.getDCMotor(1),
                        0.001,
                        0.001));
    }

    public SimTalonFXFlywheelIO(FollowerConfig config) {
        this(
                config,
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                config.motorType.getDCMotor(1), config.momentOfInertia, 1.0 / config.unitToRotorRatio),
                        config.motorType.getDCMotor(1),
                        0.001,
                        0.001));
    }

    public SimTalonFXFlywheelIO(FollowerConfig config, DCMotorSim sim) {
        super(config);
        this.sim = sim;
        var simState = talon.getSimState();
        simState.Orientation = (config.fxConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive)
                ? ChassisReference.Clockwise_Positive
                : ChassisReference.CounterClockwise_Positive;
        simState.setMotorType(config.motorType.getSimMotorType());
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        synchronized (instances) {
            instances.add(this);
            if (sharedSimNotifier == null) {
                sharedSimNotifier = new Notifier(() -> {
                    double totalCurrentDraw = 0.0;
                    synchronized (instances) {
                        for (SimTalonFXFlywheelIO inst : instances) {
                            inst.updateSimState();
                            totalCurrentDraw += inst.getSimCurrentDraw();
                        }
                    }
                    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentDraw));
                });
                sharedSimNotifier.startPeriodic(0.005);
            }
        }
    }

    public SimTalonFXFlywheelIO(FlywheelSubsystemConfig config, DCMotorSim sim) {
        super(config);
        this.sim = sim;
        var simState = talon.getSimState();
        simState.Orientation = (config.fxConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive)
                ? ChassisReference.Clockwise_Positive
                : ChassisReference.CounterClockwise_Positive;
        simState.setMotorType(config.motorType.getSimMotorType());
        /* Run simulation at a faster rate so PID gains behave more reasonably */
        synchronized (instances) {
            instances.add(this);
            if (sharedSimNotifier == null) {
                sharedSimNotifier = new Notifier(() -> {
                    double totalCurrentDraw = 0.0;
                    synchronized (instances) {
                        for (SimTalonFXFlywheelIO inst : instances) {
                            inst.updateSimState();
                            totalCurrentDraw += inst.getSimCurrentDraw();
                        }
                    }
                    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrentDraw));
                });
                sharedSimNotifier.startPeriodic(0.005);
            }
        }
    }

    // Need to use rad of the mechanism itself.
    public void setPositionRad(double rad) {
        sim.setAngle((config.fxConfig.MotorOutput.Inverted == InvertedValue.Clockwise_Positive ? -1.0 : 1.0) * rad);
        Logger.recordOutput(config.name + "/Sim/setPositionRad", rad);
    }

    protected double addFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }

    public Supplier<SimCanCoderIO.SimCanCoderState> getSupplierForCancoder(ServoMotorSubsystemWithCanCoderConfig c) {
        return () -> {
            var a = new SimCanCoderIO.SimCanCoderState();
            a.positionRotations = lastRotations.get() * c.getCanCodertoRotorRatio();
            a.velocityRotations = lastRPS.get() * c.getCanCodertoRotorRatio();
            return a;
        };
    }

    public void setInvertVoltage(boolean invertVoltage) {
        this.invertVoltage = invertVoltage;
    }

    protected double getSimCurrentDraw() {
        return sim.getCurrentDrawAmps();
    }

    protected void updateSimState() {
        var simState = talon.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double simVoltage = addFriction(simState.getMotorVoltage(), 0.25);
        simVoltage = (invertVoltage) ? -simVoltage : simVoltage;
        sim.setInput(simVoltage);

        double timestamp = RobotTime.getTimestampSeconds();
        sim.update(timestamp - lastUpdateTimestamp);
        lastUpdateTimestamp = timestamp;

        overridePos.ifPresent(aDouble -> sim.setAngle(aDouble));

        // Mutate rotor position
        double simPositionRads = sim.getAngularPositionRad();
        double rotorPosition = Units.radiansToRotations(simPositionRads) / getSimRatio();
        lastRotations.set(rotorPosition);
        simState.setRawRotorPosition(rotorPosition);

        // Mutate rotor vel
        double rotorVel = Units.radiansToRotations(sim.getAngularVelocityRadPerSec()) / getSimRatio();
        lastRPS.set(rotorVel);
        simState.setRotorVelocity(overrideRPS.isEmpty() ? rotorVel : overrideRPS.get());
    }

    public void overrideRPS(Optional<Double> rps) {
        overrideRPS = rps;
    }

    // This is the position in radians of the mechanism.
    public void overridePos(Optional<Double> pos) {
        overridePos = pos;
    }

    public double getIntendedRPS() {
        return lastRPS.get();
    }
}
