package frc.lib.subsystems.simulation;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.subsystems.real.FlywheelSubsystemConfig;
import frc.lib.time.RobotTime;

/**
 * Simulation IO for the leader motor of a flywheel mechanism. Uses WPILib's FlywheelSim
 * with multi-motor modeling (getDCMotor(N)) to accurately simulate all motors driving
 * a single shared flywheel inertia.
 */
public class SimFlywheelIO extends SimTalonFXFlywheelIO {
    private final FlywheelSim flywheelSim;
    private final double gearRatio; // rotor:mechanism speed ratio (> 1 if mechanism faster)
    private double integratedPositionRad = 0.0;

    public SimFlywheelIO(FlywheelSubsystemConfig config) {
        // Parent constructor creates an unused DCMotorSim and registers with shared notifier
        super(config);

        // Derive gear ratio from CTRE FeedbackConfigs (single source of truth).
        // SensorToMechanismRatio = rotor_rps / mechanism_rps (e.g. 24/28 = 0.857 for speed increase)
        double sensorToMechRatio = config.fxConfig.Feedback.SensorToMechanismRatio;
        this.gearRatio = (sensorToMechRatio != 0) ? (1.0 / sensorToMechRatio) : 1.0;

        int motorCount = config.getTotalMotorCount();
        DCMotor motor = config.motorType.getDCMotor(motorCount);

        // WPILib gearing = motor turns per output turn. For speed increase (mechanism faster),
        // this is < 1. SensorToMechanismRatio is exactly this value.
        this.flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(motor, config.momentOfInertia, sensorToMechRatio), motor);
    }

    @Override
    protected double getSimCurrentDraw() {
        return flywheelSim.getCurrentDrawAmps();
    }

    @Override
    protected void updateSimState() {
        var simState = talon.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        double simVoltage = addFriction(simState.getMotorVoltage(), 0.0);
        simVoltage = invertVoltage ? -simVoltage : simVoltage;

        flywheelSim.setInputVoltage(simVoltage);

        double timestamp = RobotTime.getTimestampSeconds();
        double dt = timestamp - lastUpdateTimestamp;
        flywheelSim.update(dt);
        lastUpdateTimestamp = timestamp;

        // Integrate velocity to get position (FlywheelSim is velocity-only)
        double velocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
        integratedPositionRad += velocityRadPerSec * dt;

        overridePos.ifPresent(aDouble -> integratedPositionRad = aDouble);

        // Write to CTRE sim state.
        // Convert mechanism rad/s → rotor rotations/s:
        //   mechanism_rps = rad/s ÷ 2π, then rotor_rps = mechanism_rps ÷ gearRatio
        double rotorPosition = Units.radiansToRotations(integratedPositionRad) / gearRatio;
        lastRotations.set(rotorPosition);
        simState.setRawRotorPosition(rotorPosition);

        double rotorVel = Units.radiansToRotations(velocityRadPerSec) / gearRatio;
        lastRPS.set(rotorVel);
        simState.setRotorVelocity(overrideRPS.isEmpty() ? rotorVel : overrideRPS.get());
    }

    /** Get current velocity for followers to read. */
    public double getFlywheelVelocityRadPerSec() {
        return flywheelSim.getAngularVelocityRadPerSec();
    }

    /** Get integrated position for followers to read. */
    public double getIntegratedPositionRad() {
        return integratedPositionRad;
    }
}
