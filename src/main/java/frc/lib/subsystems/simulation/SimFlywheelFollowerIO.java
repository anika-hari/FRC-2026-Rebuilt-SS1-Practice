package frc.lib.subsystems.simulation;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.subsystems.real.ServoMotorSubsystemWithFollowersConfig.FollowerConfig;

/**
 * Simulation IO for follower motors on a flywheel. Does not run its own physics —
 * instead mirrors position/velocity from the leader's shared FlywheelSim.
 */
public class SimFlywheelFollowerIO extends SimTalonFXFlywheelIO {
    private final SimFlywheelIO leader;
    private final double gearRatio;

    public SimFlywheelFollowerIO(FollowerConfig config, SimFlywheelIO leader) {
        super(config);
        this.leader = leader;
        double sensorToMechRatio = config.fxConfig.Feedback.SensorToMechanismRatio;
        this.gearRatio = (sensorToMechRatio != 0) ? (1.0 / sensorToMechRatio) : 1.0;
    }

    @Override
    protected double getSimCurrentDraw() {
        // Current is already accounted for in the leader's FlywheelSim (models N motors)
        return 0.0;
    }

    @Override
    protected void updateSimState() {
        var simState = talon.getSimState();
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Copy position and velocity from the leader's shared simulation.
        // Convert mechanism rad/s → rotor rotations/s (same conversion as leader).
        double positionRad = leader.getIntegratedPositionRad();
        double velocityRadPerSec = leader.getFlywheelVelocityRadPerSec();

        double rotorPosition = Units.radiansToRotations(positionRad) / gearRatio;
        lastRotations.set(rotorPosition);
        simState.setRawRotorPosition(rotorPosition);

        double rotorVel = Units.radiansToRotations(velocityRadPerSec) / gearRatio;
        lastRPS.set(rotorVel);
        simState.setRotorVelocity(rotorVel);
    }
}
