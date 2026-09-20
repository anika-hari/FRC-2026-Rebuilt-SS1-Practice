package frc.lib.subsystems.simulation;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.lib.subsystems.real.ServoMotorSubsystemWithCanCoderConfig;
import java.util.function.Supplier;

public class SimTalonFXWithCancoder extends SimTalonFXIO {
    ServoMotorSubsystemWithCanCoderConfig canCoderConfig;

    public SimTalonFXWithCancoder(ServoMotorSubsystemWithCanCoderConfig config) {
        super(
                config,
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                config.motorType.getDCMotor(1), config.momentOfInertia, 1.0 / config.ratioForSim),
                        config.motorType.getDCMotor(1),
                        0.001,
                        0.001));
        this.canCoderConfig = config;
    }

    @Override
    public Supplier<SimCanCoderIO.SimCanCoderState> getSupplierForCancoder(ServoMotorSubsystemWithCanCoderConfig c) {
        return () -> {
            var a = new SimCanCoderIO.SimCanCoderState();
            double ratio = this.canCoderConfig.ratioForSim / this.canCoderConfig.cancoderUnitsForSim;
            a.positionRotations = lastRotations.get() * ratio;
            a.velocityRotations = lastRPS.get() * ratio;
            return a;
        };
    }

    @Override
    protected double getSimRatio() {
        return canCoderConfig.ratioForSim;
    }
}
