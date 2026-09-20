package frc.lib.subsystems.simulation;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.subsystems.canDevice.CanCoderConfig;
import frc.lib.subsystems.canDevice.CanCoderIOHardware;
import frc.lib.subsystems.canDevice.CanCoderInputs;
import java.util.function.Supplier;

public class SimCanCoderIO extends CanCoderIOHardware {
    public static class SimCanCoderState {
        public double positionRotations = Double.NaN;
        public double velocityRotations = Double.NaN;
    }

    protected CANcoderSimState simState;
    protected Supplier<SimCanCoderState> supplier;

    public SimCanCoderIO(CanCoderConfig config, Supplier<SimCanCoderState> supplier) {
        super(config);
        this.simState = this.canCoder.getSimState();
        this.supplier = supplier;
        this.canCoder.setPosition(0.0);
    }

    @Override
    public void readInputs(CanCoderInputs inputs) {
        double invertMultiplier =
                config.config.MagnetSensor.SensorDirection == SensorDirectionValue.CounterClockwise_Positive
                        ? +1.0
                        : -1.0;
        var suppliedState = supplier.get();
        this.simState.setSupplyVoltage(RobotController.getBatteryVoltage());
        this.simState.setRawPosition(invertMultiplier * suppliedState.positionRotations);
        this.simState.setVelocity(invertMultiplier * suppliedState.velocityRotations);

        super.readInputs(inputs);
    }
}
