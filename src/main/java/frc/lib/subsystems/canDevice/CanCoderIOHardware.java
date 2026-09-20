package frc.lib.subsystems.canDevice;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.util.CTREUtil;
import frc.lib.util.Util;

public class CanCoderIOHardware implements CanCoderIO {
    protected final CANcoder canCoder;
    protected CanCoderConfig config;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final BaseStatusSignal[] signals;
    double goodValues = 0.0;

    public CanCoderIOHardware(CanCoderConfig config) {
        this.config = config;

        canCoder = new CANcoder(config.CANID.getDeviceNumber(), config.CANID.getBus());

        CTREUtil.applyConfiguration(canCoder, this.config.config);
        positionSignal = canCoder.getAbsolutePosition();
        velocitySignal = canCoder.getVelocity();

        signals = new BaseStatusSignal[] {positionSignal, velocitySignal};

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, signals);
    }

    /**
     * Expose status signals for centralized refresh.
     */
    @Override
    public BaseStatusSignal[] getStatusSignals() {
        return signals;
    }

    @Override
    public void updateFrequency(double hz) {
        BaseStatusSignal.setUpdateFrequencyForAll(hz, signals);
    }

    @Override
    public void readInputs(CanCoderInputs inputs) {
        if (Double.isNaN(inputs.absolutePositionRotations)) {
            BaseStatusSignal.waitForAll(10.0, positionSignal, velocitySignal);
            goodValues++;
        }
        if (goodValues < 50) return;

        inputs.absolutePositionRotations = positionSignal.getValue().in(Rotations);
        inputs.velocityRotations = velocitySignal.getValue().in(RotationsPerSecond);
    }

    @Override
    public void setMagnetOffset(double offset) {
        MagnetSensorConfigs currentConfigs = new MagnetSensorConfigs();
        canCoder.getConfigurator().refresh(currentConfigs);
        canCoder.getConfigurator().apply(currentConfigs.withMagnetOffset(offset));
        Util.sleep(200);
    }

    @Override
    public void printAbsoluteRotations() {
        System.out.println(canCoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public double getAbsolutePositionSlow() {
        return canCoder.getAbsolutePosition().getValueAsDouble();
    }

    @Override
    public void setPosition(double position) {
        canCoder.setPosition(position);
    }
}
