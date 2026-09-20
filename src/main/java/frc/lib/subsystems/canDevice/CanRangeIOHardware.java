package frc.lib.subsystems.canDevice;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.CTREUtil;

public class CanRangeIOHardware implements CanRangeIO {
    protected final CANrange canRange;
    protected CanRangeConfig config;
    private final BaseStatusSignal[] signals;

    private StatusSignal<Boolean> trippedSignal;
    private StatusSignal<Distance> distanceSignal;

    public CanRangeIOHardware(CanRangeConfig config) {
        canRange = new CANrange(config.CANID.getDeviceNumber(), config.CANID.getBus());
        this.config = config;

        CTREUtil.applyConfiguration(canRange, this.config.config);
        trippedSignal = canRange.getIsDetected();
        distanceSignal = canRange.getDistance();

        signals = new BaseStatusSignal[] {trippedSignal, distanceSignal};

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
    public void readInputs(CanRangeInputs inputs) {
        inputs.isTripped = trippedSignal.getValue();
        inputs.distanceMeters = distanceSignal.getValue().in(Meters);
        inputs.isConnected = canRange.isConnected();
    }

    @Override
    public void updateFrequency(double hz) {
        BaseStatusSignal.setUpdateFrequencyForAll(hz, signals);
    }

    @Override
    public String getName() {
        return config.name;
    }
}
