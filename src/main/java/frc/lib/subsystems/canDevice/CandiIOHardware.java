package frc.lib.subsystems.canDevice;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANdi;
import frc.lib.util.CTREUtil;

public class CandiIOHardware implements CandiIO {

    protected final CANdi candi;
    protected CandiConfig config;
    private final BaseStatusSignal[] signals;

    private StatusSignal<Boolean> s1ClosedSignal;
    private StatusSignal<Boolean> s2ClosedSignal;

    public CandiIOHardware(CandiConfig config) {
        candi = new CANdi(config.CANID.getDeviceNumber(), config.CANID.getBus());
        this.config = config;

        CTREUtil.applyConfiguration(candi, this.config.config);
        s1ClosedSignal = candi.getS1Closed();
        s2ClosedSignal = candi.getS2Closed();

        signals = new BaseStatusSignal[] {s1ClosedSignal, s2ClosedSignal};

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
    public void readInputs(CandiInputs inputs) {
        inputs.s1Closed = s1ClosedSignal.getValue();
        inputs.s2Closed = s2ClosedSignal.getValue();
        inputs.isConnected = candi.isConnected();
    }

    @Override
    public void updateFrequency(double hz) {
        BaseStatusSignal.setUpdateFrequencyForAll(hz, signals);
    }
}
