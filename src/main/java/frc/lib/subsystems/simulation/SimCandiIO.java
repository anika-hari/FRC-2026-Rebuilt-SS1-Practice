package frc.lib.subsystems.simulation;

import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import com.ctre.phoenix6.sim.CANdiSimState;
import frc.lib.subsystems.canDevice.CandiConfig;
import frc.lib.subsystems.canDevice.CandiIOHardware;
import frc.lib.subsystems.canDevice.CandiInputs;
import java.util.function.Supplier;

public class SimCandiIO extends CandiIOHardware {

    public static class SimCandiState {
        public boolean s1Closed;
        public boolean s2Closed;
    }

    protected CANdiSimState simState;
    protected Supplier<SimCandiState> supplier;

    public SimCandiIO(CandiConfig config, Supplier<SimCandiState> supplier) {
        super(config);
        this.simState = this.candi.getSimState();
        this.supplier = supplier;

        // Initialize both inputs as Low (not closed)
        simState.setS1State(S1StateValue.Low);
        simState.setS2State(S2StateValue.Low);
    }

    @Override
    public void readInputs(CandiInputs inputs) {
        var supplied = supplier.get();

        simState.setS1State(supplied.s1Closed ? S1StateValue.High : S1StateValue.Low);

        simState.setS2State(supplied.s2Closed ? S2StateValue.High : S2StateValue.Low);

        super.readInputs(inputs);
    }
}
