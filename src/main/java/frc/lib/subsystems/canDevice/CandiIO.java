package frc.lib.subsystems.canDevice;

import com.ctre.phoenix6.BaseStatusSignal;

public interface CandiIO {
    void readInputs(CandiInputs inputs);

    void updateFrequency(double hz);

    /**
     * Expose status signals for centralized refresh. Default: none.
     */
    default BaseStatusSignal[] getStatusSignals() {
        return new BaseStatusSignal[0];
    }
}
