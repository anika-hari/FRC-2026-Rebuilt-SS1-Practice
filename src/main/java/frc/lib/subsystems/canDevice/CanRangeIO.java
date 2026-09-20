package frc.lib.subsystems.canDevice;

import com.ctre.phoenix6.BaseStatusSignal;

public interface CanRangeIO {
    void readInputs(CanRangeInputs inputs);

    void updateFrequency(double hz);

    String getName();

    /**
     * Expose status signals for centralized refresh. Default: none.
     */
    default BaseStatusSignal[] getStatusSignals() {
        return new BaseStatusSignal[0];
    }
}
