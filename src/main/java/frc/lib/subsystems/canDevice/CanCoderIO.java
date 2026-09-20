package frc.lib.subsystems.canDevice;

import com.ctre.phoenix6.BaseStatusSignal;

public interface CanCoderIO {
    void readInputs(CanCoderInputs inputs);

    void updateFrequency(double hz);

    void setMagnetOffset(double offset);

    void printAbsoluteRotations();

    double getAbsolutePositionSlow();

    void setPosition(double position);

    /**
     * Expose status signals for centralized refresh. Default: none.
     */
    default BaseStatusSignal[] getStatusSignals() {
        return new BaseStatusSignal[0];
    }
}
