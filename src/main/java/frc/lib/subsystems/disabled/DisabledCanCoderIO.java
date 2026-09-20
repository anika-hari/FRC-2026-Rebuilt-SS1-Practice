package frc.lib.subsystems.disabled;

import frc.lib.subsystems.canDevice.CanCoderIO;
import frc.lib.subsystems.canDevice.CanCoderInputs;

/**
 * A no-op implementation of CanCoderIO for disabled subsystems. Does not initialize any hardware,
 * preventing CAN errors when hardware is not present.
 */
public class DisabledCanCoderIO implements CanCoderIO {

    @Override
    public void readInputs(CanCoderInputs inputs) {
        // No-op: inputs remain at default values
    }

    @Override
    public void updateFrequency(double hz) {
        // No-op
    }

    @Override
    public void setMagnetOffset(double offset) {
        // No-op
    }

    @Override
    public void printAbsoluteRotations() {
        // No-op
    }

    @Override
    public double getAbsolutePositionSlow() {
        return 0.0;
    }

    @Override
    public void setPosition(double position) {
        // No-op
    }
}
