package frc.lib.subsystems.disabled;

import frc.lib.subsystems.canDevice.CanRangeIO;
import frc.lib.subsystems.canDevice.CanRangeInputs;

/**
 * A no-op implementation of CanRangeIO for disabled subsystems. Does not initialize any hardware,
 * preventing CAN errors when hardware is not present.
 */
public class DisabledCanRangeIO implements CanRangeIO {

    @Override
    public void readInputs(CanRangeInputs inputs) {
        // No-op: set default values indicating not connected
        inputs.isConnected = false;
        inputs.isTripped = false;
    }

    @Override
    public void updateFrequency(double hz) {
        // No-op
    }

    @Override
    public String getName() {
        return "";
    }
}
