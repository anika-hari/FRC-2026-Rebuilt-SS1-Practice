package frc.lib.subsystems.disabled;

import frc.lib.subsystems.canDevice.CandiIO;
import frc.lib.subsystems.canDevice.CandiInputs;

/**
 * A no-op implementation of CandiIO for disabled subsystems. Does not initialize any hardware,
 * preventing CAN errors when hardware is not present.
 */
public class DisabledCandiIO implements CandiIO {

    @Override
    public void readInputs(CandiInputs inputs) {
        // No-op: set default values indicating not connected
        inputs.isConnected = false;
        inputs.s1Closed = false;
        inputs.s2Closed = false;
    }

    @Override
    public void updateFrequency(double hz) {
        // No-op
    }
}
