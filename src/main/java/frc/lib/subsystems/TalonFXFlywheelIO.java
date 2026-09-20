package frc.lib.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.subsystems.real.FlywheelSubsystemConfig;
import frc.lib.subsystems.real.ServoMotorSubsystemWithFollowersConfig.FollowerConfig;

public class TalonFXFlywheelIO extends TalonFXIO implements FlywheelIO {
    private final boolean useAutoTuneThread;

    public TalonFXFlywheelIO(FlywheelSubsystemConfig config) {
        super(config);
        useAutoTuneThread = config.useAutoTuneThread;
    }

    public TalonFXFlywheelIO(FollowerConfig config) {
        super(config);
        useAutoTuneThread = false;
    }

    /**
     * Expose status signals for centralized refresh.
     */
    @Override
    public BaseStatusSignal[] getStatusSignals() {
        if (useAutoTuneThread) {
            // Velocity signal gets refreshed
            BaseStatusSignal[] signalsWithoutVelocity = new BaseStatusSignal[signals.length - 1];
            int fillIndex = 0;
            for (BaseStatusSignal signal : signals) {
                if (signal != velocitySignal) {
                    signalsWithoutVelocity[fillIndex] = signal;
                    fillIndex += 1;
                }
            }
            return signalsWithoutVelocity;
        }
        return signals;
    }

    public StatusSignal<AngularVelocity> getVelocitySignal() {
        return velocitySignal;
    }
}
