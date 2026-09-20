package frc.lib.subsystems;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * MotorIO interface for hardware abstraction. Implementations may expose status signals
 * so a central refresher can update all signals once per robot loop.
 */
public interface FlywheelIO extends MotorIO {
    public StatusSignal<AngularVelocity> getVelocitySignal();
}
