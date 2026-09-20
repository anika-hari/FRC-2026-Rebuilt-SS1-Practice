package frc.lib.subsystems.real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.lib.drivers.CANDeviceId;
import frc.lib.subsystems.SubsystemUnit;
import frc.lib.subsystems.TalonFXMotorType;

public class ServoMotorSubsystemConfig {
    public String name = "UNNAMED";
    public SubsystemUnit unit = SubsystemUnit.GENERIC;
    public CANDeviceId talonCANID;
    public TalonFXConfiguration fxConfig = new TalonFXConfiguration();

    // Ratio of rotor to units for this talon.  rotor * by this ratio should
    // be the units.
    // <1 is reduction, or torque goes up
    public double unitToRotorRatio = 1.0;
    public double kMinPositionUnits = Double.NEGATIVE_INFINITY;
    public double kMaxPositionUnits = Double.POSITIVE_INFINITY;
    public double kSignalUpdateHertz = 50.0;

    public double unitToRotorOffsetUnits = 0.0;

    // Moment of Inertia (KgMetersSquared) for sim
    public double momentOfInertia = 0.01;

    // Motor type for simulation (default: Kraken X60 FOC for backwards compatibility)
    public TalonFXMotorType motorType = TalonFXMotorType.KRAKEN_X60_FOC;
}
