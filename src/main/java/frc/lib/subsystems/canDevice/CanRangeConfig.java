package frc.lib.subsystems.canDevice;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import frc.lib.drivers.CANDeviceId;

public class CanRangeConfig {
    public CANDeviceId CANID;
    public CANrangeConfiguration config = new CANrangeConfiguration();
    public String name = "";
}
