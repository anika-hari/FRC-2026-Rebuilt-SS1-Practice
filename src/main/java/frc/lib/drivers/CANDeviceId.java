package frc.lib.drivers;

import com.ctre.phoenix6.CANBus;

// todo: String-> CANBus object

public class CANDeviceId {
    private final int deviceNumber;
    private final String busName;
    private final CANBus bus;

    public CANDeviceId(int deviceNumber, String busName, CANBus bus) {
        this.deviceNumber = deviceNumber;
        this.busName = busName;
        this.bus = bus;
    }

    public CANDeviceId(int deviceNumber, String busName) {
        this(deviceNumber, busName, new CANBus(busName));
    }

    // Use the default bus name (empty string).
    public CANDeviceId(int deviceNumber) {
        this(deviceNumber, "");
    }

    public CANDeviceId(int deviceNumber, CANBus bus) {
        this(deviceNumber, bus.getName(), bus);
    }

    public int getDeviceNumber() {
        return deviceNumber;
    }

    public String getBusName() {
        return busName;
    }

    public CANBus getBus() {
        return bus;
    }

    public boolean equals(CANDeviceId other) {
        return other.deviceNumber == deviceNumber && other.busName == busName;
    }
}
