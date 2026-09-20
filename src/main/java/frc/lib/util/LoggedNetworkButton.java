package frc.lib.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.units.Units;
import java.util.stream.Stream;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

public class LoggedNetworkButton extends LoggedNetworkInput {
    private final String key;
    private final BooleanSubscriber subscriber;
    private final boolean defaultValue;
    private boolean value;

    private final LoggableInputs inputs = new LoggableInputs() {
        public void toLog(LogTable table) {
            table.put(key, value);
        }

        public void fromLog(LogTable table) {
            value = table.get(key, defaultValue);
        }
    };

    public LoggedNetworkButton(String key) {
        this(key, false);
    }

    public LoggedNetworkButton(String key, boolean defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.value = defaultValue;
        this.subscriber = NetworkTableInstance.getDefault()
                .getBooleanTopic("/Dashboard/" + key)
                .subscribe(
                        defaultValue,
                        PubSubOption.periodic(Units.Milliseconds.of(20).in(Seconds)),
                        PubSubOption.sendAll(true));
        periodic();
        Logger.registerDashboardInput(this);
    }

    public boolean get() {
        return value;
    }

    @Override
    public void periodic() {
        if (!Logger.hasReplaySource()) {
            // If a button is pressed and released in a single loop, treat it as if it was pressed this
            // loop
            var values = subscriber.readQueue();
            value = values.length > 0 ? Stream.of(values).anyMatch(val -> val.value) : subscriber.get();
        }

        Logger.processInputs(prefix, inputs);
    }
}
