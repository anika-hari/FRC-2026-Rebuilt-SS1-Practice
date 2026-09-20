package frc.lib.util.Controls.StreamDeck;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class StreamDeck extends SubsystemBase {
    private Map<StreamDeckButton, ButtonRecord> buttonMap = new HashMap<>();

    private static class ButtonRecord {
        private LoggedNetworkBoolean pressed;
        private boolean pressedPrev;
        private boolean toggled;
        private BooleanSupplier selected;
        private BooleanPublisher activePub;
        private StreamDeckButtonType type;

        ButtonRecord(
                LoggedNetworkBoolean pressed,
                Optional<BooleanSupplier> selected,
                BooleanPublisher activePub,
                StreamDeckButtonType type) {
            this.pressed = pressed;
            pressedPrev = false;
            toggled = false;
            this.selected = selected.orElse(type == StreamDeckButtonType.TOGGLE ? () -> toggled : pressed::get);
            this.activePub = activePub;
            this.type = type;
        }
    }

    @Override
    public void periodic() {
        buttonMap.values().forEach(button -> {
            if (button.type == StreamDeckButtonType.TOGGLE && button.pressed.get() && !button.pressedPrev) {
                button.toggled = !button.toggled;
            }
            button.activePub.set(button.selected.getAsBoolean());
            button.pressedPrev = button.pressed.get();
        });
    }

    public void configureDefaultButtons(Set<StreamDeckButton> buttons) {
        configureButton(config -> {
            for (StreamDeckButton button : buttons) {
                config.addDefault(button);
            }
        });
    }

    public void configureToggleButtons(Set<StreamDeckButton> buttons) {
        configureButton(config -> {
            for (StreamDeckButton button : buttons) {
                config.addToggle(button);
            }
        });
    }

    public void configureCustomButtons(Map<StreamDeckButton, BooleanSupplier> buttonMap) {
        configureButton(config -> {
            for (Map.Entry<StreamDeckButton, BooleanSupplier> entry : buttonMap.entrySet()) {
                config.add(entry.getKey(), entry.getValue());
            }
        });
    }

    private StreamDeck configureButton(Consumer<ButtonConfiguration> config) {
        ButtonConfiguration configuration = new ButtonConfiguration();
        config.accept(configuration);

        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        NetworkTable deckTable = nt.getTable("StreamDeck");
        List<String> networkTableKeys = StreamDeckButton.getNetworkTableKeys();
        configuration.buttonConfigurations.forEach((button, pair) -> {
            Optional<BooleanSupplier> selected = pair.getFirst();
            StreamDeckButtonType type = pair.getSecond();
            NetworkTable table = deckTable.getSubTable("Button/" + button.getIndex());
            List<String> dataToPublish = button.getDataToPublish();
            IntStream.range(0, Math.min(networkTableKeys.size(), dataToPublish.size()))
                    .forEach(i -> table.getStringTopic(networkTableKeys.get(i))
                            .publish()
                            .set(dataToPublish.get(i)));

            var loggedBoolean = new LoggedNetworkBoolean("StreamDeck/" + button.getKey(), false);
            buttonMap.put(
                    button,
                    new ButtonRecord(
                            loggedBoolean,
                            selected,
                            table.getBooleanTopic("Selected").publish(),
                            type));
        });

        deckTable.getIntegerTopic("LastModified").publish().set(Logger.getTimestamp());

        return this;
    }

    public Trigger button(String key) {
        for (StreamDeckButton button : buttonMap.keySet()) {
            if (button.getKey().equals(key)) {
                return this.button(button);
            }
        }
        StreamDeckAlert.warning("Stream Deck button trigger added for invalid button key " + key)
                .enable();
        return new Trigger(() -> false);
    }

    public Trigger button(int row, int column) {
        int index = StreamDeckButton.calculate_index(row, column);
        for (StreamDeckButton button : buttonMap.keySet()) {
            if (button.getIndex() == index) {
                return this.button(button);
            }
        }
        StreamDeckAlert.warning("Stream Deck button trigger added for invalid button index " + index)
                .enable();
        return new Trigger(() -> false);
    }

    public Trigger button(StreamDeckButton button) {
        if (!buttonMap.containsKey(button)) {
            StreamDeckAlert.warning("Stream Deck button trigger added for invalid button " + button.getIndex())
                    .enable();
            return new Trigger(() -> false);
        }

        return new Trigger(buttonMap.get(button).pressed::get);
    }

    // public ButtonGroup buttonGroup() {
    //   return new ButtonGroup();
    // }

    // add non-custom button without commands
    public StreamDeck addButton(StreamDeckButtonType buttonType, StreamDeckButton button) {
        assert buttonType != StreamDeckButtonType.CUSTOM;
        return this.addButton(buttonType, button, () -> false, Set.of());
    }

    // add non-custom button with one command
    public StreamDeck addButton(
            StreamDeckButtonType buttonType, StreamDeckButton button, StreamDeckCommand streamDeckCommand) {
        assert buttonType != StreamDeckButtonType.CUSTOM;
        return this.addButton(buttonType, button, () -> false, streamDeckCommand);
    }

    // add non-custom button with commands
    public StreamDeck addButton(
            StreamDeckButtonType buttonType, StreamDeckButton button, Set<StreamDeckCommand> streamDeckCommands) {
        assert buttonType != StreamDeckButtonType.CUSTOM;
        return this.addButton(buttonType, button, () -> false, streamDeckCommands);
    }

    // add custom button without commands
    public StreamDeck addButton(
            StreamDeckButtonType buttonType, StreamDeckButton button, BooleanSupplier activeSupplier) {
        assert buttonType == StreamDeckButtonType.CUSTOM;
        return this.addButton(buttonType, button, activeSupplier, Set.of());
    }

    // add custom button with one command
    public StreamDeck addButton(
            StreamDeckButtonType buttonType,
            StreamDeckButton button,
            BooleanSupplier activeSupplier,
            StreamDeckCommand streamDeckCommand) {
        assert buttonType == StreamDeckButtonType.CUSTOM;
        return this.addButton(buttonType, button, activeSupplier, Set.of(streamDeckCommand));
    }

    // add a button
    public StreamDeck addButton(
            StreamDeckButtonType buttonType,
            StreamDeckButton button,
            BooleanSupplier activeSupplier,
            Set<StreamDeckCommand> streamDeckCommands) {
        // setup button
        switch (buttonType) {
            case DISPLAY:
                this.configureCustomButtons(Map.of(button, () -> false));
                break;
            case PRESS:
                this.configureDefaultButtons(Set.of(button));
                break;
            case TOGGLE:
                this.configureToggleButtons(Set.of(button));
                break;
            case CUSTOM:
                this.configureCustomButtons(Map.of(button, activeSupplier));
                break;
        }

        Trigger buttonTrigger = this.button(button);

        // bind button triggers
        for (StreamDeckCommand sdc : streamDeckCommands) {
            // Command command =
            //     sdc.getCommand()
            //         .withName(
            //             sdc.getCommand().getName()
            //                 + " | from SD button "
            //                 + button.getKey()
            //                 + " "
            //                 + sdc.commandType.toString());
            switch (sdc.commandType) {
                case NONE:
                    continue;
                case ON_TRUE:
                    buttonTrigger.onTrue(sdc.getCommand());
                    break;
                case ON_FALSE:
                    buttonTrigger.onFalse(sdc.getCommand());
                    break;
                case WHILE_TRUE:
                    buttonTrigger.whileTrue(sdc.getCommand());
                    break;
                case WHILE_FALSE:
                    buttonTrigger.whileFalse(sdc.getCommand());
                    break;
                case TOGGLE_ON_TRUE:
                    buttonTrigger.toggleOnTrue(sdc.getCommand());
                    break;
                case TOGGLE_ON_FALSE:
                    buttonTrigger.toggleOnFalse(sdc.getCommand());
                    break;
            }
        }
        return this;
    }

    public static class StreamDeckCommand {
        private CommandType commandType;
        private Command command;

        public StreamDeckCommand(CommandType commandType, Command command) {
            this.commandType = commandType;
            this.command = command;
        }

        public Command getCommand() {
            return command;
        }
    }

    public enum CommandType {
        NONE,
        ON_TRUE,
        ON_FALSE,
        WHILE_TRUE,
        WHILE_FALSE,
        TOGGLE_ON_TRUE,
        TOGGLE_ON_FALSE
    }

    public enum StreamDeckButtonType {
        DISPLAY,
        PRESS,
        TOGGLE,
        CUSTOM
    }

    public class ButtonConfiguration {
        private final Map<StreamDeckButton, Pair<Optional<BooleanSupplier>, StreamDeckButtonType>>
                buttonConfigurations = new HashMap<>();

        private ButtonConfiguration() {}

        public ButtonConfiguration addDefault(StreamDeckButton button) {
            buttonConfigurations.put(button, Pair.of(Optional.empty(), StreamDeckButtonType.PRESS));
            return this;
        }

        public ButtonConfiguration addToggle(StreamDeckButton button) {
            buttonConfigurations.put(button, Pair.of(Optional.empty(), StreamDeckButtonType.TOGGLE));
            return this;
        }

        public ButtonConfiguration add(StreamDeckButton button, BooleanSupplier selected) {
            buttonConfigurations.put(button, Pair.of(Optional.of(selected), StreamDeckButtonType.CUSTOM));
            return this;
        }
    }

    public class ButtonGroup {
        private final Map<StreamDeckButton, Trigger> triggers = new HashMap<>();
        private Optional<StreamDeckButton> selected = Optional.empty();

        private ButtonGroup() {}

        public ButtonGroup option(StreamDeckButton button) {
            initButton(button);
            return this;
        }

        public ButtonGroup option(StreamDeckButton button, Consumer<Trigger> trigger) {
            var buttonTrigger = initButton(button);
            trigger.accept(buttonTrigger);
            return this;
        }

        public ButtonGroup select(StreamDeckButton button) {
            selected = Optional.of(button);
            return this;
        }

        public ButtonGroup clear() {
            selected = Optional.empty();
            return this;
        }

        public Optional<StreamDeckButton> getSelected() {
            return selected;
        }

        public boolean isSelected(StreamDeckButton button) {
            return selected.isPresent() && selected.get() == button;
        }

        public Trigger trigger(StreamDeckButton button) {
            if (triggers.containsKey(button)) {
                return triggers.get(button);
            }

            return initButton(button);
        }

        private Trigger initButton(StreamDeckButton button) {
            button(button)
                    .onTrue(Commands.runOnce(() -> selected = Optional.of(button))
                            .ignoringDisable(true)
                            .withName("UpdateStreamDeckButtonGroupActiveIndex"));

            var trigger = new Trigger(() -> isSelected(button));
            triggers.put(button, trigger);
            return trigger;
        }
    }
}
