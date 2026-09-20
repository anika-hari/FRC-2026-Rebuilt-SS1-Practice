package frc.lib.util.Controls.StreamDeck;

import java.util.ArrayList;
import java.util.List;

public class StreamDeckButton {

    private final int index;
    private final String key;
    private String active_background = "#000000";
    private String inactive_background = "#000000";
    private String active_foreground = "#FFFFFF";
    private String inactive_foreground = "#FFFFFF";
    private String active_text = "";
    private String inactive_text = "";
    private boolean active_set = false;
    private boolean inactive_set = false;

    public StreamDeckButton(int row, int col, String key) {
        index = calculate_index(row, col);
        this.key = key;
    }

    public static int calculate_index(int row, int col) {
        return row * 8 + col % 8;
    }

    private void setInactiveIfUnconfigured() {
        if (!this.inactive_set) {
            this.inactive_background = this.active_background;
            this.inactive_foreground = this.active_foreground;
            this.inactive_text = this.active_text;
        }
    }

    private void setActiveIfUnconfigured() {
        if (!this.active_set) {
            this.active_background = this.inactive_background;
            this.active_foreground = this.inactive_foreground;
            this.active_text = this.inactive_text;
        }
    }

    public StreamDeckButton withActiveConfig(String active_background, String active_foreground, String active_text) {
        this.active_background = active_background;
        this.active_foreground = active_foreground;
        this.active_text = active_text;
        this.active_set = true;
        setInactiveIfUnconfigured();
        return this;
    }

    public StreamDeckButton withActiveConfig(StreamDeckButtonConfig streamDeckText) {
        this.active_background = streamDeckText.getBackground();
        this.active_foreground = streamDeckText.getForeground();
        this.active_text = streamDeckText.getText();
        this.active_set = true;
        setInactiveIfUnconfigured();
        return this;
    }

    public StreamDeckButton withInactiveConfig(
            String inactive_background, String inactive_foreground, String inactive_text) {
        this.inactive_background = inactive_background;
        this.inactive_foreground = inactive_foreground;
        this.inactive_text = inactive_text;
        this.inactive_set = true;
        setActiveIfUnconfigured();
        return this;
    }

    public StreamDeckButton withInactiveConfig(StreamDeckButtonConfig streamDeckText) {
        this.inactive_background = streamDeckText.getBackground();
        this.inactive_foreground = streamDeckText.getForeground();
        this.inactive_text = streamDeckText.getText();
        this.inactive_set = true;
        setActiveIfUnconfigured();
        return this;
    }

    public StreamDeckButton withActiveBackground(String background) {
        this.active_background = background;
        return this;
    }

    public StreamDeckButton withInactiveBackground(String background) {
        this.inactive_background = background;
        return this;
    }

    public StreamDeckButton withActiveForeground(String foreground) {
        this.active_foreground = foreground;
        return this;
    }

    public StreamDeckButton withInactiveForeground(String foreground) {
        this.inactive_foreground = foreground;
        return this;
    }

    public StreamDeckButton withActiveText(String text) {
        this.active_text = text;
        return this;
    }

    public StreamDeckButton withInactiveText(String text) {
        this.inactive_text = text;
        return this;
    }

    public StreamDeckButton withText(String text) {
        this.active_text = text;
        this.inactive_text = text;
        return this;
    }

    public int getIndex() {
        return index;
    }

    public String getKey() {
        return key;
    }

    public static List<String> getNetworkTableKeys() {
        List<String> ntKeys = new ArrayList<>();
        ntKeys.add("Appearance");
        // ntKeys.add("Key");
        // ntKeys.add("ActiveBackground");
        // ntKeys.add("InactiveBackground");
        // ntKeys.add("ActiveForeground");
        // ntKeys.add("InactiveForeground");
        // ntKeys.add("ActiveText");
        // ntKeys.add("InactiveText");
        return ntKeys;
    }

    public List<String> getDataToPublish() {
        List<String> dataToPublish = new ArrayList<>();
        StringBuilder sb = new StringBuilder();
        sb.append(key);
        sb.append("$&$");
        sb.append(active_background);
        sb.append("$&$");
        sb.append(inactive_background);
        sb.append("$&$");
        sb.append(active_foreground);
        sb.append("$&$");
        sb.append(inactive_foreground);
        sb.append("$&$");
        sb.append(active_text);
        sb.append("$&$");
        sb.append(inactive_text);
        dataToPublish.add(sb.toString());
        return dataToPublish;
    }
}
