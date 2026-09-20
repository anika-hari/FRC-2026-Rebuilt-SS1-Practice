package frc.lib.util.Controls.StreamDeck;

public class StreamDeckButtonConfig {
    private String foreground;
    private String background;
    private String text;

    public StreamDeckButtonConfig(String background, String foreground, String text) {
        this.background = background;
        this.foreground = foreground;
        this.text = text;
    }

    public String getForeground() {
        return foreground;
    }

    public String getBackground() {
        return background;
    }

    public String getText() {
        return text;
    }
}
