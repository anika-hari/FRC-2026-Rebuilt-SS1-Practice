package frc.lib.util.Controls.ElasticButton;

import edu.wpi.first.networktables.NetworkTable;
import java.util.LinkedHashMap;
import java.util.Map;

public class ElasticTab {
    private Map<String, ElasticButton> buttons = new LinkedHashMap<>();
    // private List<String> tunables = new ArrayList<>();
    private NetworkTable table;

    public ElasticTab(ElasticDashboard parent, String key) {
        table = parent.getTable().getSubTable(key);
    }

    public ElasticButton addButton(String key) {
        ElasticButton button = new ElasticButton(this, key, false);
        buttons.put(key, button);
        return button;
    }

    public ElasticButton addButton(String key, boolean defaultValue) {
        ElasticButton button = new ElasticButton(this, key, defaultValue);
        buttons.put(key, button);
        return button;
    }

    public NetworkTable getTable() {
        return table;
    }

    public ElasticButton getButton(String key) {
        return buttons.get(key);
    }

    public Map<String, ElasticButton> getButtons() {
        return buttons;
    }
}
