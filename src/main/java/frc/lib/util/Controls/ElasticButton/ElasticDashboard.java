package frc.lib.util.Controls.ElasticButton;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.subsystems.real.ServoMotorSubsystem;
import frc.lib.util.LoggedTunableNumber;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Helper to generate an Elastic dashboard layout JSON from the registered tabs and subsystems.
 *
 * The generated layout will:
 * - Preserve button insertion order from ElasticTab (LinkedHashMap)
 * - Place buttons in vertical columns (up to a fixed number of rows)
 * - Place the standard tunables (kP..Jerk) for all subsystems after the button columns
 * - Place subsystem-specific tunables (from getTunables()) in a contiguous block of columns
 *   after the standard tunables (one column per subsystem)
 */
public class ElasticDashboard {
    private Map<String, ElasticTab> tabs = new LinkedHashMap<>();
    private Map<ElasticTab, ServoMotorSubsystem<?>[]> subsystems = new LinkedHashMap<>();
    private NetworkTable table;

    public ElasticDashboard() {
        this("Elastic");
    }

    public ElasticDashboard(String tableName) {
        table = NetworkTableInstance.getDefault().getTable(tableName);
    }

    public ElasticTab addTab(String key, ServoMotorSubsystem<?>... subsystems) {
        ElasticTab tab = new ElasticTab(this, key);
        this.subsystems.put(tab, subsystems);
        tabs.put(key, tab);
        return tab;
    }

    public NetworkTable getTable() {
        return table;
    }

    public ElasticTab getTab(String key) {
        return tabs.get(key);
    }

    /**
     * Generate the dashboard JSON and write it to dashboard/elastic-layout-generated.json.
     * This method is defensive and will skip copying Teleoperated/Cameras if the source
     * layout isn't available.
     */
    public void generateDashboardJson() {
        final double gridSize = 96.0;

        // Try to read the existing dashboard layout so we can extract Teleoperated and Cameras
        String teleopJson = null;
        String camerasJson = null;
        try {
            Path layoutPath = Paths.get("dashboard", "elastic-layout.json");
            if (Files.exists(layoutPath)) {
                String layout = Files.readString(layoutPath, StandardCharsets.UTF_8);
                teleopJson = extractTabObject(layout, "Teleoperated");
                camerasJson = extractTabObject(layout, "Cameras");
            }
        } catch (IOException ignored) {
            // If we can't read the file, we'll just skip copying those tabs.
        }

        final int BUTTON_WIDTH_CELLS = 2;
        final int BUTTON_HEIGHT_CELLS = 1;
        final int MAX_BUTTON_ROWS = 6 / BUTTON_HEIGHT_CELLS; // visual rows available per button-column

        List<String> generated = new ArrayList<>();

        for (Map.Entry<String, ElasticTab> tabEntry : tabs.entrySet()) {
            String tabName = tabEntry.getKey();
            ElasticTab tab = tabEntry.getValue();

            StringBuilder t = new StringBuilder();
            t.append("    {\n");
            t.append("      \"name\": \"").append(escapeJson(tabName)).append("\",\n");
            t.append("      \"grid_layout\": {\n");
            t.append("        \"layouts\": [],\n");
            t.append("        \"containers\": [\n");

            boolean firstContainer = true;

            // Buttons: vertical columns, preservation of insertion order
            int bIndex = 0;
            int nButtons = tab.getButtons().size();

            for (Map.Entry<String, ElasticButton> buttonEntry : tab.getButtons().entrySet()) {
                String buttonName = buttonEntry.getKey();
                if (!firstContainer) t.append(",\n");

                int col = bIndex / MAX_BUTTON_ROWS;
                int row = bIndex % MAX_BUTTON_ROWS;
                double x = col * BUTTON_WIDTH_CELLS * gridSize;
                double y = row * BUTTON_HEIGHT_CELLS * gridSize;

                t.append("          {\n");
                t.append("            \"title\": \" \",\n");
                t.append("            \"x\": ").append(x).append(",\n");
                t.append("            \"y\": ").append(y).append(",\n");
                t.append("            \"width\": ")
                        .append((gridSize * BUTTON_WIDTH_CELLS))
                        .append(",\n");
                t.append("            \"height\": ")
                        .append((gridSize * BUTTON_HEIGHT_CELLS))
                        .append(",\n");
                t.append("            \"type\": \"Toggle Button\",\n");
                t.append("            \"properties\": {\n");
                t.append("              \"topic\": \"/Elastic/")
                        .append(escapeJson(tabName))
                        .append("/")
                        .append(escapeJson(buttonName))
                        .append("\",\n");
                t.append("              \"period\": 0.15,\n");
                t.append("              \"data_type\": \"boolean\"\n");
                t.append("            }\n");
                t.append("          }");

                firstContainer = false;
                bIndex++;
            }

            // Standard tunables: collect subsystem names and short names
            ServoMotorSubsystem<?>[] servos = this.subsystems.get(tab);
            String[] tunableKeys = new String[] {"kP", "kI", "kD", "kG", "kS", "kA", "kV", "Velo", "Accel", "Jerk"};

            String[] subsystemNames = new String[0];
            String[] shortNames = new String[0];
            if (servos != null) {
                subsystemNames = new String[servos.length];
                for (int sidx = 0; sidx < servos.length; sidx++) {
                    ServoMotorSubsystem<?> servo = servos[sidx];
                    subsystemNames[sidx] = (servo == null) ? "" : servo.getName();
                }

                // compute short unique names
                shortNames = new String[servos.length];
                String[][] tokens = new String[servos.length][];
                int maxTokens = 0;
                for (int sidx = 0; sidx < servos.length; sidx++) {
                    String name = subsystemNames[sidx];
                    if (name == null) name = "";
                    tokens[sidx] = name.split(" ");
                    maxTokens = Math.max(maxTokens, tokens[sidx].length);
                }

                for (int sidx = 0; sidx < servos.length; sidx++) {
                    String chosen = null;
                    int bestFreq = Integer.MAX_VALUE;
                    for (int pos = 0; pos < maxTokens; pos++) {
                        String tok = pos < tokens[sidx].length ? tokens[sidx][pos] : "";
                        if (tok.isEmpty()) continue;
                        int freq = 0;
                        for (int k = 0; k < servos.length; k++) {
                            String other = pos < tokens[k].length ? tokens[k][pos] : "";
                            if (tok.equals(other)) freq++;
                        }
                        if (freq < bestFreq) {
                            bestFreq = freq;
                            chosen = tok;
                            if (freq == 1) break;
                        }
                    }
                    if (chosen == null || chosen.isEmpty()) {
                        String[] parts = tokens[sidx];
                        chosen = (parts.length > 0) ? parts[parts.length - 1] : subsystemNames[sidx];
                    }
                    shortNames[sidx] = chosen;
                }
            }

            // Emit standard kP..Jerk tunables for all subsystems (two columns of 5 rows each per-subsystem)

            // compute numeric base column where standard tunables start
            int numButtonColsUsed = (nButtons == 0) ? 0 : ((nButtons + MAX_BUTTON_ROWS - 1) / MAX_BUTTON_ROWS);
            int standardTunablesStartCol = numButtonColsUsed * BUTTON_WIDTH_CELLS;

            if (servos != null) {
                for (int sidx = 0; sidx < servos.length; sidx++) {
                    ServoMotorSubsystem<?> servo = servos[sidx];
                    if (servo == null) continue;
                    int baseCol = standardTunablesStartCol + sidx * 2; // two columns per subsystem

                    for (int tIdx = 0; tIdx < tunableKeys.length; tIdx++) {
                        String tunable = tunableKeys[tIdx];
                        int colOffset = (tIdx >= 5) ? 1 : 0; // first 5 in first column, next 5 in second
                        int row = tIdx % 5; // 5 rows
                        int col = baseCol + colOffset;

                        if (!firstContainer) t.append(",\n");

                        double x = col * gridSize;
                        double y = row * gridSize;

                        t.append("          {\n");
                        t.append("            \"title\": \"")
                                .append(escapeJson(tunable + shortNames[sidx]))
                                .append("\",\n");
                        t.append("            \"x\": ").append(x).append(",\n");
                        t.append("            \"y\": ").append(y).append(",\n");
                        t.append("            \"width\": ")
                                .append((int) gridSize)
                                .append(",\n");
                        t.append("            \"height\": ")
                                .append((int) gridSize)
                                .append(",\n");
                        t.append("            \"type\": \"Text Display\",\n");
                        t.append("            \"properties\": {\n");
                        t.append("              \"topic\": \"/Tuning/")
                                .append(escapeJson(subsystemNames[sidx]))
                                .append("/")
                                .append(escapeJson(tunable))
                                .append("\",\n");
                        t.append("              \"period\": 0.3,\n");
                        t.append("              \"data_type\": \"double\",\n");
                        t.append("              \"show_submit_button\": true\n");
                        t.append("            }\n");
                        t.append("          }");

                        firstContainer = false;
                    }
                }
            }

            // Collect subsystem-specific tunables first so we can place them in a contiguous block
            List<LoggedTunableNumber[]> perSubsystemSpecific = new ArrayList<>();
            if (servos != null) {
                for (int sidx = 0; sidx < servos.length; sidx++) {
                    ServoMotorSubsystem<?> servo = servos[sidx];
                    LoggedTunableNumber[] specific = servo.getTunables();
                    perSubsystemSpecific.add(specific);
                }
            }

            // Now emit subsystem-specific tunables in a block after all standard tunables.

            final int TUNABLE_WIDTH_CELLS = 2;
            final int TUNABLE_HEIGHT_CELLS = 1;
            int specBase = standardTunablesStartCol + ((servos == null) ? 0 : servos.length * 2);
            if (servos != null) {
                for (int sidx = 0; sidx < servos.length; sidx++) {
                    LoggedTunableNumber[] specific = perSubsystemSpecific.get(sidx);
                    if (specific == null || specific.length == 0) continue;

                    int specCol = specBase + sidx * TUNABLE_WIDTH_CELLS; // one column per subsystem

                    for (int si = 0; si < specific.length; si++) {
                        LoggedTunableNumber ltn = specific[si];
                        if (ltn == null) continue;
                        if (!firstContainer) t.append(",\n");

                        double xSpec = specCol * gridSize;
                        double ySpec = si * gridSize * TUNABLE_HEIGHT_CELLS;

                        String key = ltn.getKey();
                        String titlePart = (key != null && key.contains("/"))
                                ? key.substring(key.lastIndexOf('/') + 1)
                                : "Tunable";

                        t.append("          {\n");
                        // subsystem-specific tunables are two cells wide and one cell tall
                        t.append("            \"title\": \"")
                                .append(escapeJson(shortNames[sidx] + " " + titlePart))
                                .append("\",\n");
                        t.append("            \"x\": ").append(xSpec).append(",\n");
                        t.append("            \"y\": ").append(ySpec).append(",\n");
                        t.append("            \"width\": ")
                                .append((gridSize * TUNABLE_WIDTH_CELLS))
                                .append(",\n");
                        t.append("            \"height\": ")
                                .append(gridSize * TUNABLE_HEIGHT_CELLS)
                                .append(",\n");
                        t.append("            \"type\": \"Text Display\",\n");
                        t.append("            \"properties\": {\n");
                        t.append("              \"topic\": \"")
                                .append(escapeJson(key))
                                .append("\",\n");
                        t.append("              \"period\": 0.3,\n");
                        t.append("              \"data_type\": \"double\",\n");
                        t.append("              \"show_submit_button\": true\n");
                        t.append("            }\n");
                        t.append("          }");

                        firstContainer = false;
                    }
                }
            }

            t.append("\n        ]\n");
            t.append("      }\n");
            t.append("    }");

            generated.add(t.toString());
        }

        // Assemble final tabs order: Teleoperated (if present), generated tabs, Cameras (if present)
        StringBuilder sb = new StringBuilder();
        sb.append("{\n");
        sb.append("  \"version\": 1.0,\n");
        sb.append("  \"grid_size\": ").append((int) gridSize).append(",\n");
        sb.append("  \"tabs\": [\n");

        boolean firstTab = true;
        if (teleopJson != null) {
            sb.append(teleopJson);
            firstTab = false;
        }

        for (String g : generated) {
            if (!firstTab) sb.append(",\n");
            sb.append(g);
            firstTab = false;
        }

        if (camerasJson != null) {
            if (!firstTab) sb.append(",\n");
            sb.append(camerasJson);
        }

        sb.append("\n  ]\n");
        sb.append("}\n");

        // Write to dashboard/elastic-layout-generated.json
        Path out = Paths.get("dashboard", "elastic-layout-generated.json");
        try {
            if (out.getParent() != null) Files.createDirectories(out.getParent());
            Files.write(out, sb.toString().getBytes(StandardCharsets.UTF_8));
        } catch (IOException e) {
            System.err.println("Failed to write elastic-layout-generated.json: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private String extractTabObject(String json, String tabName) {
        if (json == null || tabName == null) return null;
        String needle = "\"name\": \"" + tabName + "\"";
        int idx = json.indexOf(needle);
        if (idx < 0) return null;
        // find the opening brace of the object containing this name
        int objStart = json.lastIndexOf('{', idx);
        if (objStart < 0) return null;
        int depth = 0;
        for (int i = objStart; i < json.length(); i++) {
            char c = json.charAt(i);
            if (c == '{') depth++;
            else if (c == '}') depth--;
            if (depth == 0) {
                return json.substring(objStart, i + 1);
            }
        }
        return null;
    }

    private String escapeJson(String s) {
        if (s == null) return "";
        StringBuilder sb = new StringBuilder();
        for (char c : s.toCharArray()) {
            switch (c) {
                case '\\':
                    sb.append("\\\\");
                    break;
                case '"':
                    sb.append("\\\"");
                    break;
                case '\b':
                    sb.append("\\b");
                    break;
                case '\f':
                    sb.append("\\f");
                    break;
                case '\n':
                    sb.append("\\n");
                    break;
                case '\r':
                    sb.append("\\r");
                    break;
                case '\t':
                    sb.append("\\t");
                    break;
                default:
                    if (c < 32) {
                        sb.append(String.format("\\u%04x", (int) c));
                    } else {
                        sb.append(c);
                    }
            }
        }
        return sb.toString();
    }
}
