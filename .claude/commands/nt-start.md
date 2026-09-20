Start the NetworkTables capture script to record NT4 data from a robot or simulation for debugging.

$ARGUMENTS

The argument should be one of:
- `sim` - Connect to simulation NetworkTables on localhost:5810
- `robot` - Connect to real robot NT (auto-discovers roborio-3476-frc.local or 10.34.76.2)
- `robot <address>` - Connect to robot at a specific address

Additional options can be appended:
- `--stream` - Continuous capture with change detection (default is one-time snapshot)
- `--duration <seconds>` - Stop streaming after N seconds
- `--filter <prefix>` - Only capture topics matching prefix (e.g., `--filter Drive`, `--filter Shooter`)
- `--exclude <prefix>` - Exclude topics matching prefix

Examples:
  sim                              # Snapshot of all sim NT values
  sim --stream                     # Continuous capture from sim
  sim --stream --filter Drive      # Stream only Drive subsystem topics
  sim --filter Shooter             # Snapshot of Shooter subsystem only
  robot --stream --duration 30     # Capture robot data for 30 seconds

Run the capture script using Bash:
```
python scripts/networktables.py <mode> [options]
```

The script will:
- Connect to NetworkTables via the NT4 WebSocket protocol on port 5810
- Write topic list to `.ntcapture/topics.log`
- Write current values snapshot to `.ntcapture/snapshot.log`
- In stream mode, write value changes to `.ntcapture/changes.log`

After starting, inform the user:
- The capture is running and writing to `.ntcapture/`
- They can use `/nt` to have Claude analyze the captured data
- For snapshot mode (default), it captures and exits automatically
- For stream mode, they can stop with Ctrl+C

If no argument is provided, ask the user whether they want `sim` or `robot` mode.
If the script reports missing dependencies, run `pip install websockets msgpack` first.
