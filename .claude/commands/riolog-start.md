Start the RIOLog console capture script to record robot/simulation output for debugging.

$ARGUMENTS

The argument should be one of:
- `sim` - Launch the simulation (`./gradlew simulateJava`) and capture its console output
- `robot` - Connect to the real robot via the NetConsole TCP protocol (default: roborio-3476-frc.local)
- `robot <address>` - Connect to robot at a specific address

Run the capture script using Bash:
```
python scripts/riolog.py <mode> [address]
```

The script will:
- Write all output to `.riolog/console.log` with timestamps
- Pass through output to the terminal so you can see it live
- For simulation: launch `./gradlew simulateJava` as a subprocess
- For robot: connect via NetConsole TCP binary protocol on port 1741

After starting, inform the user:
- The capture is running and writing to `.riolog/console.log`
- They can use `/riolog` to have Claude analyze the captured output
- They can stop capture with Ctrl+C in the terminal

If no argument is provided, ask the user whether they want `sim` or `robot` mode.
