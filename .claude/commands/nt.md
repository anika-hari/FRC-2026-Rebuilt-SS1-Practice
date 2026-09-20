Read and analyze NetworkTables data captured in `.ntcapture/` from an FRC robot or simulation.

Use the Read tool to read files in the `.ntcapture/` directory. If the directory doesn't exist or files are empty, tell the user to start capture first with `/nt-start sim` or `/nt-start robot`.

$ARGUMENTS

If arguments are provided, use them as a filter or command:
- "snapshot" → read `.ntcapture/snapshot.log` for current state overview (this is also the default)
- "changes" → read `.ntcapture/changes.log` for recent value changes (stream mode only)
- "topics" → read `.ntcapture/topics.log` for all announced topic names and types
- A subsystem name (e.g., "Drive", "Shooter", "Vision") → show only that subsystem's data from snapshot
- "tuning" → show all Tuning/ values (PID gains, motion magic params)
- "timing" or "loop" → show LoopTiming data for performance analysis
- Any other text → search across all log files for matching topic names or values

Default behavior (no arguments): read snapshot.log for a full state overview.

After reading the data, provide structured analysis:

1. **Robot State Summary**: Is the robot enabled/disabled? What mode (auto/teleop/test)? Simulation or real?
2. **Subsystem Health**: For each subsystem visible in the data:
   - Current command running
   - Key state values (positions, velocities, setpoints)
   - Any anomalies (zero values that shouldn't be, extreme values, stale data)
3. **Performance**: If LoopTiming data is present:
   - Current loop utilization percentage
   - Any loop overruns
   - Which subsystems take the most time
4. **Tuning Values**: If Tuning/ data is present, list current PID/FF gains
5. **Issues Detected**: Flag any problems:
   - Subsystems not responding (zero velocity with non-zero voltage)
   - CAN bus issues (all zeros in motor inputs)
   - Vision disconnected
   - Loop timing problems
   - Unexpected state combinations

If analyzing changes.log (streaming data):
6. **Change Analysis**:
   - Which topics are changing most frequently?
   - Are there topics that stopped changing (potential stall/disconnect)?
   - What was the sequence of events? (useful for debugging auto routines)

Cross-reference with source code when relevant - read subsystem files to understand expected behavior and identify root causes of issues.
