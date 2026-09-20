Read and analyze the FRC robot/simulation console output captured in `.riolog/console.log`.

Use the Read tool to read the file `.riolog/console.log`. If the file doesn't exist, tell the user to start capture first with `/riolog-start`.

$ARGUMENTS

If arguments are provided, use them as a filter:
- "errors" or "error" → focus only on lines containing [ERROR]
- "recent" → show only the last 50 lines
- Any other text → search for lines containing that text (e.g., "CAN", "shooter", "NullPointer")

After reading the log, provide a structured analysis:

1. **Status**: Is the robot/simulation running normally or are there issues?
2. **Errors**: List any Java exceptions, stack traces, WPILib warnings, CAN bus errors, or loop overrun messages. For each error:
   - What the error is
   - The likely root cause
   - Which file/class is involved
   - Suggested fix
3. **Warnings**: Any non-fatal warnings worth noting
4. **Recent Activity**: Brief summary of what the robot code is doing based on the latest output

Cross-reference errors with the actual source code in this project when possible - read the relevant source files to provide specific fix suggestions.
