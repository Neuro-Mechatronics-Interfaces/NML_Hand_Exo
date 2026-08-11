# Pre-session validation

Run the diagnostic before an experiment to verify the host/firmware connection,
motor IDs, repeated angle feedback, joint limits, and enabled state.

The default command is read-only and does not enable or command any motor:

```powershell
.\.venv\Scripts\python.exe tools\pre_session_check.py --port COM5
```

For dual CDC, provide both device ports:

```powershell
.\.venv\Scripts\python.exe tools\pre_session_check.py `
  --command-port COM5 --telemetry-port COM6
```

## Optional hardware exercises

Motion and auxiliary-hold exercises are disabled unless the exact confirmation
phrase is provided. They also refuse to run if any other motor is enabled. The
motion test is capped at 1 rpm and 0.5 seconds by the script.

Keep the mechanism clear and use an unloaded or safely fitted device:

```powershell
.\.venv\Scripts\python.exe tools\pre_session_check.py --port COM5 `
  --exercise-hold 14 `
  --confirm-motion I_UNDERSTAND_THIS_MOVES_HARDWARE
```

```powershell
.\.venv\Scripts\python.exe tools\pre_session_check.py --port COM5 `
  --exercise-motion 15 --motion-rpm 0.5 --motion-duration 0.2 `
  --confirm-motion I_UNDERSTAND_THIS_MOVES_HARDWARE
```

The diagnostic always attempts to stop and disable the exercised motor and
restore position mode, including when an exception occurs. It does not replace
the operator's physical inspection, calibrated joint-limit verification, or
access to the GUI's **STOP ALL MOTION** control.
