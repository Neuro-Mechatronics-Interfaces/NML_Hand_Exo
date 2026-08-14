# Motor control

These scripts query the firmware motor map before selecting IDs, but they can
enable and move hardware. Review their constants and use only a bench-tested
device with validated limits.

```powershell
python examples/02_motor_control/example_motor_config.py
python examples/02_motor_control/example_batch_operations.py
```

Both scripts currently expose their port near the top of the file and are
intended as readable API demonstrations, not general command-line tools. In
dual firmware, replace any broad `all` operation with explicit active-side IDs
for participant use.
