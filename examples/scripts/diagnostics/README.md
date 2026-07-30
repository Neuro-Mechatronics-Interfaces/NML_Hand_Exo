# Hardware Diagnostics

Manual serial and HC-05 troubleshooting utilities:

- `scan_ports.py` lists serial ports and probes common baud rates.
- `test_bluetooth_ports.py` identifies a responsive Bluetooth COM port.
- `test_hc05_data_mode.py` checks whether commands reach the OpenRB through HC-05.
- `test_hc05_wiring.py` walks through USB, HC-05, and wiring checks.

Run these from the repository root after activating the project environment:

```powershell
python examples/scripts/diagnostics/scan_ports.py
```

These scripts interact with real hardware and are not automated unit tests.
