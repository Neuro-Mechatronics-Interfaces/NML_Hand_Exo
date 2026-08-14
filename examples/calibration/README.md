# Calibration and ROM assessment

```powershell
python examples/calibration/calibrate_exo.py --help
python examples/calibration/rom_assessment.py --help
```

Profiles are safety-critical and side-specific. Always inspect the connected
firmware's integer motor IDs and the profile's `side` metadata before applying
values. Do not use another participant's limits on a worn device.
