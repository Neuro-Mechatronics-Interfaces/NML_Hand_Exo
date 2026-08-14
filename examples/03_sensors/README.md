# IMU examples

```powershell
python examples/03_sensors/imu/imu_serial.py
python examples/03_sensors/example_imu_control.py
```

Both examples currently configure their serial port in the script. The first
prints IMU orientation until Ctrl+C. The second demonstrates firmware-assisted
yaw control and can move a wrist motor; validate its target ID and limits before
running it.
