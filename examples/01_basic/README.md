# Basic connections

Run from the repository root:

```powershell
python examples/01_basic/example_serial_exo.py --help
python examples/01_basic/example_serial_exo.py --port COM12

python examples/01_basic/example_bluetooth_exo.py --help
python examples/01_basic/example_bluetooth_exo.py --list
python examples/01_basic/example_bluetooth_exo.py --port COM8 --baud 115200

python examples/01_basic/example_tcp_exo.py --help
python examples/01_basic/example_tcp_exo.py --host 192.168.1.200
```

Bluetooth uses the same serial protocol as USB after the HC-05 has been paired
and exposed as a virtual COM port. TCP requires the optional Pico bridge or a
compatible server implementing the firmware's framed serial protocol.
