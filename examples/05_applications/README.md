# Application integrations

Read the first EMG stream visible through LSL:

```powershell
python examples/05_applications/example_pylsl_read.py
```

Run the optional joystick/virtual-joystick UDP controller:

```powershell
python -m pip install -e ".[integrations]"
python examples/05_applications/joystick_udp_direct_gui.py
```

The joystick GUI sends bounded JSON commands to the main HandExo GUI's optional
UDP input. Configure the target motor IDs and safety gates in the main GUI first.

Use the maintained `nml-task-cue` command for event-marked participant tasks.
Incomplete PCA and task prototypes were removed from this directory.
