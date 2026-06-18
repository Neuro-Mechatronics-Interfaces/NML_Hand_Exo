# Application Examples

Real-world applications and task implementations using the Hand Exoskeleton.

## Examples

### 1. `example_pylsl_read.py`
**LSL stream reading**

Example of reading EMG data from Lab Streaming Layer.

```bash
python example_pylsl_read.py
```

---

### 2. `live_decoder_from_lsl_stream.py`
**Real-time gesture decoding**

Decode gestures from EMG signals in real-time.

```bash
python live_decoder_from_lsl_stream.py
```

**Requirements:**
- Trained gesture classifier model
- LSL EMG stream

---

### 3. `pca_viewer.py`
**PCA visualization**

Visualize EMG data in principal component space.

```bash
python pca_viewer.py
```

**Useful for:**
- Data quality assessment
- Feature visualization
- Dimensionality reduction analysis

---

### 4. `joystick_udp_direct_gui.py`
**Joystick-driven UDP direct control test GUI**

Sends `set_velocity` or `set_current` commands to the running HandExo GUI
through UDP command input.

```bash
pip install pygame
python joystick_udp_direct_gui.py
```

`pygame` is optional when using the on-screen virtual 2D joystick. Install it only
if you want physical gamepad input.

**Before use in HandExo GUI:**
- Connect to the device.
- Open Settings.
- Enable UDP Command Input.
- Enable Advanced mode (required for non-gesture commands).

**Notes:**
- Uses JSON UDP payloads in the form `{"command":"set_velocity:<id>:<value>"}`.
- Safety limits are clamped to +/-10 rpm (velocity) and +/-910 mA (current/torque).
- Use explicit integer motor IDs that are active in the current GUI mode.
- Includes a deadman profile: left stick Y controls command magnitude and the
    right trigger must be held to stream nonzero commands.
- Includes an interactive virtual 2D joystick: click and drag with the mouse,
  release to auto-center.
- Optional 2-axis dual-motor mode maps Y to the primary motor ID and X to a
    secondary motor ID.

---

## Task Applications

The `task/` subfolder contains complete task implementations:

### `training_task.py`
**Data collection task with GUI**

Collect labeled training data for gesture classification.

```bash
python task/training_task.py
```

**Features:**
- Visual prompts for gesture execution
- Synchronized data recording
- EMG + exo state capture
- Trial management

---

### `task_gui_minimal.py`
**Minimal task interface**

Simplified task GUI for custom experiments.

```bash
python task/task_gui_minimal.py
```

---

### `task_config.json`
**Task configuration**

JSON configuration file for task parameters:
- Trial timing
- Gesture sequences
- Recording settings
- Display options

---

## Building Applications

### Typical Application Structure

```python
from nml_hand_exo.interface import HandExo, SerialComm, LSLClient

# 1. Connect to exo
comm = SerialComm(port="COM6", baudrate=57600)
exo = HandExo(comm)
exo.connect()

# 2. Set up LSL stream (if using EMG)
lsl_client = LSLClient(stream_type="EMG", stream_name="OpenEphysEMG")

# 3. Application main loop
try:
    while running:
        # Read EMG
        emg_data = lsl_client.pull()
        
        # Process data / classify gesture
        gesture = classifier.predict(emg_data)
        
        # Control exo
        exo.set_gesture(gesture, "open")
        
except KeyboardInterrupt:
    pass
finally:
    exo.close()
```

---

## Common Application Patterns

**Data Collection:**
- Synchronize EMG + exo state
- Record timestamps
- Label trials
- Save to file

**Real-Time Control:**
- Low-latency streaming
- Predictive decoding
- Smooth gesture transitions
- Error handling

**Feedback Applications:**
- Visual feedback (GUI)
- Haptic feedback (motor resistance)
- Audio cues
- Performance metrics

---

## Dependencies

Applications may require additional packages:
```bash
pip install pylsl numpy pandas scikit-learn pyqt5 pyqtgraph matplotlib
```

See individual script imports for specific requirements.
