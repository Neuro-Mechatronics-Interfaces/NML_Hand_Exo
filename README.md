# NML_Hand_Exo

<p align="center">
  <img src="docs/source/_static/hand-exo.jpg" width="80%"/>
</p>

[![Python](https://img.shields.io/badge/python-3.10-blue)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo/actions/workflows/gh-pages.yml/badge.svg)](https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo/actions/workflows/gh-pages.yml)
[![Docs](https://img.shields.io/badge/docs-online-blue.svg)](https://neuro-mechatronics-interfaces.github.io/NML_Hand_Exo/)

This repository contains the firmware and Python tools for controlling the **NML Hand Exoskeleton**—a modular, open-source robotic hand exoskeleton platform for research and prototyping.

## 🚀 Overview

The **NML Hand Exoskeleton** includes:
- 🦾 Microcontroller firmware (Arduino/C++): real-time motor control and communication.
- 🐍 Python API: high-level interface for controlling the device.
- 🛠️ Demo scripts: examples of using the device with real-time EMG streaming and GUI control.

Release **0.2.17** is tested on **Windows 11** with **Python 3.11**.

## Primary applications

The supported desktop workflow consists of four applications:

- `handexo emg-intent` - discovers participant-specific control intents, ranks
  candidate pairs, fits an orientation-aware decoder, and publishes guarded LSL intent.
- `nml-task-cue` - presents participant task prompts and publishes timestamped
  LSL string markers without connecting to or controlling the exoskeleton.

- `handexo emg-centroid` — trains, visualizes, and publishes EMG intent over LSL.
- `handexo gui` — connects to the exoskeleton and provides guarded control,
  telemetry, calibration, direct control, and optional EMG Teleop.

EMG Teleop is not a third application and does not open a direct serial link from
the decoder. It is opt-in inside `handexo gui`; it requires a versioned LSL intent
stream, an active calibrated DXL ID armed for direct velocity control, fresh valid
intent, and a held deadman. Any stale, rest, invalid, or low-confidence intent stops
the previously commanded motor.

---

## Installation 

### 1. Clone the Repository

```bash
git clone https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo.git
cd NML_Hand_Exo
```

### 2. Create a virtual environment (recommended)

Choose one of the following methods to create a virtual environment for this project:
   - Using [Anaconda](https://www.anaconda.com/products/distribution) :
      ```bash
      conda create -n handexo
      conda activate handexo
      ```
   - or using Python's virtualenv:
     ```bash
     python3 -m venv .handexo # Use python -m venv .handexo on Windows
     source .handexo/bin/activate # On Linux/Mac
     # call .handexo/Scripts/activate  # On Windows
     ```
  
### 3. Install Python dependencies

```bash
pip install -r requirements.txt
```

For local development, you can also install the python API as a package. 
```bash
pip install -e .
```

Launch the supported applications after installation:

```bash
handexo gui
handexo emg-intent
handexo emg-centroid
nml-task-cue
```

The firmware defaults to **1 Mbps** for USB and the Dynamixel bus, and
**115200** for the HC-05 command link. Reflash the OpenRB firmware after
changing firmware constants.

## Participant task cue and LSL markers

Launch the standalone task-cue application after installing the project:

```powershell
nml-task-cue
```

This application publishes visual prompts and event markers only. It does not
connect to, control, enable, or arm the exoskeleton, and it does not require a
hand-angle stream. It can be used in EMG-only experiments.

The operator window's **Publish LSL markers** toggle is enabled by default.
When enabled, the GUI creates the marker outlet immediately at startup so it is
visible to LabRecorder before **Start Task** is pressed. The GUI displays a
green `LIVE` indicator and the terminal reports the stream name and source ID.
Turn the toggle off to run the participant display as visual cues only, without
creating an LSL outlet or requiring a working `pylsl` installation. Stream name
and source ID settings are disabled while marker publishing is off.

### Build or load a prompt plan

A saved file is optional. In the operator window, use one of the three quick-add
buttons or enter any custom marker label and duration, then reorder or remove
steps in the preview. Each quick-add button can be edited to store a preferred
label and duration for the current session. The resulting plan can start
immediately or be saved with **Save Plan As...** for reuse. An unsaved plan is
identified in the marker stream as `plan=gui_prompt_plan.json`.

To reuse a plan, load a JSON file containing a non-empty array of prompt
objects. Each object requires a non-empty `label` and a positive duration in
seconds:

```json
[
  {"label": "rest", "duration": 2},
  {"label": "isolated_digits:thumb_flex", "duration": 5},
  {"label": "coordinated_grasp:pinch", "duration": 5}
]
```

Labels are preserved exactly in the marker stream. A label equal to `rest`
uses `trial=000`. Trial IDs increment only for non-rest prompts.

### LabRecorder setup

The default marker stream is named `NML_TaskMarkers`, has LSL type `Markers`,
and uses source ID `nml_hand_exo_task_cue`. In LabRecorder, select both the EMG
stream and `NML_TaskMarkers` before recording.

Recommended order:

1. Start LabRecorder and select EMG plus `NML_TaskMarkers`.
2. Start recording in LabRecorder.
3. Start the task cue GUI with `nml-task-cue`.
4. Build a plan in the GUI or load and preview a JSON plan, open the participant
   cue window, and click **Start Task**.

### Marker protocol

Markers are single-channel irregular LSL string samples timestamped with the
LSL local clock. A typical sequence is:

```text
session_start
prompt_sequence_start|plan=<filename>
rest_onset|duration_s=2.000
prompt_onset|phase=rest|trial=000|gesture=rest|duration_s=2.000
prompt_offset|phase=rest|trial=000|gesture=rest
trial_start|trial=001|gesture=isolated_digits:thumb_flex|duration_s=5.000
prompt_onset|phase=gesture|trial=001|gesture=isolated_digits:thumb_flex|duration_s=5.000
prompt_offset|phase=gesture|trial=001|gesture=isolated_digits:thumb_flex
trial_end|trial=001|gesture=isolated_digits:thumb_flex
session_complete
```

Task-level state changes use `session_pause`, `session_resume`, and
`session_abort`. Pausing freezes the displayed cue and remaining duration;
resuming extends that prompt's deadline by the time spent paused. A normal run
emits `session_complete` once, while an operator stop emits `session_abort`
once.

### 4. (Optional) Install Max WTF dependencies
If you are on Max's WTF `dev/Max` side-branch, you can also add his WTF code at your own risk.
```bash
pip install -e src/nml_wtf_exo      
```

(this extra/dev installation lets you use entry-level scripts for Max WTF applications, such as):
```bash
nml-wtf-exo          # launches main exo app
nml-wtf-exo-viewer   # opens your viewer GUI
nml-wtf-exo-logger   # starts logger
nml-wtf-exo-keyboard # opens keyboard overlay app
```
   
## Exo Firmware

The exo device uses an [openRB-150](https://emanual.robotis.com/docs/en/parts/controller/openrb-150/) microcontroller from ROBOTIS. The firmware is located in `src/cpp/nml_hand_exo` and can be uploaded via the [Arduino IDE](https://www.arduino.cc/en/software/). 

The firmware includes a class NMLHandExo, which handles:

- Dynamixel initialization and setup
- Motor control by ID, name, or alias
- Joint limits and angle-to-position conversion
- Calibration and LED feedback
- Serial command parsing

To upload the firmware:

1. Open `nml_hand_exo.ino` in the Arduino IDE.
2. Select the correct board and port under Tools.
3. Upload the sketch.

## Usage

An example of using the Python API for scripting and control:
```python
from nml_hand_exo.interface import HandExo, SerialComm

comm = SerialComm(port="COM3", baudrate=1000000)
exo = HandExo(comm)
exo.connect()
exo.enable_motor(1)
exo.set_motor_angle(1, 45)
angle = exo.get_motor_angle(1)
print(f"Motor angle: {angle} degrees")
exo.disable_motor(1)
exo.close()
```

You can control the hand exoskeleton over USB or Bluetooth using simple, structured serial commands. For example:

- `set_angle:WRIST:30` — set wrist motor to 30 degrees.

- `enable:1` — enable motor 1 torque.

- `get_angle:1` — query relative angle.

Supported aliases are `THUMB`, `INDEX`, `MIDDLE`, `RING`, `PINKY`, `WRIST`

For a complete list of commands, see the [Usage Guide](https://neuro-mechatronics-interfaces.github.io/NML_Hand_Exo/usage.html).

## Validation before an experiment

Run the automated host suite and host/firmware command-contract check after
changes:

```powershell
.\.venv\Scripts\python.exe -m pytest -q
.\.venv\Scripts\python.exe tools\check_protocol_contract.py
```

With hardware connected, the pre-session diagnostic is read-only by default:

```powershell
.\.venv\Scripts\python.exe tools\pre_session_check.py --port COM5
```

It checks device information, repeated motor-ID/angle feedback, joint limits,
and enabled states. Optional hold and low-speed motion exercises require an
explicit confirmation phrase and refuse to run while another motor is enabled.
See [docs/pre_session_validation.md](docs/pre_session_validation.md).

## Demo

#### MindRove EMG Streaming

![](/docs/source/_static/pyqtemg.gif)

A demo script is included to showcase real-time plotting of EMG signals from a connected MindRove EMG band. 
1) Connect your MindRove EMG Band to the PC (using a Wifi dongle if you want to maintain internet connection on a separate wifi network)
2) Run the demo script
   ~~~
   python demo_mindrove_realtime.py
   ~~~

## 📖 How to Cite

If you use this project in your research, please cite it as:

Jonathan Shulgach & Kriti Kacker. (2025). NML Hand Exoskeleton [Computer software]. https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo

BibTeX:
```bibtex
@misc{shulgach_kacker_2025_nmlhandexo,
  author       = {Jonathan Shulgach and Kriti Kacker},
  title        = {NML Hand Exoskeleton},
  year         = {2025},
  publisher    = {GitHub},
  journal      = {GitHub repository},
  howpublished = {\url{https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo}}
}
```

## License

This project is licensed under the MIT License.
