# NML_Hand_Exo

<p align="center">
  <img src="docs/source/_static/new exo.png" width="80%"/>
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

Code was tested on **Windows 11**, **Python 3.10**.

---

## Installation 

### 1. Clone the Repository

```bash
git clone https://github.com/Neuro-Mechatronics-Interfaces/NML_Hand_Exo.git
cd NML_Hand_Exo
```

### 2. Create and activate a virtual environment (recommended)

Use a local environment directory inside the repo, and keep it out of Git.

Preferred local venv name: `.venv`

Windows PowerShell:
```powershell
python -m venv .venv
Set-ExecutionPolicy -Scope Process -ExecutionPolicy RemoteSigned
.\.venv\Scripts\Activate.ps1
```

Windows CMD:
```cmd
python -m venv .venv
.\.venv\Scripts\activate.bat
```

macOS / Linux:
```bash
python3 -m venv .venv
source .venv/bin/activate
```

### 3. Install Python dependencies

Once the venv is active:
```bash
python -m pip install --upgrade pip setuptools wheel
python -m pip install -e .
```

This repo uses `pyproject.toml` as the authoritative dependency source. If you want to install the local package in editable mode, use `pip install -e .`.

Optional extra package set for Max WTF applications:
```bash
python -m pip install -e src/nml_wtf_exo
```

### 4. Launch the GUI

```bash
python src/nml_hand_exo/applications/hand_exo_gui.py
```

### 5. Verify the active interpreter

```bash
python --version
python -c "import sys; print(sys.executable)"
python -c "import nml_hand_exo; print(nml_hand_exo.__file__)"
```

Make sure these commands run from the same activated virtual environment.

### Notes

- Do not commit local virtual environment folders such as `.venv/`, `.handexo/`, or `.handexo311/`.
- If you see stale or broken env behavior after pulling changes, recreate the venv instead of trying to patch it in place.
- If you switch Python versions, delete `.venv/` and recreate it.


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

### GUI Application

The primary way to interact with the exoskeleton is through the PyQt5 desktop GUI:

```bash
python src/nml_hand_exo/applications/hand_exo_gui.py
```

The GUI provides:

- **Connection panel** — connect via USB serial port or Bluetooth (HC-05). Auto-detects available COM ports.
- **Controls tab**
  - Per-motor angle sliders and enable/disable toggles
  - Gesture presets (open, close, pinch, etc.) with a lazy-initialize safety gate
  - **Calibration** — opens a guided dialog for streaming joint-limit calibration. Operator moves each DoF through its full range during two global streaming phases (extension, then flexion). Results are saved as a per-user profile JSON.
  - **ROM Assessment** — opens a dialog to run a range-of-motion protocol and export data to `output_data/<name>_rom_<date>_<run>.csv`.
- **Telemetry tab** — live table showing position, torque, and current for all 9 motors. Includes a manual Refresh button and an Auto-refresh checkbox (250 ms polling).
- **Log panel** — timestamped command/response log at the bottom of the window.

Calibration profiles are stored in `examples/calibration/profiles/<name>.json`. A default profile can be set in `profiles/config.json` and is applied automatically before gesture commands.

### Python API

An example of using the Python API for scripting and control:

```python
from nml_hand_exo.hand_exo import HandExo

exo = HandExo('COM3', baudrate=57600)
exo.enable_motor(1)
exo.set_motor_angle(1, 45)
angle = exo.get_motor_angle(1)
print(f"Motor angle: {angle} degrees")
exo.disable_motor(1)
```

You can control the exoskeleton over USB or Bluetooth using structured serial commands:

- `set_angle:WRIST:30` — set wrist motor to 30 degrees.
- `enable:1` — enable motor 1 torque.
- `get_angle:1` — query relative angle.

Supported motor aliases: `THUMB`, `INDEX`, `MIDDLE`, `RING`, `PINKY`, `WRIST`

For a complete list of commands, see the [Usage Guide](https://neuro-mechatronics-interfaces.github.io/NML_Hand_Exo/usage.html).

### CLI Calibration

A command-line calibration wizard is also available:

```bash
python examples/calibration/calibrate_exo.py
```

This saves a profile JSON and updates `src/cpp/nml_hand_exo/config.h` with the calibrated joint limits.

## Demo

#### MindRove EMG Streaming

![](/docs/source/_static/pyqtemg.gif)

A demo script is included to showcase real-time plotting of EMG signals from a connected MindRove EMG band.
1) Connect your MindRove EMG Band to the PC (using a Wifi dongle if you want to maintain internet connection on a separate wifi network)
2) Run the demo script
   ```
   python demo_mindrove_realtime.py
   ```

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
