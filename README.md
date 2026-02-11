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
from nml_hand_exo.hand_exo import HandExo

exo = HandExo('COM3', baudrate=57600)
exo.enable_motor(1)
exo.set_motor_angle(1, 45)
angle = exo.get_motor_angle(1)
print(f"Motor angle: {angle} degrees")
exo.disable_motor(1)
```

You can control the hand exoskeleton over USB or Bluetooth using simple, structured serial commands. For example:

- `set_angle:WRIST:30` — set wrist motor to 30 degrees.

- `enable:1` — enable motor 1 torque.

- `get_angle:1` — query relative angle.

Supported aliases are `THUMB`, `INDEX`, `MIDDLE`, `RING`, `PINKY`, `WRIST`

For a complete list of commands, see the [Usage Guide](https://neuro-mechatronics-interfaces.github.io/NML_Hand_Exo/usage.html).

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

## License

This project is licensed under the MIT License.
