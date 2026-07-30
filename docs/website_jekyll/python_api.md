---
layout: single
title: Python API Documentation
permalink: /python-api/
toc: true
toc_sticky: true
description: "Complete Python API documentation for the NML Hand Exoskeleton package."
header:
  overlay_color: "#1e1e1e"
  overlay_filter: "0.5"
---

# Overview

The **`nml_hand_exo`** package lets you:

- Connect to the Hand Exoskeleton over **Serial** or **TCP**
- Enable/disable motors, **set angles/velocities**, read status and **IMU**
- Manage **limits** (current/angle), **homing**, **modes**, and **gestures**
- Stream/consume data via **Lab Streaming Layer (LSL)**
- **Plot** real-time signals and build simple **gesture triggers**

Core modules:

- `nml_hand_exo.interface` — device & comms (✅ *use this first*)
- `nml_hand_exo.processing` — filters & features (EMG/utility)
- `nml_hand_exo.plotting` — PyQt + pyqtgraph views
- `nml_hand_exo.control` — simple RMS-based trigger logic
- `nml_hand_exo.applications` — example GUI (`hand_exo_gui.py`)

---

# Install locally (Windows 11)

> Requires Python **3.10+**.

**Option A — dev install (recommended)**
```powershell
# From your repo root
py -3.10 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -U pip wheel

# If your package is in a "src" layout (pyproject.toml has: package-dir = { "" = "src" })
pip install -e ".[docs]"
```

**If your package folder isn’t under `src/`:**  
Either move `nml_hand_exo/` into `src/`, **or** edit `pyproject.toml`:

```toml
[tool.setuptools]
packages = ["nml_hand_exo"]
package-dir = { "" = "." }   # ← use "." if your package is at the repo root
```

Then:
```powershell
pip install -e ".[docs]"
```

**Option B — one-shot install from the folder**
```powershell
pip install .
```

**Sanity check**
```powershell
python - <<'PY'
import nml_hand_exo as m
print("nml_hand_exo version:", getattr(m, "__version__", "<unknown>"))
print("modules:", m.__all__ if hasattr(m, "__all__") else "<n/a>")
PY
```

---

# Quickstart: connect over **Serial**

```python
from nml_hand_exo.interface import HandExo, SerialComm

# USB firmware uses 1000000 baud and semicolon-delimited responses.
comm = SerialComm(port="COM5", baudrate=1000000, command_delimiter=";", timeout=1)

# HandExo expects a comm object at construction
exo = HandExo(comm, name="NMLHandExo", verbose=True)
exo.connect()

# Basic probes
print("FW:", exo.version())       # device firmware info (string)
print("Help:
", exo.help())      # list of supported tokens from firmware

# Enable + move
exo.enable_motor('all')
exo.set_motor_velocity('all', 0.5)     # 0..1 (depends on firmware mapping)
exo.set_motor_angle(1, 10.0)           # move motor 1 to 10 degrees

# Read back status
a1 = exo.get_motor_angle(1)
abs_a1 = exo.get_absolute_motor_angle(1)
print("M1 angle:", a1, "absolute:", abs_a1)

# Safe shutdown
exo.disable_motor('all')
exo.close()
```

**Tip (EOL mismatch):** If commands are ignored, your firmware may expect **CRLF** or `\n` EOL. When using `SerialComm`, the **delimiter** you pass is what’s appended to every command. Try `command_delimiter="\r\n"`.

---

# Quickstart: connect over **TCP**

```python
from nml_hand_exo.interface import HandExo, TCPComm

comm = TCPComm(ip="192.168.0.42", port=5001, timeout=5, verbose=True)
exo = HandExo(comm, verbose=True)
exo.connect()

print(exo.version())
exo.enable_motor('all')
exo.set_motor_angle(2, 15.0)
exo.disable_motor('all')
exo.close()
```

---

# HandExo — common operations

Below are **real method names** you can call from `HandExo`. Availability depends on your firmware’s command set (run `exo.help()` to confirm).

## Power, homing, LEDs
```python
exo.enable_motor('all')        # or motor id/name
exo.disable_motor('all')
exo.is_enabled('all')          # bool (may require polling)

exo.home('all')                # perform homing
exo.get_home('all')            # query stored home
exo.set_home('all')            # set current pos as home

exo.enable_led(1)
exo.disable_led(1)
```

## Motion & limits
```python
# Angles
exo.set_motor_angle(1, 12.5)
angle = exo.get_motor_angle(1)
abs_angle = exo.get_absolute_motor_angle(1)
exo.set_absolute_motor_angle(1, 0.0)

# Velocity
exo.set_motor_velocity('all', 0.5)
v1 = exo.get_motor_velocity(1)

# Limits
exo.set_current_limit('all', 0.8)
lim_i = exo.get_motor_current_limit(1)

exo.set_motor_limits(1, lower=-5.0, upper=45.0)
lims = exo.get_motor_limits(1)
exo.set_motor_upper_limit(1, 45.0)
exo.set_motor_lower_limit(1, -5.0)
```

## Sensors & status
```python
torque = exo.get_motor_torque(1)
amps  = exo.get_motor_current(1)

imu    = exo.get_imu_data()     # raw vector if supported
rpy    = exo.get_imu_angles()   # (roll, pitch, yaw)
roll   = exo.get_imu_roll()
pitch  = exo.get_imu_pitch()
yaw    = exo.get_imu_heading()
```

## Modes & gestures
```python
exo.set_exo_mode("assist")             # depends on firmware
print(exo.get_exo_mode())

exo.set_gesture("IndexPinch")
exo.set_gesture_state("open")          # or "close"
print(exo.get_gesture())               # current gesture
print(exo.get_gesture_state())         # "open"/"close"
print(exo.get_gesture_list())          # list of names (if supported)
exo.cycle_gesture()
exo.cycle_gesture_state()
```

## Misc
```python
print(exo.get_motor_mode(1))
exo.set_motor_mode(1, "position")      # depends on firmware
exo.reboot_motor(1)
```

---

# GestureController (convenience)

```python
from nml_hand_exo.interface import HandExo, SerialComm, GestureController

exo = HandExo(SerialComm("COM5", 1000000, ";", 1), verbose=False)
exo.connect()

gc = GestureController(exo, verbose=True)
gc.set_gesture("IndexPinch", "open")
gc.set_gesture("IndexPinch", "close")
print("Current gesture:", gc.get_current_gesture())
```

---

# LSL — subscribing & publishing

## Subscribe with `LSLClient`
```python
from nml_hand_exo.interface import LSLClient

# Choose exactly one: stream_name OR stream_type
client = LSLClient(stream_name="ExoTelemetry", auto_start=True, verbose=True)

# Latest 1000 ms window as (channels x samples) NumPy array
window = client.get_latest_window(window_ms=1000)

# Metadata: name, type, fs, n_channels, labels/units (if provided)
meta = client.get_metadata()
print(meta)

client.stop_streaming()
```

## Publish marker messages
```python
from nml_hand_exo.interface import LSLMessagePublisher

with LSLMessagePublisher(name="ExoMarkers", stream_type="Markers") as pub:
    pub.publish("start")                      # simple string
    pub.publish('{"gesture":"pinch"}')        # or your own JSON encoding
```

## Subscribe to markers/numerics (callbacks)
```python
from nml_hand_exo.interface import LSLMarkerSubscriber, LSLNumericSubscriber

def on_marker(ts, value):
    print(f"[marker @ {ts:.3f}] {value}")

def on_numeric(ts, vec):
    print(f"[numeric @ {ts:.3f}] {vec}")

markers = LSLMarkerSubscriber(name="ExoMarkers", on_marker=on_marker)
nums    = LSLNumericSubscriber(name="ExoTelemetry", on_sample=on_numeric)

with markers, nums:
    print("Listening... Press Ctrl-C to stop.")
    import time; 
    time.sleep(10)
```

---

# Real-time plotting (PyQt + pyqtgraph)

```python
from PyQt5 import QtWidgets
from nml_hand_exo.interface import LSLClient
from nml_hand_exo.plotting import StackedPlotter

client = LSLClient(stream_name="ExoTelemetry", auto_start=True)

app = QtWidgets.QApplication([])
win = StackedPlotter(
    client,
    channels_to_plot=[0,1,2,3,4,5],
    interval_ms=30,
    buffer_secs=2,
    downsample_factor=1,
    enable_car=False, enable_bandpass=False, enable_notch=False,
)
win.show()
app.exec_()
```

There’s also a grid layout:
```python
from nml_hand_exo.plotting import GridPlotter
grid = GridPlotter(client, rows=2, cols=3, buffer_secs=2)
grid.show()
```

---

# Processing utilities (EMG/features)

```python
import numpy as np
from nml_hand_exo.processing import (
    RealtimeEMGFilter,  # live-bandpass/notch/CAR in the plotter
    rectify, window_rms, window_rms_1D, compute_rms,
    compute_rolling_rms, downsample, common_average_reference,
    compute_grid_average, z_score_norm, orthogonalize, normalize,
)

x = np.random.randn(8, 2000)     # 8 channels, 2k samples
xr = rectify(x)
rms = window_rms(xr, win=200, step=50)
```

---

# Simple RMS-triggered controller

```python
from nml_hand_exo.control import StateTriggerRMS
from nml_hand_exo.interface import HandExo, SerialComm, LSLClient

exo = HandExo(SerialComm("COM5", 1000000, ";", 1))
exo.connect()

client = LSLClient(stream_name="ExoTelemetry", auto_start=True)

ctrl = StateTriggerRMS(
    exo, client,
    threshold_percent=0.10,     # 10% above baseline
    window_ms=200,
    rest_duration_sec=5,
    verbose=True,
)

# Map gestures to channel groups
ctrl.channel_groups = {
  "IndexPinch": [0,1],
  "ThumbFlex" : [2,3],
}

# (Pseudo) loop
for _ in range(100):
    ctrl.step()     # reads LSL, checks RMS, may call exo.set_gesture(...)
```

> Keep currents/velocities conservative for first runs. Clear workspace and use an E-stop.

---

# Example GUI

A minimal GUI lives at:
```
nml_hand_exo/applications/hand_exo_gui.py
```

Try:
```powershell
python -m nml_hand_exo.applications.hand_exo_gui
```

---

# Troubleshooting

- **No serial ports on Windows:** try another cable/USB port; install **CDC/WinUSB** using Zadig if needed.  
- **Commands ignored:** switch EOL/delimiter — if firmware expects CRLF, set `command_delimiter="\r\n"`.  
- **High CPU in plots:** reduce channels or increase `interval_ms`; avoid printing inside tight loops.  
- **Install fails with "src layout" error:** either move the package into `src/` **or** set `package-dir = { "" = "." }` in `pyproject.toml`.  
- **TCP timeout:** check IP/port, firewall, and that the device server is running.

---

# Next Steps

<div class="feature__wrapper">
  <div class="feature__item">
    <div class="archive__item">
      <div class="archive__item-body">
        <h2 class="archive__item-title">🚀 Examples</h2>
        <div class="archive__item-excerpt">
          <p>Step-by-step tutorials and code examples for common tasks.</p>
        </div>
        <p><a href="/NML_Hand_Exo/examples/" class="btn btn--primary">View Examples</a></p>
      </div>
    </div>
  </div>

  <div class="feature__item">
    <div class="archive__item">
      <div class="archive__item-body">
        <h2 class="archive__item-title">🛠️ Assembly Guide</h2>
        <div class="archive__item-excerpt">
          <p>Hardware assembly instructions and electrical integration.</p>
        </div>
        <p><a href="/NML_Hand_Exo/assembly/" class="btn btn--primary">View Assembly</a></p>
      </div>
    </div>
  </div>

  <div class="feature__item">
    <div class="archive__item">
      <div class="archive__item-body">
        <h2 class="archive__item-title">💬 Community</h2>
        <div class="archive__item-excerpt">
          <p>Get help, report issues, and contribute to the project.</p>
        </div>
        <p><a href="/NML_Hand_Exo/community/" class="btn btn--primary">Join Community</a></p>
      </div>
    </div>
  </div>
</div>

---

# API Index (Quick Reference)

**Comms:** `SerialComm(port, baudrate, command_delimiter=';', timeout=1)`, `TCPComm(ip, port=5001, timeout=5)`

**Device (`HandExo`)**
- Connection & info: `connect()`, `close()`, `send_command(str)`, `help()`, `version()`
- Power/LEDs: `enable_motor(id|'all')`, `disable_motor(id|'all')`, `is_enabled(id)`, `enable_led(id)`, `disable_led(id)`
- Homing: `home(id|'all')`, `get_home(id)`, `set_home(id)`
- Motion: `set_motor_angle(id, deg)`, `get_motor_angle(id)`, `get_absolute_motor_angle(id)`, `set_absolute_motor_angle(id, deg)`, `set_motor_velocity(id|'all', v)`
- Limits: `set_current_limit(id|'all', amps)`, `get_motor_current_limit(id)`, `set_motor_limits(id, lower, upper)`, `get_motor_limits(id)`, `set_motor_upper_limit(id, val)`, `set_motor_lower_limit(id, val)`
- Status: `get_motor_torque(id)`, `get_motor_current(id)`
- Modes & gestures: `get_motor_mode(id)`, `set_motor_mode(id, mode)`, `get_exo_mode()`, `set_exo_mode(mode)`, `get_gesture()`, `set_gesture(name)`, `set_gesture_state('open'|'close')`, `get_gesture_state()`, `get_gesture_list()`, `cycle_gesture()`, `cycle_gesture_state()`
- IMU: `get_imu_data()`, `get_imu_angles()`, `get_imu_roll()`, `get_imu_pitch()`, `get_imu_heading()`

**LSL:** `LSLClient(stream_name|stream_type, auto_start=True)`, `LSLMessagePublisher`, `LSLMarkerSubscriber`, `LSLNumericSubscriber`

**Plotting:** `StackedPlotter`, `GridPlotter`

**Processing:** `RealtimeEMGFilter`, `rectify`, `window_rms`, `compute_rms`, `downsample`, `common_average_reference`, `compute_grid_average`, `z_score_norm`, `orthogonalize`, `normalize`, `OrientationFilter`

**Convenience:** `GestureController`, `StateTriggerRMS`
