# NML WTF Exo #
This is for Max developmental/side-branch code.

## Quick Start ##
From repository root:  
```bat
pip install -e src/nml_wtf_exo
```

## Scripts Overview ##
To launch a [GUI for interactions](#controller-gui) with the Exo, you can now run:
```bat
nml-wtf-exo
```
To launch a [demo GUI](#demo-gui) for driving the Exo with different parametric motion functions (e.g. sinusoid, triangle, step, white or colored noise), you can  run:
```bat
nml-wtf-exo-demo
```

### Controller GUI ###
Launches the Hand Exo — LSL/Manual Controller GUI (ExoControlApp). To open:  
```bat
nml-wtf-exo
```

#### Exo Connection ####
* Lists serial ports; choose baud (default 57600), connect/disconnect.
* Persists last used port/baud in:
* PATHS['configs_dir']/.nml_hand_exo_gui.xml

#### Control Mode ####
* Manual – nudge a selected motor by Δ°; torque and LED toggles.
* LSL – drive motors from an LSL stream (multi-joint or single-channel).

##### Manual panel #####
* Pick Motor ID (0–5), set Δ angle, Torque ON/OFF, LED ON/OFF.
* Shows current angle and torque state (polled periodically).

##### LSL panel #####
* Refresh Streams to discover LSL outlets.
* Settings… – set OneEuro filter params and per-motor gains.
* Calibrate LSL – grabs one sample, initializes filters, defines “home.”
* Start/Stop LSL Control – begins polling (default ~100 Hz).
* Fix range – applies θ → θ−90° pre-transform (toggle).
* Log LSL while controlling – writes CSV logs via StreamLogger.
* Multi-joint map (L/R) – auto-maps six channels for the chosen hand
  - _(expects labels like L.WristFE, Thumb_R, etc.; tolerant parsing)._
* Single-channel legacy – map one stream channel → one motor with (scale, offset).

##### Raw Command (advanced) #####
* Send raw serial commands (e.g., info, help) to the device.
* Adjustable timeout; prints response to terminal and a status line in the GUI.

### Demo GUI ###
This is a very basic GUI intended for use to show how the relative position commands can be used via software interface to control the state of the hand exo, manipulating individual digits asynchronously via master loop that updates each motor state sequentially every 100-milliseconds. Currently, there is no logging associated (although some of the framework exists to save the motion configuration, and it wouldn't be too hard to add a logging component). 

#### How to run ####
```bash
nml-wtf-exo-demo
```
By default, the motion/joint parameterization in `PATHS['exo_demo_default_motion.json']` are used for the initial per-joint configurations (which are shown in the gif below):  
![This should be a gif with GUI capture and accompanying exo OBS video](/src/nml_wtf_exo/resources/2025-11-16_Hand-Exo-Demo.gif)


### Viewer GUI ###
Launches a **landmark log viewer** for CSV recordings (e.g., from your LSL logger).  
It visualizes 2-D landmark trajectories over time and lets you scrub/play them with basic controls.

#### How to run ####
```bash
nml-wtf-exo-viewer
```
By default it opens the directory in PATHS["landmarks_dir"].

#### What it does #### 
1. Reads a CSV into a LogReader (timestamps + flattened landmark coordinates).
2. Renders landmarks as a scatter plot (matplotlib in a Qt window).
3. Plays back frames at recorded timing (with adjustable speed).
4. Lets you highlight an individual landmark (via the right-side list).
5. Can flip Y for UI-style coordinates (top-left origin) vs math-style (bottom-left).

#### UI ####
* Directory + Refresh: choose a folder and list all .csv files.
* File list: double-click or select + Load to open a CSV.
* Playback: Playback starts, Pause stops; Speed sets 0.1×–5×.
* Flip Y (UI coords): toggles y → 1 − y so [0,1] remains in view.
* Landmark list (right pane): select an entry to enlarge that point on the plot.
* The plot shows normalized coordinates in [0,1]×[0,1], grid on, equal aspect.

#### Landmark labels & channels
* If the CSV (or its embedded metadata) includes per-channel labels (e.g., wrist.x, wrist.y), the viewer groups every N channels into a single landmark label (where N is dims_per_landmark()—typically 2).
* If no labels are available, landmarks are named lm[0], lm[1], …

#### Expected CSV shape #### 

The viewer expects a regular, row-per-frame layout, for example:

```csv
# optional header/meta lines are allowed
timestamp,x0,y0,x1,y1,x2,y2
0.000,0.42,0.58,0.31,0.27,0.80,0.10
0.020,0.43,0.57,0.32,0.28,0.79,0.11
...
```

* `timestamp`: seconds or milliseconds (the LogReader handles the unit it wrote).
* `xk`, `yk`: normalized positions; if your source logs pixels, pre-normalize or ensure your LogReader normalizes on read.
* 3-D logs (`x,y,z` per landmark) are supported; the viewer plots x/y.

#### Tips ####
* If the window title shows (..., N landmarks, D dims), then there are N landmarks and D dims per landmark (2 or 3).
* The viewer auto-flips Y based on the reader’s hint (recommended_flip_y()); you can override with the checkbox.
* Highlighting: picking a label in the right pane enlarges that landmark in the scatter.

#### Troubleshooting ####
* Blank file list: click Refresh; verify the Directory path; only .csv are listed.
* “Load failed”: check that your CSV is well-formed and that LogReader can parse it.
* No movement on Playback: verify timestamps increase and peek_delta_to_next_ms() returns reasonable values; try Speed = 1.0×.
* Backend errors: the app forces Qt5Agg if a different matplotlib backend is active; ensure PyQt5 is installed in the same venv.

### Logger GUI ###
A simple GUI to record **LSL** streams to disk. Supports both continuous numeric streams (e.g., landmarks/EMG features) and **Markers** streams (key–value events). Run using:
```bash
nml-wtf-exo-logger
```

#### What it does #### 
1. Discovers available LSL streams (resolve_streams()).
2. If the selected stream’s type is "Markers", uses ParameterLogger (event log).
3. Otherwise uses StreamLogger (continuous samples).
4. Creates the output folder if needed and writes time-stamped CSV files.
5. Shows the active destination path in the status line.

#### UI ####
1. Stream to log / Refresh Streams – pick the LSL source (name + source_id).
2. Base filename suffix – freeform text appended to the log base name.
3. Select Log Folder – choose where CSV files will be written (default: landmarks/).
4. Start / Stop Logging – begin/end capture. Status displays the base filename.

#### Output files ####
This logger generates multiple output file-types, as described below.

##### Continuous (StreamLogger): #####
* CSV with header and per-row timestamp + channel values.
* Typical columns: timestamp,x0,y0,x1,y1,...
Filename pattern (example):
`landmarks/<stream-name>_<YYYYmmdd_HHMMSS>_<suffix>.csv`

##### Markers (ParameterLogger): #####
* CSV with timestamps and decoded marker payloads (e.g., loop_ts,name,value).
Filename pattern (example):
`landmarks/markers_<YYYYmmdd_HHMMSS>_<suffix>.csv`

#### Notes #### 
* You can start the logger even without connecting the exo; it only depends on LSL.
* For high-rate streams, disk I/O is buffered internally by the logger classes.
* The app embeds a StreamInlet into the logger instance (logger.inlet = StreamInlet(...)) before start().

#### Tips & Troubleshooting ####
* No streams shown → Click Refresh Streams, ensure the producer is running on the same network.
* Permission error on folder → Pick a writable Log Folder (avoid system dirs).
* Nothing written → Check that the selected stream is actively sending (some outlets only push on demand).
* Playback → Open the resulting CSVs with nml-wtf-exo-viewer to visualize 2-D landmarks over time.

### Keyboard GUI ###
This is basically completely unrelated to the exo and I think I just brought it "along for the ride" while copy/pasting from my other repo. Maybe eventually this would be involved in testing the exo? Call it using:  
```bat
nml-wtf-exo-keyboard
```
