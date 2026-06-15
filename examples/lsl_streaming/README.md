# LSL Streaming Examples

Lab Streaming Layer (LSL) integration for real-time EMG streaming and visualization.

## Prerequisites

Install LSL support:
```bash
pip install pylsl
```

Additional visualization dependencies:
```bash
pip install pyqtgraph pyqt5 numpy
```

---

## Control Examples

### 1. `LSL/lsl_classifier_trigger.py`
**EMG classifier-based gesture control**

Uses a trained classifier to decode gestures from EMG and control the exo.

```bash
python LSL/lsl_classifier_trigger.py --port COM4 --baudrate 115200
```

**Arguments:**
- `--port`: Serial port for exo (default: COM4)
- `--baudrate`: Serial baudrate (default: 115200)
- `--verbose`: Enable debug output

---

### 2. `LSL/lsl_gesture_controller.py`
**LSL marker-based gesture control**

Listens to LSL marker streams and executes gestures on the exo.

```bash
python LSL/lsl_gesture_controller.py --port COM4 --type Markers --name EMGGesture
```

**Arguments:**
- `--type`: LSL stream type (default: Markers)
- `--name`: LSL stream name (default: EMGGesture)
- `--port`: Serial port (default: COM4)
- `--baudrate`: Serial baudrate (default: 115200)
- `--timeout`: LSL connection timeout (default: 5.0s)
- `--verbose`: Enable debug output

**Marker Format:**
Expected formats: `KeyGripOpen`, `HandClose`, `IndexPinchClose`, etc.

Automatically maps marker names to exo gestures:
- `indexpinch` → `pinch_index`
- `middlepinch` → `pinch_middle`
- `hand` → `grasp`

---

### 3. `LSL/lsl_state_trigger.py`
**RMS threshold-based control**

Triggers gestures based on EMG RMS exceeding thresholds.

```bash
python LSL/lsl_state_trigger.py --port COM4
```

**How it works:**
- Calculates RMS of EMG channels
- Detects threshold crossings
- Maps to gesture states
- Controls exo accordingly

---

## Visualization Examples

### 1. `LSL/lsl_stacked_plot.py`
**Real-time stacked EMG plotting**

Display multiple EMG channels as stacked waveforms.

```bash
python LSL/lsl_stacked_plot.py --type EMG --name OpenEphysEMG
```

**Features:**
- Multi-channel display
- Adjustable time window
- Auto-scaling
- Real-time updates

---

### 2. `LSL/lsl_grid_plot.py`
**Grid-based EMG visualization**

Display EMG channels in a grid layout matching electrode positions.

```bash
python LSL/lsl_grid_plot.py --type EMG --name OpenEphysEMG
```

**Features:**
- Spatial electrode mapping
- Simultaneous channel view
- Useful for HD-EMG arrays

---

### 3. `LSL/lsl_rms_barplot.py`
**RMS bar plot visualization**

Display RMS amplitude of each EMG channel as bars.

```bash
python LSL/lsl_rms_barplot.py --type EMG --name OpenEphysEMG
```

**Features:**
- Real-time RMS calculation
- Bar chart display
- Threshold visualization
- Channel activity monitoring

---

## Testing Examples

### 1. `LSL/lsl_broadcast_test.py`
**LSL broadcasting test**

Test LSL stream creation and broadcasting.

```bash
python LSL/lsl_broadcast_test.py
```

---

### 2. `LSL/lsl_subscribe_test.py`
**LSL subscription test**

Test LSL stream discovery and data reception.

```bash
python LSL/lsl_subscribe_test.py
```

---

## LSL Architecture

```
EMG Acquisition → LSL Outlet → Network → LSL Inlet → Processing → Exo Control
```

**Key Components:**
- **LSLClient**: Subscribe to EMG streams
- **LSLMarkerSubscriber**: Subscribe to marker streams
- **LSLMessagePublisher**: Publish control messages
- **GestureController**: High-level gesture management

---

## Common LSL Patterns

### Subscribe to EMG Stream
```python
from nml_hand_exo.interface import LSLClient

client = LSLClient(stream_type="EMG", stream_name="OpenEphysEMG")
data, timestamp = client.pull(timeout=1.0)
```

### Subscribe to Markers
```python
from nml_hand_exo.interface import LSLMarkerSubscriber

def on_marker(value, timestamp):
    print(f"Marker: {value} at {timestamp}")

sub = LSLMarkerSubscriber(stream_type="Markers", stream_name="Events")
sub.set_callback(on_marker)
```

### Publish Messages
```python
from nml_hand_exo.interface import LSLMessagePublisher

pub = LSLMessagePublisher(name="ExoControl", channel_count=1)
pub.push_sample(["gesture_start"])
```

---

## Troubleshooting

**No streams found:**
- Check LSL outlet is running
- Verify stream name/type match
- Check network connectivity
- Use `pylsl.resolve_streams()` to debug

**High latency:**
- Reduce processing complexity
- Use faster sampling rates
- Check network bandwidth
- Optimize callback functions

**Dropped samples:**
- Increase buffer size
- Check CPU usage
- Reduce visualization refresh rate
- Use pull_chunk() for bulk reads

---

## Integration with Other Systems

LSL can integrate with:
- **OpenEphys**: Neural recording
- **BioSemi**: EEG/EMG acquisition
- **Unity**: Virtual reality
- **MATLAB**: Data analysis
- **Python**: Custom processing

All examples work with any LSL-compatible source!
