import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import queue
import time
import math
import random
import json
import csv
import os
import builtins
import collections

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.gridspec import GridSpec
from scipy.stats import pearsonr
from scipy.signal import butter, filtfilt, iirnotch, detrend
import shap

from sklearn.ensemble import RandomForestRegressor, GradientBoostingRegressor
from sklearn.svm import SVR
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import KFold, GroupKFold, cross_val_predict
from sklearn.metrics import r2_score, mean_squared_error
from sklearn.pipeline import Pipeline

try:
    from xgboost import XGBRegressor
    XGB_AVAILABLE = True
except ImportError:
    XGB_AVAILABLE = False

try:
    from lightgbm import LGBMRegressor
    LGB_AVAILABLE = True
except ImportError:
    LGB_AVAILABLE = False

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    from screeninfo import get_monitors
    SCREENINFO_AVAILABLE = True
except ImportError:
    SCREENINFO_AVAILABLE = False

try:
    from gdx import gdx as gdx_lib
    GDX_AVAILABLE = True
except ImportError:
    GDX_AVAILABLE = False

try:
    from pylsl import StreamInlet, resolve_streams
    LSL_AVAILABLE = True
except ImportError:
    LSL_AVAILABLE = False

try:
    from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds
    from mindrove.data_filter import DataFilter, FilterTypes, DetrendOperations
    MINDROVE_AVAILABLE = True
except ImportError:
    MINDROVE_AVAILABLE = False

# ─────────────────────────────────────────────
#  DEVICE SELECTION  ("umyo" or "mindrove")
# ─────────────────────────────────────────────
ACTIVE_DEVICE = "mindrove"   # default to MindRove; can still switch at runtime

# ─────────────────────────────────────────────
#  THEME  (identical to your existing GUI)
# ─────────────────────────────────────────────
BG        = "#0a0c0f"
PANEL     = "#111418"
PANEL2    = "#161b22"
BORDER    = "#2a3040"
ACCENT    = "#00d4ff"
ACCENT2   = "#ff6b35"
GREEN     = "#39d353"
YELLOW    = "#f0c040"
RED       = "#ff4757"
PURPLE    = "#a855f7"
TEXT      = "#c9d1d9"
MUTED     = "#586069"
FONT_MONO    = ("Courier New", 10)
FONT_MONO_SM = ("Courier New", 9)
FONT_COND    = ("Arial", 11, "bold")
FONT_COND_LG = ("Arial", 20, "bold")

# ─────────────────────────────────────────────
#  CONFIG
# ─────────────────────────────────────────────
EMG_BAUD        = 115200
NUM_SENSORS     = 1
BINS_PER_SENSOR = 4
HIST_LEN        = 300
GDX_POLL_MS     = 50
MVC_DURATION    = 5.0     # seconds for MVC calibration
ROBOT_RFO_CAL_DURATION = 3.0  # seconds to ramp motor current 0 -> max for RFO calibration
FORCE_LEVELS    = [0, 25, 50, 75, 100]
HOLD_SEC        = 3
REST_SEC        = 3
NUM_REPS        = 3
RAMP_SEC        = 8
RAMP_PEAKS      = [70, 90, 100]   # per-rep ramp peak (% MVC), cycles if reps > len
RAMP_PEAK_HOLD_SEC = 1.0          # brief hold at peak before ramping back down
# Every ramp climbs from 0%, so the low/mid range is covered by all reps
# regardless of peak — only the 75-100% band depends on peak height. With
# the old [50,75,100], just 1 of 3 reps (the 100% one) ever touched that
# band, starving it of training density and causing the model to undershoot
# high-effort peaks. Shifting peaks up to [70,90,100] gets 2 of 3 reps into
# 75-100% instead of 1, without changing NUM_REPS/session duration (which
# would also drift the 40s effort-graph default out of sync again).

# Feature vector layout (built from 2 sensors × (1 act + 8 bins) = 18 raw values)
# Plus derived cross-channel features → 30 total
def _build_feature_names(n_sensors=None, bins=None):
    ns = n_sensors if n_sensors is not None else NUM_SENSORS
    bp = bins      if bins      is not None else BINS_PER_SENSOR
    return (
        [f"s{s}_act"        for s in range(ns)] +
        [f"s{s}_b{b}"       for s in range(ns) for b in range(bp)] +
        [f"s{s}_mean"       for s in range(ns)] +
        [f"s{s}_std"        for s in range(ns)] +
        [f"s{s}_slope"      for s in range(ns)] +
        [f"s{s}_peak_ratio" for s in range(ns)] +
        ["diff_mean", "diff_std", "ratio_mean", "total_act",
     "diff02_mean", "diff12_mean", "total_act3"]
    )

FEATURE_NAMES = _build_feature_names()

# ─────────────────────────────────────────────
#  HARDWARE STATE  (module-level singletons)
# ─────────────────────────────────────────────
emg_serial        = None
gdx_handle        = None
_gdx_last_force   = [0.0]
_gdx_bg_running   = threading.Event()
_emg_running      = threading.Event()
_lsl_force_inlet  = None       # pylsl StreamInlet when force source = LSL
_lsl_bg_running   = threading.Event()

# ── Robot Integration dual-scale mirror state ────────────────────────────────
# A single gdx_handle now opens BOTH physical GDX scales (if both are
# plugged in) right from the main "CONNECT GDX" button. Tabs 1-6 / the exo
# tab only ever read _gdx_last_force[0] (scale 1), so a second scale being
# connected is invisible to them. The robot tab reads
# _robot_gdx_scale1[0]/_robot_gdx_scale2[0], which the same background
# thread (_gdx_bg) keeps in sync. There is no separate dual handle/thread
# any more — that toggle between a single- and dual-device gdx_lib.gdx()
# instance was what crashed hidapi (class-level shared state + reopening an
# already-open USB HID path).
_robot_gdx_scale1  = [0.0]       # human grip force from GDX device 1 (mirrors _gdx_last_force)
_robot_gdx_scale2  = [0.0]       # robot pincher force from GDX device 2 (0.0 if only one scale connected)
_gdx_bg_thread     = None        # Thread running _gdx_bg(), so teardown can join() it
_gdx_device_count  = 0           # how many physical GDX scales are currently open (0, 1, or 2)
_robot_rfo         = 0.1         # Robot Force Output (N): running max of scale-2 force seen this session
_latest_emg     = [0.0] * (NUM_SENSORS * (BINS_PER_SENSOR + 1))
_emg_lock       = threading.Lock()
mvc_reference   = None   # 1-D array, same shape as EMG vector
force_mvc_reference = None  # peak GDX force captured during MVC calibration
mr_session_baseline = np.zeros(8, dtype=float)  # relaxed per-channel baseline for MindRove
MR_BASELINE_ALPHA = 0.01


# ─────────────────────────────────────────────
#  FORCE NORMALISATION HELPERS
# ─────────────────────────────────────────────
def _force_to_pct_mvc(force_n):
    """Convert raw force (N) → % MVC for display. Returns 0 if not calibrated."""
    if force_mvc_reference and force_mvc_reference > 0:
        return float(force_n) / force_mvc_reference * 100.0
    return 0.0

def _arr_to_pct_mvc(arr):
    """Vectorised version of _force_to_pct_mvc for NumPy arrays."""
    if force_mvc_reference and force_mvc_reference > 0:
        return np.asarray(arr, dtype=float) / force_mvc_reference * 100.0
    return np.zeros(len(arr), dtype=float)


# ─────────────────────────────────────────────
#  EMG SERIAL  (reader lives inside App, matching working pipeline)
# ─────────────────────────────────────────────
def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()] if SERIAL_AVAILABLE else []


EMG_DEBUG = True   # set False once EMG is confirmed working

def _parse_emg_line(raw):
    """Parse one serial line from the ESP32.
    Format: act0,b0,b1,b2,b3[,act1,b0,b1,b2,b3[,act2,b0,b1,b2,b3]]
    Sensor count is inferred from column count (5 values per sensor).
    Returns (emg_list, n_sensors) or (None, 0) on failure.
    """
    global NUM_SENSORS, FEATURE_NAMES, _latest_emg
    try:
        parts = [float(v) for v in raw.split(",")]
        stride = BINS_PER_SENSOR + 1        # 5 values per sensor
        n = len(parts) // stride
        n = max(1, min(n, 3))               # clamp to 1-3

        expected = n * stride
        if len(parts) < expected:
            return None, 0

        emg = parts[:expected]

        # Auto-update NUM_SENSORS if hardware reports a different count
        if n != NUM_SENSORS:
            NUM_SENSORS   = n
            FEATURE_NAMES = _build_feature_names(n, BINS_PER_SENSOR)
            _latest_emg   = [0.0] * expected
            if EMG_DEBUG:
                print(f"[EMG] Auto-detected {n} sensor(s) from column count")

        return emg, n
    except Exception:
        return None, 0


def _emg_reader():
    _dbg_count = 0
    while _emg_running.is_set():
        if emg_serial is None or not emg_serial.is_open:
            time.sleep(0.05); continue
        try:
            raw = emg_serial.readline().decode("utf-8", errors="ignore").strip()
            if not raw: continue
            emg, n = _parse_emg_line(raw)
            if emg is not None:
                with _emg_lock:
                    # Resize _latest_emg if sensor count changed
                    if len(_latest_emg) != len(emg):
                        _latest_emg[:] = emg
                    else:
                        _latest_emg[:] = emg
                if EMG_DEBUG and _dbg_count < 5:
                    print(f"[EMG OK]  {n} sensor(s), {len(emg)} values: {emg}")
                    _dbg_count += 1
            else:
                if raw.startswith("dev_count") or raw.startswith("Scanning") or raw.startswith("Invalid"):
                    pass  # status lines — ignore silently
                elif EMG_DEBUG and _dbg_count < 5:
                    print(f"[EMG SKIP] unparseable: {raw[:80]}")
                    _dbg_count += 1
        except ValueError:
            if EMG_DEBUG and _dbg_count < 5:
                print(f"[EMG ERR] non-numeric line: {raw[:80]}")
                _dbg_count += 1
            continue
        except UnicodeDecodeError:
            continue
        except Exception as e:
            if EMG_DEBUG:
                print(f"[EMG EXC] {e}")
            time.sleep(0.01)

def get_emg():
    with _emg_lock:
        return list(_latest_emg)

def open_emg_serial(port):
    global emg_serial
    try:
        # Open with default DTR — the uMyo ESP32 needs the reset pulse
        # to start streaming data (unlike the OpenRB-150 exo controller).
        emg_serial = serial.Serial(port, EMG_BAUD, timeout=1)
        time.sleep(2.0)   # wait for ESP32 to boot and start sending
        emg_serial.reset_input_buffer()
        return True
    except Exception as e:
        print(f"[EMG] {e}"); emg_serial = None; return False


# ─────────────────────────────────────────────
#  GDX SENSOR
# ─────────────────────────────────────────────
def _gdx_bg():
    # gdx.read() already blocks until the sensor delivers the next sample
    # (at the rate set by gdx_handle.start(GDX_POLL_MS)).
    # Adding an extra sleep() on top doubles the latency, so we don't sleep here.
    #
    # gdx_handle may have 1 or 2 physical scales open. read() then returns a
    # list with 1 or 2 values respectively. Scale 1 feeds BOTH
    # _gdx_last_force (read by tabs 1-6 / the exo tab) and _robot_gdx_scale1
    # (read by the robot tab); scale 2 only feeds _robot_gdx_scale2.
    while _gdx_bg_running.is_set():
        if gdx_handle:
            try:
                m = gdx_handle.read()
                if m:
                    if len(m) >= 1 and m[0] is not None:
                        v = max(0.0, float(m[0]))
                        _gdx_last_force[0]   = v
                        _robot_gdx_scale1[0] = v
                    if len(m) >= 2 and m[1] is not None:
                        _robot_gdx_scale2[0] = max(0.0, float(m[1]))
            except Exception:
                time.sleep(0.005)  # only pause on error to avoid a tight error-loop

def setup_gdx():
    """Open ALL physical GDX scales found on USB (1 or 2) on a single handle.

    Only scale 1's force is ever used for collection/training/validation
    (tabs 1-6) and the exoskeleton tab — see _gdx_bg() above. If a second
    scale is plugged in, it's opened too so the robot integration tab can
    show both traces live without needing its own separate "connect" step
    (and without ever re-opening a USB path that's already held open, which
    is what crashed the process when toggling between a single- and
    dual-device gdx_lib.gdx() instance).
    """
    global gdx_handle, _gdx_bg_thread, _gdx_device_count
    if not GDX_AVAILABLE:
        return False, "gdx library not found"

    # Tear down any existing GDX connection first — see _teardown_all_gdx()
    # for why this is required to avoid a hidapi crash.
    _teardown_all_gdx()

    orig_input = builtins.input
    # "1,2" answers the device-selection prompt when 2+ scales are found
    # (gdx asks "type the number(s), e.g. 1,2"); it's simply unused if only
    # 1 device is found, since open_usb() auto-selects a single device.
    builtins.input = lambda p="": "1,2"
    try:
        gdx_handle = gdx_lib.gdx()
        gdx_handle.open(connection="usb")

        n_devices = len(gdx_lib.gdx.devices)
        if n_devices == 0:
            raise RuntimeError("No GDX devices found")

        # Pass sensor indices directly instead of the interactive
        # select_sensors() — avoids relying on mocked input() call counts.
        sensors_arg = [[1], [1]] if n_devices >= 2 else [1]
        try:
            gdx_handle.select_sensors(sensors_arg)
        except Exception:
            gdx_handle.select_sensors()

        gdx_handle.start(GDX_POLL_MS)
        _gdx_device_count = n_devices
        _gdx_bg_running.set()
        _gdx_bg_thread = threading.Thread(target=_gdx_bg, daemon=True)
        _gdx_bg_thread.start()
        if n_devices >= 2:
            return True, "ok"
        return True, "ok (only 1 scale found — plug in the 2nd for robot integration)"
    except Exception as e:
        gdx_handle = None
        _gdx_device_count = 0
        return False, str(e)
    finally:
        builtins.input = orig_input


# ─────────────────────────────────────────────
#  LSL FORCE STREAM
# ─────────────────────────────────────────────
def _lsl_bg():
    """Background thread: pull scalar force samples from LSL inlet into _gdx_last_force."""
    while _lsl_bg_running.is_set():
        if _lsl_force_inlet is None:
            time.sleep(0.05)
            continue
        try:
            sample, _ = _lsl_force_inlet.pull_sample(timeout=0.1)
            if sample:
                _gdx_last_force[0] = float(sample[0])
        except Exception:
            time.sleep(0.01)


def get_lsl_streams():
    """Return a list of (name, type, channel_count) tuples for all visible LSL streams."""
    if not LSL_AVAILABLE:
        return []
    try:
        infos = resolve_streams(wait_time=1.0)
        return [(inf.name(), inf.type(), inf.channel_count()) for inf in infos]
    except Exception:
        return []


def connect_lsl_force(stream_name):
    """Connect to the named LSL stream and start the background reader.
    Returns (True, "") or (False, error_str).
    """
    global _lsl_force_inlet
    if not LSL_AVAILABLE:
        return False, "pylsl not installed (pip install pylsl)"
    try:
        infos = resolve_streams(wait_time=2.0)
        match = next((i for i in infos if i.name() == stream_name), None)
        if match is None:
            return False, f"Stream '{stream_name}' not found"
        _lsl_force_inlet = StreamInlet(match)
        _lsl_bg_running.set()
        threading.Thread(target=_lsl_bg, daemon=True).start()
        return True, "ok"
    except Exception as e:
        return False, str(e)


def disconnect_lsl_force():
    global _lsl_force_inlet
    _lsl_bg_running.clear()
    if _lsl_force_inlet is not None:
        try:
            _lsl_force_inlet.close_stream()
        except Exception:
            pass
        _lsl_force_inlet = None


# ─────────────────────────────────────────────
#  GDX TEARDOWN  (shared by every (re)connect attempt)
# ─────────────────────────────────────────────
def _teardown_all_gdx():
    """Fully tear down the GDX connection (a single handle that may cover 1
    or 2 physical scales) and reset the gdx library's shared class-level
    state. Call this before every open() so a reconnect never collides
    with an already-open hidapi USB path.

    Why this exists:
      - The gdx library stores devices/sensors at the CLASS level
        (gdx.devices, gdx.device_sensors, gdx.enabled_sensors, gdx.buffer),
        shared across every gdx_lib.gdx() instance. gdx.close() only empties
        gdx.devices — the other three lists are left stale, corrupting the
        next select_sensors()/read() call.
      - gdx.open_usb() unconditionally calls
        open_all_usb_devices_to_get_name(), which calls hid.device().open_path()
        on every discovered Go Direct USB device. If a previous handle still
        holds one of those paths open, this segfaults the whole Python
        process on macOS.
      - Disconnecting via .stop() alone (no .close()) leaves the HID path
        open, so a *later* connect attempt is the one that crashes.

    Ordering note: the default GDX read timeout is 5000ms. If we closed a
    handle while its background thread was still blocked inside read(),
    hidapi can crash from the handle being closed mid-read on another
    thread. To avoid that we clear the run-flag and join() the background
    thread FIRST, while the device is still streaming (so read() returns
    within one ~GDX_POLL_MS cycle) — only THEN do we call stop()/close().
    """
    global gdx_handle, _gdx_bg_thread, _gdx_device_count

    _gdx_bg_running.clear()

    if _gdx_bg_thread is not None and _gdx_bg_thread.is_alive():
        _gdx_bg_thread.join(timeout=1.0)
    _gdx_bg_thread = None

    if gdx_handle is not None:
        try:
            gdx_handle.stop()
        except Exception:
            pass
        try:
            gdx_handle.close()
        except Exception:
            pass
        gdx_handle = None

    _gdx_device_count = 0

    # gdx.close() above only clears gdx.devices — wipe the rest of the
    # class-level state by hand so the next open()/select_sensors() starts
    # from a clean slate.
    gdx_lib.gdx.devices = []
    gdx_lib.gdx.device_sensors = []
    gdx_lib.gdx.enabled_sensors = []
    gdx_lib.gdx.buffer = []


# ─────────────────────────────────────────────
#  MINDROVE HARDWARE STATE
# ─────────────────────────────────────────────
_mindrove_board      = None
_mindrove_running    = threading.Event()
_mindrove_lock       = threading.Lock()
_latest_mindrove_emg = [0.0] * 8   # 8 channels
MR_SAMPLING_RATE     = 500          # Hz (actual rate queried at connect time)
MR_WINDOW_SAMPLES    = 100          # samples per feature window (~200ms at 500Hz)
# 200ms gives ~5Hz FFT bin resolution (vs ~7.8Hz at the old 64-sample/128ms
# window) and steadier RMS/MAV/band-power estimates, at the cost of ~36ms
# more update latency (50%-overlap hop grows from 32 to 50 samples). Typical
# sEMG pattern-recognition windows are 150-250ms, so this stays in range.
DISPLAY_UPDATE_MS   = 80           # throttle heavy Tk widget updates
SNAPSHOT_UPDATE_MS  = 150          # throttle feature snapshot panel
VAL_PLOT_UPDATE_MS  = 200          # throttle validation plot redraws
VAL_SCATTER_UPDATE_MS = 400        # throttle live scatter redraws

# MindRove feature names — 8 channels × 12 features + 3 cross-channel + 8 relational channel-share features = 107 features
MR_N_CHANNELS = 8
MR_FEATURES_PER_CH = [
    "rms",          # Root Mean Square          — activation level
    "mav",          # Mean Absolute Value       — rectified amplitude
    "wl",           # Waveform Length           — signal complexity
    "zc",           # Zero Crossings            — frequency proxy
    "ssc",          # Slope Sign Changes        — frequency proxy (sensitive)
    "var",          # Variance                  — signal spread
    "mnf",          # Mean Frequency            — spectral centroid (drops w/ fatigue)
    "mdf",          # Median Frequency          — spectral median  (drops w/ fatigue)
    # Band powers below match the 20-200Hz bandpass applied in
    # preprocess_mindrove_window() — the signal has ~nothing left outside
    # that range, so bands must live inside it or they just measure filter
    # stopband noise/edge artifacts instead of muscle activity.
    "bp_low",       # Band Power 20–50 Hz   (low/dominant EMG energy)
    "bp_mid",       # Band Power 50–100 Hz
    "bp_high",      # Band Power 100–150 Hz
    "bp_vhigh",     # Band Power 150–200 Hz (shifts down with fatigue)
]
MR_FEATURE_NAMES = (
    [f"ch{ch}_{feat}"
     for ch in range(MR_N_CHANNELS)
     for feat in MR_FEATURES_PER_CH]
    + ["cross_mean_mav",   # mean MAV across all 8 channels (global activation)
       "cross_std_mav",    # std  MAV across all 8 channels (inter-channel imbalance)
       "cross_max_mav"]    # max  MAV across all 8 channels (dominant muscle)
    + [f"ch{ch}_mav_share" for ch in range(MR_N_CHANNELS)]  # relational contribution per channel
)
# Total: 8 channels × 12 features = 96  +  3 cross-channel + 8 MAV-share = 107 features
# Breakdown:
#   Time-domain  per channel  : RMS, MAV, WL, ZC, SSC, VAR        = 6 × 8 = 48
#   Frequency-domain per ch   : MNF, MDF                           = 2 × 8 = 16
#   Band powers per channel   : low,mid,high,vhigh (20-200Hz)      = 4 × 8 = 32
#   Cross-channel aggregates  : mean_mav, std_mav, max_mav         =      3
#   Relational features       : per-channel MAV share              =      8
#                                                               TOTAL = 107


def connect_mindrove():
    """Connect to the MindRove WiFi board. Returns (True, "") or (False, error_str)."""
    global _mindrove_board, MR_SAMPLING_RATE
    if not MINDROVE_AVAILABLE:
        return False, "mindrove package not installed (pip install mindrove)"
    try:
        params    = MindRoveInputParams()
        board     = BoardShim(BoardIds.MINDROVE_WIFI_BOARD, params)
        board.prepare_session()
        board.start_stream()
        MR_SAMPLING_RATE  = BoardShim.get_sampling_rate(BoardIds.MINDROVE_WIFI_BOARD)
        _mindrove_board   = board
        return True, "ok"
    except Exception as e:
        _mindrove_board = None
        return False, str(e)


def disconnect_mindrove():
    global _mindrove_board
    if _mindrove_board is not None:
        try:
            _mindrove_board.stop_stream()
            _mindrove_board.release_session()
        except Exception:
            pass
        _mindrove_board = None


def get_mindrove_emg():
    """Return latest 8-channel raw EMG sample (µV). Thread-safe."""
    with _mindrove_lock:
        return list(_latest_mindrove_emg)


def _safe_filtfilt(b, a, sig):
    min_len = max(len(a), len(b)) * 3
    if len(sig) <= min_len:
        return sig
    try:
        return filtfilt(b, a, sig)
    except Exception:
        return sig


def preprocess_mindrove_window(emg_window):
    """
    Preprocess raw MindRove EMG to improve invariance to session drift.
    Steps:
      1) subtract relaxed per-channel session baseline
      2) detrend each channel
      3) notch filter at 60 Hz
      4) bandpass filter 20–200 Hz
      5) normalize by per-channel MVC peak when available
    Returns np.array shape (T, 8)
    """
    global mr_session_baseline, mvc_reference
    W = np.array(emg_window, dtype=float)
    if W.ndim != 2 or W.shape[1] != MR_N_CHANNELS:
        return W

    # subtract learned relaxed baseline
    W = W - mr_session_baseline.reshape(1, -1)

    fs = float(MR_SAMPLING_RATE)
    try:
        b_notch, a_notch = iirnotch(60.0, 30.0, fs=fs)
    except TypeError:
        # older scipy fallback with normalized frequency
        w0 = 60.0 / (fs / 2.0)
        b_notch, a_notch = iirnotch(w0, 30.0)

    b_bp, a_bp = butter(4, [20.0 / (fs / 2.0), 200.0 / (fs / 2.0)], btype="band")

    proc = np.zeros_like(W, dtype=float)
    for ch in range(MR_N_CHANNELS):
        sig = detrend(W[:, ch], type="constant")
        sig = _safe_filtfilt(b_notch, a_notch, sig)
        sig = _safe_filtfilt(b_bp, a_bp, sig)

        if mvc_reference is not None and len(mvc_reference) >= MR_N_CHANNELS:
            mvc = float(mvc_reference[ch])
            if mvc > 1e-9:
                sig = sig / mvc
        proc[:, ch] = sig

    return proc



def _mindrove_reader_thread(stop_event, data_queue):
    """Background thread: polls MindRove board and pushes into data_queue."""
    emg_channels = BoardShim.get_exg_channels(BoardIds.MINDROVE_WIFI_BOARD)
    poll_samples = max(8, MR_SAMPLING_RATE // 20)   # poll at ~20 Hz
    while not stop_event.is_set():
        if _mindrove_board is None:
            time.sleep(0.05)
            continue
        try:
            if _mindrove_board.get_board_data_count() >= poll_samples:
                data = _mindrove_board.get_board_data(poll_samples)
                emg_data = data[emg_channels]   # shape (8, poll_samples)
                # Push each sample individually into the pipeline queue
                for i in range(emg_data.shape[1]):
                    sample = emg_data[:, i].tolist()
                    with _mindrove_lock:
                        _latest_mindrove_emg[:] = sample
                    try:
                        data_queue.put_nowait({
                            "emg":   sample,
                            "force": _gdx_last_force[0],
                        })
                    except queue.Full:
                        pass
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"[MindRove reader] {e}")
            time.sleep(0.05)


# ─────────────────────────────────────────────
#  MINDROVE FEATURE EXTRACTION
#  Input: np.array of shape (window_samples, 8)
#  Output: np.array of shape (40,)
# ─────────────────────────────────────────────
def extract_features_mindrove(emg_window):
    """
    Extract 12 features × 8 channels + 3 cross-channel + 8 relational channel-share features = 107 features from a
    raw MindRove EMG window.

    Features per channel:
        Time-domain  : RMS, MAV, WL, ZC, SSC, VAR
        Frequency    : MNF (mean freq), MDF (median freq)
        Band powers  : low (20-50Hz), mid (50-100Hz),
                       high (100-150Hz), vhigh (150-200Hz)
                       — kept inside the 20-200Hz bandpass so they measure
                       signal, not stopband attenuation.
    Cross-channel   : mean_MAV, std_MAV, max_MAV  (global activation summary)
    Relational      : per-channel MAV share across all 8 channels

    emg_window: list of raw sample vectors (each len-8), or np.array (T, 8)
    Returns flat np.array of length 107, or None if window too small.
    """
    if len(emg_window) < 8:   # need at least 8 samples for FFT to be meaningful
        return None

    W = preprocess_mindrove_window(emg_window)   # shape (T, 8)
    if W.ndim != 2 or W.shape[1] != MR_N_CHANNELS:
        return None

    T = W.shape[0]
    threshold = 0.0   # µV threshold for ZC / SSC (keeps noise from inflating counts)

    # Pre-compute FFT quantities shared across channels
    # Use real FFT: output length = T//2 + 1
    freqs = np.fft.rfftfreq(T, d=1.0 / MR_SAMPLING_RATE)  # frequency bins in Hz

    # Band masks (computed once, reused for every channel).
    # Kept inside the 20-200Hz bandpass applied upstream — bands outside
    # that range would only measure filter stopband attenuation, not signal.
    _bands = [
        (20.0,  50.0),   # low  (dominant EMG energy)
        (50.0,  100.0),  # mid
        (100.0, 150.0),  # high
        (150.0, 200.0),  # vhigh (shifts down with fatigue)
    ]
    band_masks = [
        (freqs >= lo) & (freqs < hi)
        for lo, hi in _bands
    ]

    feats = []
    mav_per_ch = []   # collect MAV for cross-channel features

    for ch in range(MR_N_CHANNELS):
        sig = W[:, ch]

        # ── Time-domain ──────────────────────────────────────────────────────
        rms = float(np.sqrt(np.mean(sig ** 2)))
        mav = float(np.mean(np.abs(sig)))
        var = float(np.var(sig))
        wl  = float(np.sum(np.abs(np.diff(sig))))

        # Zero crossings (threshold filters out noise-floor flicker)
        zc = float(np.sum(
            (sig[:-1] * sig[1:] < 0) &
            (np.abs(sig[:-1] - sig[1:]) >= threshold)
        ))

        # Slope sign changes
        d   = np.diff(sig)
        ssc = float(np.sum(
            (d[:-1] * d[1:] < 0) &
            ((np.abs(d[:-1]) >= threshold) | (np.abs(d[1:]) >= threshold))
        ))

        # ── Frequency-domain ─────────────────────────────────────────────────
        fft_mag  = np.abs(np.fft.rfft(sig))          # magnitude spectrum
        psd      = fft_mag ** 2                       # power spectrum (unnormalised)
        total_pwr = psd.sum()

        if total_pwr > 1e-12:
            # MNF — power-weighted mean frequency
            mnf = float(np.sum(freqs * psd) / total_pwr)
            # MDF — frequency below which 50% of power sits
            cumsum = np.cumsum(psd)
            mdf_idx = np.searchsorted(cumsum, total_pwr * 0.5)
            mdf = float(freqs[min(mdf_idx, len(freqs) - 1)])
            # Band powers (absolute, not ratio — scaler will normalise later)
            bp = [float(psd[mask].sum()) for mask in band_masks]
        else:
            mnf = 0.0
            mdf = 0.0
            bp  = [0.0] * len(_bands)

        feats.extend([rms, mav, wl, zc, ssc, var, mnf, mdf] + bp)
        mav_per_ch.append(mav)

    # ── Cross-channel aggregates ──────────────────────────────────────────────
    mav_arr = np.array(mav_per_ch)
    feats.append(float(mav_arr.mean()))   # global activation level
    feats.append(float(mav_arr.std()))    # inter-channel imbalance
    feats.append(float(mav_arr.max()))    # dominant muscle contribution
    total_mav = float(mav_arr.sum()) + 1e-9
    feats.extend((mav_arr / total_mav).tolist())  # relational contribution per channel

    return np.array(feats, dtype=float)


# ─────────────────────────────────────────────
#  MVC NORMALIZATION
# ─────────────────────────────────────────────
def normalize_emg(raw):
    """Normalize raw EMG vector to [0,1] relative to MVC. Returns raw if not calibrated."""
    if mvc_reference is None:
        return list(raw)
    normed = []
    for r, m in zip(raw, mvc_reference):
        normed.append(min(1.0, r / m) if m > 1e-9 else 0.0)
    return normed


def compute_signal_lag_seconds(y_true, y_pred, timestamps=None, max_lag_seconds=0.30):
    """Estimate lag using bounded cross-correlation.
    Positive lag means prediction lags behind ground truth.
    The search window is limited to a physically realistic delay range so
    periodic validation waveforms do not snap to the wrong cycle.
    Returns lag in seconds, or None when it cannot be estimated robustly.
    """
    yt = np.asarray(y_true, dtype=float)
    yp = np.asarray(y_pred, dtype=float)
    if len(yt) < 5 or len(yp) < 5 or len(yt) != len(yp):
        return None

    dt = None
    if timestamps is not None and len(timestamps) == len(yt) and len(timestamps) >= 2:
        ts = np.asarray(timestamps, dtype=float)
        diffs = np.diff(ts)
        diffs = diffs[np.isfinite(diffs) & (diffs > 0)]
        if len(diffs):
            dt = float(np.median(diffs))

    if dt is None or not np.isfinite(dt) or dt <= 0:
        # Validation samples are usually spaced close to the feature-window step.
        # Fall back to a conservative default instead of allowing effectively
        # unbounded lag estimates.
        dt = 0.05

    # Remove DC offset and normalize so the score is not biased by amplitude.
    yt = yt - np.mean(yt)
    yp = yp - np.mean(yp)
    yt_std = np.std(yt)
    yp_std = np.std(yp)
    if yt_std < 1e-9 or yp_std < 1e-9:
        return None
    yt = yt / yt_std
    yp = yp / yp_std

    n = len(yt)
    max_lag_samples = int(round(max_lag_seconds / dt))
    max_lag_samples = max(1, min(n - 1, max_lag_samples))

    corr = np.correlate(yt, yp, mode="full")
    lags = np.arange(-n + 1, n)
    mask = (lags >= -max_lag_samples) & (lags <= max_lag_samples)
    if not np.any(mask):
        return None

    corr = corr[mask]
    lags = lags[mask]

    # Prefer the strongest correlation within the realistic window.
    best_idx = int(np.argmax(corr))
    best_lag_samples = int(lags[best_idx])
    return float(best_lag_samples * dt)


# ─────────────────────────────────────────────
#  FEATURE EXTRACTION
#  Input: list of N raw_emg vectors (each = act + 8 bins per sensor)
#  Output: 1-D feature vector matching FEATURE_NAMES
# ─────────────────────────────────────────────
def extract_features(emg_window):
    """
    Routes to the correct feature extractor based on ACTIVE_DEVICE.
    emg_window: list of raw EMG vectors over a time window.
    Returns flat numpy array of features, or None if window too small.
    """
    if ACTIVE_DEVICE == "mindrove":
        return extract_features_mindrove(emg_window)
    # uMyo path (original)
    if len(emg_window) < 3:
        return None

    W = np.array(emg_window, dtype=float)  # shape [T, NUM_SENSORS*(1+BINS)]
    stride = BINS_PER_SENSOR + 1

    feats = []

    # Per-sensor raw features
    act_channels = []
    for s in range(NUM_SENSORS):
        act   = W[:, s * stride]               # scalar activity over time
        bins  = W[:, s*stride+1 : s*stride+1+BINS_PER_SENSOR]  # [T, 8]

        act_mean  = float(act.mean())
        bins_mean = bins.mean(axis=0)           # [8] mean per bin
        act_std   = float(act.std())
        t_idx     = np.arange(len(act), dtype=float)
        slope     = float(np.polyfit(t_idx, act, 1)[0]) if act.std() > 1e-9 else 0.0
        peak_ratio= float(act.max() / (act_mean + 1e-9))

        feats.append(act_mean)                  # s{s}_act
        act_channels.append(act)

    # Per-sensor bin means (8 per sensor)
    for s in range(NUM_SENSORS):
        bins = W[:, s*stride+1 : s*stride+1+BINS_PER_SENSOR]
        feats.extend(bins.mean(axis=0).tolist())  # s{s}_b0..b7

    # Per-sensor temporal stats
    for s in range(NUM_SENSORS):
        act = W[:, s * stride]
        feats.append(float(act.mean()))         # s{s}_mean
    for s in range(NUM_SENSORS):
        act = W[:, s * stride]
        feats.append(float(act.std()))          # s{s}_std
    for s in range(NUM_SENSORS):
        act = W[:, s * stride]
        t   = np.arange(len(act), dtype=float)
        sl  = float(np.polyfit(t, act, 1)[0]) if act.std() > 1e-9 else 0.0
        feats.append(sl)                        # s{s}_slope
    for s in range(NUM_SENSORS):
        act  = W[:, s * stride]
        mean = act.mean()
        feats.append(float(act.max() / (mean + 1e-9)))  # s{s}_peak_ratio

    # Cross-channel features (gracefully handles 1, 2, or 3 sensors)
    a0 = W[:, 0 * stride]
    a1 = W[:, min(1, NUM_SENSORS - 1) * stride]  # falls back to a0 if only 1 sensor
    a2 = W[:, min(2, NUM_SENSORS - 1) * stride]  # falls back to a1/a0 if <3 sensors
    diff = a0 - a1
    feats.append(float(diff.mean()))              # diff_mean  (s0-s1)
    feats.append(float(diff.std()))               # diff_std
    feats.append(float((a0 / (a1 + 1e-9)).mean()))# ratio_mean (s0/s1)
    feats.append(float((a0 + a1).mean()))         # total_act  (s0+s1)
    # Extra cross-channel features when 3 sensors present (always appended
    # to keep feature vector length consistent — zero when NUM_SENSORS < 3)
    diff02 = a0 - a2
    feats.append(float(diff02.mean()))            # diff02_mean (s0-s2)
    feats.append(float((a1 - a2).mean()))         # diff12_mean (s1-s2)
    feats.append(float((a0 + a1 + a2).mean()))    # total_act3

    return np.array(feats, dtype=float)


# ─────────────────────────────────────────────
#  PIPELINE STATE
# ─────────────────────────────────────────────
class PipelineState:
    def __init__(self):
        self.connected_emg  = False
        self.connected_gdx  = False
        self.recording      = False
        self.inferring      = False
        self.model_trained  = False

        # Live EMG history for display
        self.act0_hist = collections.deque(maxlen=HIST_LEN)
        self.act1_hist = collections.deque(maxlen=HIST_LEN)
        self.act2_hist = collections.deque(maxlen=HIST_LEN)
        self.force_hist = collections.deque(maxlen=HIST_LEN)

        # Window buffer for feature extraction
        self.win_buf       = []    # list of raw emg vectors
        self.win_force_buf = []    # matching force values
        self.win_label     = None  # int % MVC or None

        # Dataset: list of {"features": np.array, "force": float, "level": int}
        self.dataset   = []
        self.sample_count = 0
        self.level_window_counts = {lvl: 0 for lvl in FORCE_LEVELS}

        # Model
        self.model_name = "XGBoost"
        self.scaler     = None
        self.model      = None

        # Validation live buffers (real-time tab 3)
        self.val_pred    = collections.deque(maxlen=300)
        self.val_true    = collections.deque(maxlen=300)
        # Smoothing
        self.ema_enabled    = False
        self.median_enabled = False
        self.ema_alpha      = 0.2      # 0=max smooth, 1=no smooth
        self.median_window  = 7        # must be odd
        self._ema_last      = None     # running EMA state

        # Data queue: EMG thread → main thread
        self.data_queue = queue.Queue(maxsize=400)

        # Protocol
        self.protocol_seq  = []
        self.protocol_idx  = 0
        self.protocol_run_id = 0   # increments each protocol run so window "group" ids stay unique across sessions
        self.protocol_running = False
        self.current_level = 0
        self.live_force_pct = 0.0   # real-time force as % MVC (updated by data loop)
        self.current_phase_type = "idle"   # hold, rest, ramp_up, ramp_down, idle

        # Metrics (updated after each validation window)
        self.live_r2     = 0.0
        self.live_rmse   = 0.0
        self.live_pearson = 0.0
        self.live_lag_s  = None
        self.live_filt_r2 = None
        self.live_filt_rmse = None
        self.live_filt_pearson = None
        self.live_filt_lag_s = None
        self.cv_r2         = None
        self.cv_rmse       = None
        self.cv_pearson    = None
        self.cv_model_name = None

        self.log_lines = []

        # Deploy tab
        self.deploy_recording  = False
        self.deploy_duration   = 20.0   # seconds
        self.deploy_raw_pred   = []     # raw predictions during timed run
        self.deploy_filt_pred  = []     # filtered predictions
        self.deploy_true       = []     # ground truth force
        self.deploy_timestamps = []     # time in seconds from start

        # Performance tab
        self.performance_recording  = False
        self.performance_duration   = 20.0
        self.performance_raw_pred   = []
        self.performance_filt_pred  = []
        self.performance_true       = []
        self.performance_timestamps = []
        self.performance_sessions   = []   # list of per-session summary dicts
        self.performance_detail_rows = []   # in-memory per-sample rows for all recorded performance sessions
        self.performance_session_traces = []   # index-aligned with performance_sessions: raw ts/yt/yp arrays for PNG export

        # Robot integration performance tab (pred → robot grip metrics)
        self.robot_recording    = False
        self.robot_duration     = 20.0
        self.robot_session_pred = []   # prediction % MVC during recorded session
        self.robot_session_sc1  = []   # human grip % MVC during recorded session
        self.robot_session_sc2  = []   # robot grip % RFO during recorded session
        self.robot_session_ts   = []   # elapsed-time timestamps (s from stream start)
        self.robot_sessions     = []   # per-session summary dicts
        self.robot_detail_rows  = []   # in-memory per-sample rows for all recorded robot sessions
        self.robot_session_traces = []  # index-aligned with robot_sessions: raw ts/pred/sc1/sc2 arrays for PNG export

STATE = PipelineState()


# ─────────────────────────────────────────────
#  SIMULATION  (for testing without hardware)
# ─────────────────────────────────────────────
class EMGSimulator:
    def __init__(self):
        self.t = 0.0
        self.env = [0.05, 0.05]

    def step(self, force_pct):
        self.t += 0.05
        target = force_pct / 100.0
        for i in range(2):
            self.env[i] += (target * (0.9 + i*0.15) - self.env[i]) * 0.12
        result = []
        for s in range(NUM_SENSORS):
            act = max(0, self.env[s] + random.gauss(0, 0.02))
            bins = [max(0, self.env[s] * (0.7 - b*0.06) + random.gauss(0, 0.01))
                    for b in range(BINS_PER_SENSOR)]
            result.append(act)
            result.extend(bins)
        return result

SIM = EMGSimulator()


# ═══════════════════════════════════════════════════════════
#  MAIN APPLICATION
# ═══════════════════════════════════════════════════════════
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("EMG → Force  Pipeline  v2.0  |  uMyo + MindRove")
        self.configure(bg=BG)
        self.geometry("1440x880")
        self.minsize(1200, 720)

        self._stop_event     = threading.Event()
        self._sim_thread     = None
        self._protocol_timer = None
        self._elapsed_id     = None
        self._elapsed_start  = 0
        self._sim_mode       = False
        self._win_size_samples = 10
        self._last_live_ui_ts = 0.0
        self._last_force_ui_ts = 0.0
        self._last_snapshot_ts = 0.0
        self._last_val_plot_ts = 0.0
        self._last_val_scatter_ts = 0.0
        # Ramp overlay state
        self._ramp_t0             = None
        self._ramp_total_duration = 1.0

        # Shared pop-out window state (second-monitor effort-graph view)
        self._popout_win = None
        self._popout_key = None
        self._popout_last_monitor = None
        self._popout_monitor_check_id = None

        self._build_ui()
        self._start_consumer()
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ════════════════════════════════════════
    #  UI CONSTRUCTION
    # ════════════════════════════════════════
    def _build_ui(self):
        self._build_header()
        pane = tk.PanedWindow(self, orient=tk.HORIZONTAL, bg=BG,
                               sashwidth=4, sashrelief=tk.FLAT)
        pane.pack(fill=tk.BOTH, expand=True)
        left   = self._build_left(pane)
        center = self._build_center(pane)
        right  = self._build_right(pane)
        pane.add(left,   minsize=240, width=270, stretch="never")
        pane.add(center, minsize=600, width=860, stretch="always")
        pane.add(right,  minsize=180, width=220, stretch="never")

    def _build_header(self):
        hdr = tk.Frame(self, bg=PANEL, height=50)
        hdr.pack(fill=tk.X); hdr.pack_propagate(False)

        lf = tk.Frame(hdr, bg=PANEL)
        lf.pack(side=tk.LEFT, padx=16, pady=8)
        tk.Label(lf, text="EMG → FORCE",
                 font=("Arial",16,"bold"), fg=ACCENT, bg=PANEL).pack(side=tk.LEFT)
        tk.Label(lf, text="  PROPORTIONAL CONTROL PIPELINE v2.0  |  uMyo + MindRove",
                 font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack(side=tk.LEFT)

        rf = tk.Frame(hdr, bg=PANEL)
        rf.pack(side=tk.RIGHT, padx=16)
        self._conn_dot   = self._dot(rf, MUTED)
        self._conn_lbl   = self._stat_lbl(rf, "EMG: DISC")
        self._gdx_dot    = self._dot(rf, MUTED)
        self._gdx_lbl    = self._stat_lbl(rf, "GDX: DISC")
        self._rec_dot    = self._dot(rf, MUTED)
        self._rec_lbl    = self._stat_lbl(rf, "IDLE")
        self._n_lbl      = self._stat_lbl(rf, "0 WINDOWS")

    def _dot(self, p, c):
        cv = tk.Canvas(p, width=8, height=8, bg=PANEL, highlightthickness=0)
        cv.create_oval(1,1,7,7, fill=c, outline="")
        cv.pack(side=tk.LEFT, padx=(8,2), pady=14)
        return cv

    def _stat_lbl(self, p, t):
        l = tk.Label(p, text=t, font=FONT_MONO_SM, fg=TEXT, bg=PANEL,
                     padx=8, pady=2)
        l.pack(side=tk.LEFT, padx=2)
        return l

    # ── LEFT PANEL ───────────────────────────
    def _build_left(self, parent):
        frame = tk.Frame(parent, bg=PANEL, width=270)
        frame.pack_propagate(False)
        canvas = tk.Canvas(frame, bg=PANEL, highlightthickness=0)
        sb = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=canvas.yview)
        sf = tk.Frame(canvas, bg=PANEL)

        # Keep scroll frame width in sync with canvas width so fill=X works
        def _on_canvas_resize(e):
            canvas.itemconfig(_win_id, width=e.width)
        canvas.bind("<Configure>", _on_canvas_resize)

        # Update scrollregion whenever content changes size
        sf.bind("<Configure>", lambda e: canvas.configure(
            scrollregion=canvas.bbox("all")))

        _win_id = canvas.create_window((0, 0), window=sf, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

        # Mousewheel scrolling
        def _on_mousewheel(e):
            canvas.yview_scroll(int(-1 * (e.delta / 120)), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)

        self._build_conn_section(sf)
        self._build_session_section(sf)
        self._build_protocol_section(sf)
        self._build_data_section(sf)
        return frame

    def _section(self, p, title):
        outer = tk.Frame(p, bg=PANEL)
        outer.pack(fill=tk.X, padx=8, pady=(8,0))
        hdr = tk.Frame(outer, bg=PANEL)
        hdr.pack(fill=tk.X)
        tk.Label(hdr, text=title, font=("Arial",9,"bold"),
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT)
        tk.Frame(hdr, bg=BORDER, height=1).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=(6,0), pady=6)
        inner = tk.Frame(outer, bg=PANEL)
        inner.pack(fill=tk.X, pady=(4,0))
        return inner

    def _entry(self, p, label, default, width=22):
        f = tk.Frame(p, bg=PANEL); f.pack(fill=tk.X, pady=2)
        tk.Label(f, text=label, font=FONT_MONO_SM, fg=MUTED,
                 bg=PANEL).pack(anchor=tk.W)
        var = tk.StringVar(value=str(default))
        tk.Entry(f, textvariable=var, width=width,
                 bg=PANEL2, fg=TEXT, insertbackground=TEXT,
                 relief=tk.FLAT, font=FONT_MONO_SM,
                 highlightbackground=BORDER,
                 highlightcolor=ACCENT, highlightthickness=1).pack(fill=tk.X)
        return var

    def _build_conn_section(self, p):
        s = self._section(p, "CONNECTION")

        # ── DEVICE SELECTOR ──────────────────────────────────────────────────
        tk.Label(s, text="EMG DEVICE", font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack(anchor=tk.W, pady=(0,2))
        dev_row = tk.Frame(s, bg=PANEL); dev_row.pack(fill=tk.X, pady=(0,6))
        self._dev_var = tk.StringVar(value="mindrove")
        self._dev_umyo_btn = tk.Button(
            dev_row, text="uMyo", font=FONT_MONO_SM,
            bg=PANEL2, fg=MUTED, relief=tk.FLAT,
            command=lambda: self._select_device("umyo"))
        self._dev_umyo_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,2))
        self._dev_mr_btn = tk.Button(
            dev_row, text="MindRove", font=FONT_MONO_SM,
            bg=PURPLE, fg=BG, relief=tk.FLAT,
            command=lambda: self._select_device("mindrove"))
        self._dev_mr_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Feature count info label
        self._feat_info_lbl = tk.Label(
            s, text="MindRove: 8 live EMG channels + full ML pipeline",
            font=FONT_MONO_SM, fg=MUTED, bg=PANEL, wraplength=230)
        self._feat_info_lbl.pack(anchor=tk.W, pady=(0,4))

        # ─────────────────────────────────────────────────────────────────────
        mf = tk.Frame(s, bg=PANEL); mf.pack(fill=tk.X, pady=(0,6))
        self._sim_btn = tk.Button(mf, text="◈ SIMULATE", font=FONT_MONO_SM,
                                   bg=PANEL2, fg=MUTED, relief=tk.FLAT,
                                   command=lambda: self._set_sim(True))
        self._sim_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self._hw_btn  = tk.Button(mf, text="⬡ HARDWARE", font=FONT_MONO_SM,
                                   bg=ACCENT, fg=BG, relief=tk.FLAT,
                                   command=lambda: self._set_sim(False))
        self._hw_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)

        # EMG port
        tk.Label(s, text="EMG PORT (ESP32-S3)", font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack(anchor=tk.W)
        pf = tk.Frame(s, bg=PANEL); pf.pack(fill=tk.X)
        self._port_var = tk.StringVar()
        self._port_cb  = ttk.Combobox(pf, textvariable=self._port_var,
                                       font=FONT_MONO_SM, width=14)
        self._port_cb.pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Button(pf, text="↺", font=FONT_MONO_SM,
                  bg=PANEL2, fg=ACCENT, relief=tk.FLAT,
                  command=self._refresh_ports).pack(side=tk.LEFT, padx=(4,0))
        self._refresh_ports()

        self._emg_btn = tk.Button(s, text="⬡  CONNECT EMG",
                                   font=("Arial",10,"bold"),
                                   bg=PANEL2, fg=ACCENT, relief=tk.FLAT,
                                   highlightbackground=ACCENT, highlightthickness=1,
                                   pady=6, command=self._toggle_emg)
        self._emg_btn.pack(fill=tk.X, pady=(6,4))

        # MindRove connect button (shown/hidden based on device selection)
        self._mr_btn = tk.Button(s, text="⬡  CONNECT MINDROVE",
                                  font=("Arial",10,"bold"),
                                  bg=PANEL2, fg=PURPLE, relief=tk.FLAT,
                                  highlightbackground=PURPLE, highlightthickness=1,
                                  pady=6, command=self._toggle_mindrove)
        # shown by default (MindRove is default)
        self._mr_btn.pack(fill=tk.X, pady=(6,4))

        # ── FORCE SOURCE selector ─────────────────────────────────────────────
        tk.Label(s, text="FORCE SOURCE", font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack(anchor=tk.W, pady=(6,2))
        self._force_src_var = tk.StringVar(value="GDX (USB)")
        self._force_src_cb = ttk.Combobox(
            s, textvariable=self._force_src_var,
            values=["GDX (USB)", "LSL Stream"],
            font=FONT_MONO_SM, state="readonly")
        self._force_src_cb.pack(fill=tk.X, pady=(0,4))
        self._force_src_cb.bind("<<ComboboxSelected>>", self._on_force_src_change)

        self._gdx_btn = tk.Button(s, text="⬡  CONNECT GDX",
                                   font=("Arial",10,"bold"),
                                   bg=PANEL2, fg=GREEN, relief=tk.FLAT,
                                   highlightbackground=GREEN, highlightthickness=1,
                                   pady=6, command=self._connect_force)
        self._gdx_btn.pack(fill=tk.X, pady=(0,2))

        # LSL stream picker row — hidden until "LSL Stream" is chosen
        self._lsl_pick_frame = tk.Frame(s, bg=PANEL)
        tk.Label(self._lsl_pick_frame, text="LSL STREAM", font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack(anchor=tk.W)
        lsl_row = tk.Frame(self._lsl_pick_frame, bg=PANEL)
        lsl_row.pack(fill=tk.X)
        self._lsl_stream_var = tk.StringVar()
        self._lsl_stream_cb = ttk.Combobox(
            lsl_row, textvariable=self._lsl_stream_var,
            font=FONT_MONO_SM, state="readonly", width=14)
        self._lsl_stream_cb.pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Button(lsl_row, text="↺", font=FONT_MONO_SM,
                  bg=PANEL2, fg=ACCENT, relief=tk.FLAT,
                  command=self._refresh_lsl_streams).pack(side=tk.LEFT, padx=(4,0))

        # Start in MindRove mode on launch.
        self.after(50, lambda: self._select_device("mindrove"))

        self._conn_status = tk.Label(s, text="", font=FONT_MONO_SM,
                                      fg=MUTED, bg=PANEL, wraplength=230)
        self._conn_status.pack(anchor=tk.W, pady=(4,0))

    def _build_session_section(self, p):
        s = self._section(p, "SESSION")
        self._subject_var = self._entry(s, "SUBJECT ID", "SUB_001")
        row = tk.Frame(s, bg=PANEL); row.pack(fill=tk.X)
        lf = tk.Frame(row, bg=PANEL); lf.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,4))
        rf = tk.Frame(row, bg=PANEL); rf.pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Label(lf, text="HOLD (s)", font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack(anchor=tk.W)
        self._hold_var = tk.StringVar(value=str(HOLD_SEC))
        tk.Entry(lf, textvariable=self._hold_var, width=6,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER,
                 highlightthickness=1).pack(fill=tk.X)
        tk.Label(rf, text="REST (s)", font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack(anchor=tk.W)
        self._rest_var = tk.StringVar(value=str(REST_SEC))
        tk.Entry(rf, textvariable=self._rest_var, width=6,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER,
                 highlightthickness=1).pack(fill=tk.X)
        row2 = tk.Frame(s, bg=PANEL); row2.pack(fill=tk.X, pady=(4,0))
        lf2 = tk.Frame(row2, bg=PANEL); lf2.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,4))
        rf2 = tk.Frame(row2, bg=PANEL); rf2.pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Label(lf2, text="REPS", font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack(anchor=tk.W)
        self._reps_var = tk.StringVar(value=str(NUM_REPS))
        tk.Entry(lf2, textvariable=self._reps_var, width=6,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER,
                 highlightthickness=1).pack(fill=tk.X)
        tk.Label(rf2, text="WINDOW (samples)", font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack(anchor=tk.W)
        self._win_var = tk.StringVar(value="10")
        tk.Entry(rf2, textvariable=self._win_var, width=6,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER,
                 highlightthickness=1).pack(fill=tk.X)

    def _build_protocol_section(self, p):
        s = self._section(p, "FORCE LEVELS")
        self._level_counts = {}
        colors = [MUTED, ACCENT2, YELLOW, GREEN, ACCENT]
        for i, lvl in enumerate(FORCE_LEVELS):
            f = tk.Frame(s, bg=PANEL2, pady=3, padx=6,
                         highlightbackground=BORDER, highlightthickness=1)
            f.pack(fill=tk.X, pady=2)
            self._dot_widget(f, colors[i]).pack(side=tk.LEFT, padx=(0,6))
            tk.Label(f, text=f"{lvl}% MVC", font=FONT_MONO, fg=TEXT, bg=PANEL2).pack(side=tk.LEFT)
            var = tk.StringVar(value="0")
            tk.Label(f, textvariable=var, font=FONT_MONO, fg=MUTED, bg=PANEL2).pack(side=tk.RIGHT)
            self._level_counts[lvl] = var

    def _dot_widget(self, parent, color):
        cv = tk.Canvas(parent, width=10, height=10, bg=PANEL2, highlightthickness=0)
        cv.create_oval(1,1,9,9, fill=color, outline="")
        return cv

    def _build_data_section(self, p):
        s = self._section(p, "DATA")
        tk.Button(s, text="⟳  CLEAR DATASET", font=FONT_MONO_SM,
                  bg=PANEL2, fg=YELLOW, relief=tk.FLAT,
                  pady=4, command=self._clear_dataset).pack(fill=tk.X, pady=2)
        row = tk.Frame(s, bg=PANEL); row.pack(fill=tk.X)
        tk.Button(row, text="↓ CSV", font=FONT_MONO_SM,
                  bg=PANEL2, fg=GREEN, relief=tk.FLAT,
                  pady=4, command=self._export_csv).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,2))
        tk.Button(row, text="▲ LOAD", font=FONT_MONO_SM,
                  bg=PANEL2, fg=ACCENT, relief=tk.FLAT,
                  pady=4, command=self._load_csv).pack(side=tk.LEFT, fill=tk.X, expand=True)
        tk.Button(s, text="💾  SAVE MODEL", font=FONT_MONO_SM,
                  bg=PANEL2, fg=PURPLE, relief=tk.FLAT,
                  pady=4, command=self._save_model).pack(fill=tk.X, pady=(2,0))
        tk.Button(s, text="📂  LOAD MODEL", font=FONT_MONO_SM,
                  bg=PANEL2, fg=MUTED, relief=tk.FLAT,
                  pady=4, command=self._load_model).pack(fill=tk.X, pady=(2,0))

    # ── CENTER PANEL — 3 tabs ────────────────
    def _build_center(self, parent):
        frame = tk.Frame(parent, bg=BG)
        tab_frame = tk.Frame(frame, bg=PANEL)
        tab_frame.pack(fill=tk.X)
        self._tabs = {}
        for key, label in [("collect","01  COLLECT"),
                            ("train",  "02  TRAIN"),
                            ("validate","03  VALIDATE"),
                            ("deploy",  "04  DEPLOY"),
                            ("performance","05  PERFORMANCE"),
                            ("robot",   "06  ROBOT INT"),
                            ("exo",     "07  EXOSKELETON")]:
            btn = tk.Button(tab_frame, text=label,
                            font=("Arial",10,"bold"),
                            bg=PANEL2 if key=="collect" else PANEL,
                            fg=ACCENT  if key=="collect" else MUTED,
                            relief=tk.FLAT, padx=16, pady=8,
                            command=lambda k=key: self._switch_tab(k))
            btn.pack(side=tk.LEFT)
            self._tabs[key] = btn

        content = tk.Frame(frame, bg=BG)
        content.pack(fill=tk.BOTH, expand=True)
        self._tab_frames = {}
        for key in ["collect","train","validate","deploy","performance","exo","robot"]:
            f = tk.Frame(content, bg=BG)
            f.place(relwidth=1, relheight=1)
            self._tab_frames[key] = f

        self._build_collect_tab(self._tab_frames["collect"])
        self._build_train_tab(self._tab_frames["train"])
        self._build_validate_tab(self._tab_frames["validate"])
        self._build_deploy_tab(self._tab_frames["deploy"])
        self._build_performance_tab(self._tab_frames["performance"])
        self._build_robot_tab(self._tab_frames["robot"])
        self._build_exo_tab(self._tab_frames["exo"])
        self._switch_tab("collect")
        return frame

    def _switch_tab(self, key):
        for k, f in self._tab_frames.items():
            f.lower()
            self._tabs[k].configure(bg=PANEL, fg=MUTED)
        self._tab_frames[key].lift()
        self._tabs[key].configure(bg=PANEL2, fg=ACCENT)

    # ════════════════════════════════════════
    #  EFFORT-GRAPH POP-OUT  (second-monitor view)
    # ════════════════════════════════════════
    # One shared Toplevel window that can be re-targeted at any of the four
    # live effort graphs (collect / validate / performance / robot). Opening
    # it on a different graph swaps its content in place rather than
    # spawning a new window. It's a plain resizable window — the
    # experimenter drags it to the second monitor and resizes/maximizes it
    # there; all control stays on the main GUI.
    _POPOUT_TITLES = {
        "collect":     "Effort Guide  —  Collect",
        "validate":    "Effort Guide  —  Validate",
        "performance": "Effort Guide  —  Performance",
        "robot":       "Effort Guide  —  Robot",
    }
    _POPOUT_AX_ATTR = {
        "collect":     "_overlay_ax",
        "validate":    "_val_ax",
        "performance": "_perf_ax",
        "robot":       "_robot_ax",
    }

    def _popout_source_ax(self, key):
        return getattr(self, self._POPOUT_AX_ATTR.get(key, ""), None)

    def _add_popout_button(self, parent, key):
        """Small header bar with a button that opens/targets the shared
        pop-out window on this graph, placed directly above it. Returns the
        bar frame so callers can pack additional controls (e.g. visibility
        toggles) into the same strip."""
        bar = tk.Frame(parent, bg=PANEL)
        bar.pack(fill=tk.X, padx=0, pady=(4, 0))
        tk.Button(bar, text="⛶  POP-OUT VIEW", font=("Arial", 9, "bold"),
                  bg=PANEL2, fg=ACCENT, relief=tk.FLAT, padx=8, pady=3,
                  command=lambda k=key: self._popout_open(k)
                  ).pack(side=tk.RIGHT, padx=4)
        return bar

    def _make_visibility_toggle(self, parent, label, color, lines, redraw_fn,
                                 popout_key, initial=False):
        """Small ON/OFF button that shows/hides a group of line artists
        together (e.g. a data line plus its leading-edge dot). Used so a
        participant can be shown just the target guide + their own live
        effort during a trial, with prediction/robot-output readouts
        revealed afterward for review without losing the recorded data —
        the toggle only ever changes visibility, never the underlying data.
        """
        for ln in lines:
            ln.set_visible(initial)
        state = {"on": initial}

        def toggle():
            state["on"] = not state["on"]
            for ln in lines:
                ln.set_visible(state["on"])
            btn.configure(
                text=("◈  " if state["on"] else "⬡  ") + label + (" ON" if state["on"] else " OFF"),
                fg=color if state["on"] else MUTED,
                highlightbackground=color if state["on"] else BORDER)
            redraw_fn()
            self._popout_force_sync(popout_key)

        btn = tk.Button(parent,
                         text=("◈  " if initial else "⬡  ") + label + (" ON" if initial else " OFF"),
                         font=FONT_MONO_SM,
                         fg=color if initial else MUTED, bg=PANEL2, relief=tk.FLAT,
                         highlightbackground=color if initial else BORDER, highlightthickness=1,
                         padx=8, pady=3, command=toggle)
        btn.pack(side=tk.LEFT, padx=4)
        return btn

    def _popout_open(self, key):
        """Open the shared pop-out window (or re-target it if already open)
        on the graph identified by key."""
        src_ax = self._popout_source_ax(key)
        if src_ax is None:
            return
        if self._popout_win is None or not self._popout_win.winfo_exists():
            self._popout_build_window()
        self._popout_key = key
        self._popout_built_key = None   # force a full rebuild for the new graph
        self._popout_win.title(self._POPOUT_TITLES.get(key, "Effort Guide"))
        self._popout_render(src_ax)
        self._popout_win.deiconify()
        self._popout_win.lift()

    def _popout_build_window(self):
        win = tk.Toplevel(self)
        win.configure(bg=PANEL)
        win.geometry("1000x700")
        win.protocol("WM_DELETE_WINDOW", self._popout_close)
        win.bind("<Configure>", self._popout_on_configure)

        bar = tk.Frame(win, bg=PANEL)
        bar.pack(fill=tk.X)
        tk.Button(bar, text="✕  CLOSE", font=("Arial", 10, "bold"),
                  bg=PANEL2, fg=RED, relief=tk.FLAT, padx=10, pady=4,
                  command=self._popout_close).pack(side=tk.RIGHT, padx=8, pady=6)

        fig = Figure(figsize=(10, 7), facecolor=PANEL)
        fig.subplots_adjust(top=0.90, bottom=0.14, left=0.07, right=0.97)
        ax = fig.add_subplot(111)
        canvas = FigureCanvasTkAgg(fig, master=win)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))

        self._popout_win     = win
        self._popout_fig     = fig
        self._popout_ax      = ax
        self._popout_canvas  = canvas
        self._popout_built_key = None
        self._popout_lines     = []
        self._popout_texts     = []
        self._popout_patches   = []
        self._popout_title     = None
        self._popout_last_render_ts = 0.0
        self._popout_last_monitor = None

    def _popout_on_configure(self, event):
        """Fires on every move/resize of the pop-out window. Debounced so a
        drag across the screen doesn't spam monitor lookups or fight the
        user mid-drag — only check once the window has been still for a
        moment, then snap to fill whichever monitor it landed on."""
        if not SCREENINFO_AVAILABLE:
            return
        if self._popout_monitor_check_id is not None:
            try:
                self.after_cancel(self._popout_monitor_check_id)
            except Exception:
                pass
        self._popout_monitor_check_id = self.after(200, self._popout_maybe_snap_to_monitor)

    def _popout_maybe_snap_to_monitor(self):
        self._popout_monitor_check_id = None
        win = self._popout_win
        if win is None or not win.winfo_exists():
            return
        try:
            monitors = get_monitors()
        except Exception:
            return
        if not monitors:
            return

        # Which monitor is the window's centre currently sitting on?
        cx = win.winfo_x() + win.winfo_width() // 2
        cy = win.winfo_y() + win.winfo_height() // 2
        target = None
        for m in monitors:
            if m.x <= cx < m.x + m.width and m.y <= cy < m.y + m.height:
                target = m
                break
        if target is None:
            return

        key = (target.x, target.y, target.width, target.height)
        if key == self._popout_last_monitor:
            return   # already snapped to this monitor — the geometry() call
                     # below would otherwise re-trigger this same handler
        self._popout_last_monitor = key
        win.geometry(f"{target.width}x{target.height}+{target.x}+{target.y}")

    def _popout_close(self):
        if self._popout_monitor_check_id is not None:
            try:
                self.after_cancel(self._popout_monitor_check_id)
            except Exception:
                pass
            self._popout_monitor_check_id = None
        if self._popout_win is not None:
            try:
                self._popout_win.destroy()
            except Exception:
                pass
        self._popout_win = None
        self._popout_key = None
        self._popout_built_key = None

    _POPOUT_MIN_INTERVAL = 0.1    # seconds; 10 Hz — matches the native update
                                   # rate of the validate/performance/robot
                                   # graphs (already smooth) and only costs
                                   # ~6% timing overrun on the main thread,
                                   # vs. ~24% at the collect tab's full 20 Hz

    def _popout_sync(self, key):
        """Call after a live effort graph updates. If the pop-out window is
        open and currently targeted at this graph, mirror the new frame
        into it; cheap no-op otherwise.

        The actual cost of a redraw here is small (~9ms measured); the
        earlier freeze came from doing a full ax.clear()+rebuild of every
        artist on every tick, not from the raster cost itself. This
        interval just caps against pathological back-to-back calls.
        """
        if self._popout_win is None or self._popout_key != key:
            return
        if not self._popout_win.winfo_exists():
            self._popout_win = None
            self._popout_key = None
            return
        now = time.time()
        if now - self._popout_last_render_ts < self._POPOUT_MIN_INTERVAL:
            return
        self._popout_last_render_ts = now
        src_ax = self._popout_source_ax(key)
        if src_ax is not None:
            self._popout_render(src_ax)

    def _popout_force_sync(self, key):
        """Like _popout_sync but bypasses the rate limit — for explicit user
        actions (e.g. clicking a visibility toggle) that should reflect in
        the pop-out immediately rather than waiting out the throttle."""
        self._popout_last_render_ts = 0.0
        self._popout_sync(key)

    def _popout_render(self, src_ax):
        """Mirror src_ax into the pop-out axes, enlarged for a second
        monitor. Runs on every graph tick (as often as 20 Hz), so it must
        stay cheap: line/patch data is just updated in place. A full
        rebuild (clear + recreate every artist + legend) only happens when
        the graph's structure actually changes — e.g. the guide/labels
        being (re)drawn at START or cleared on reset/finish — detected by
        the line/text counts changing since the last frame.
        """
        src_lines = src_ax.get_lines()
        src_texts = src_ax.texts
        structure_changed = (
            self._popout_built_key != self._popout_key
            or len(self._popout_lines) != len(src_lines)
            or len(self._popout_texts) != len(src_texts)
        )

        if structure_changed:
            self._popout_full_rebuild(src_ax, src_lines, src_texts)
        else:
            for src_line, dst_line in zip(src_lines, self._popout_lines):
                dst_line.set_data(src_line.get_xdata(), src_line.get_ydata())
                dst_line.set_visible(src_line.get_visible())

        # "Now" band patches are removed + recreated by the source graph on
        # every tick, so identity never persists — just mirror that (cheap:
        # there's at most one of these).
        ax = self._popout_ax
        for p in self._popout_patches:
            try: p.remove()
            except Exception: pass
        self._popout_patches = []
        for patch in src_ax.patches:
            x0 = patch.get_x()
            x1 = x0 + patch.get_width()
            newp = ax.axvspan(x0, x1, facecolor=patch.get_facecolor(),
                               zorder=patch.get_zorder())
            self._popout_patches.append(newp)

        ax.set_xlim(src_ax.get_xlim())
        ax.set_ylim(src_ax.get_ylim())
        title_artist = getattr(src_ax, "_left_title", src_ax.title)
        if self._popout_title is not None:
            self._popout_title.set_text(title_artist.get_text())
            self._popout_title.set_color(title_artist.get_color())

        self._popout_canvas.draw_idle()

    def _popout_full_rebuild(self, src_ax, src_lines, src_texts):
        """Recreate every artist on the pop-out axes from scratch. Only
        called when the source graph's structure changes, not every tick."""
        ax = self._popout_ax
        ax.clear()
        ax.set_facecolor(PANEL2)
        for sp in ax.spines.values():
            sp.set_color(BORDER); sp.set_linewidth(1.0)
        ax.tick_params(colors=MUTED, labelsize=15)

        self._popout_lines = []
        for line in src_lines:
            marker = line.get_marker()
            has_marker = marker not in (None, "None", "")
            lbl = line.get_label()
            # axhline/axvline use a blended axes-fraction/data transform
            # instead of plain data coordinates (e.g. the dotted 25/50/75%
            # guides and the dashed playhead) — preserve it, or the line's
            # [0,1]-ranged coordinate gets misread as a tiny data-space
            # sliver and effectively vanishes.
            src_transform = line.get_transform()
            if src_transform is src_ax.get_yaxis_transform():
                transform = ax.get_yaxis_transform()
            elif src_transform is src_ax.get_xaxis_transform():
                transform = ax.get_xaxis_transform()
            else:
                transform = ax.transData

            # axvline/axhline artists are frequently updated with a
            # single-element set_xdata()/set_ydata() call elsewhere in the
            # app (e.g. the playhead's set_xdata([elapsed])), leaving the
            # *other* axis at its original 2-element span. Line2D tolerates
            # that mismatch internally, but ax.plot()'s strict shape check
            # does not — so reconcile it here rather than crashing.
            xdata = np.asarray(line.get_xdata(), dtype=float)
            ydata = np.asarray(line.get_ydata(), dtype=float)
            if len(xdata) != len(ydata):
                if len(xdata) == 1 and len(ydata) > 1:
                    xdata = np.full(len(ydata), xdata[0])
                elif len(ydata) == 1 and len(xdata) > 1:
                    ydata = np.full(len(xdata), ydata[0])
                else:
                    n = min(len(xdata), len(ydata))
                    xdata, ydata = xdata[:n], ydata[:n]

            new_line, = ax.plot(xdata, ydata,
                    color=line.get_color(),
                    lw=line.get_linewidth() * 1.3,
                    linestyle=line.get_linestyle(),
                    marker=marker if has_marker else None,
                    markersize=(line.get_markersize() * 1.3) if has_marker else None,
                    alpha=line.get_alpha(),
                    zorder=line.get_zorder(),
                    solid_capstyle=line.get_solid_capstyle(),
                    label=None if lbl.startswith("_") else lbl,
                    transform=transform)
            new_line.set_visible(line.get_visible())
            self._popout_lines.append(new_line)

        self._popout_texts = []
        for txt in src_texts:
            transform = (ax.get_yaxis_transform()
                         if txt.get_transform() is src_ax.get_yaxis_transform()
                         else ax.transData)
            x, y = txt.get_position()
            new_txt = ax.text(x, y, txt.get_text(), color=txt.get_color(),
                    fontsize=max(txt.get_fontsize() * 1.3, 11),
                    ha=txt.get_ha(), va=txt.get_va(),
                    fontweight=txt.get_fontweight(),
                    transform=transform, clip_on=False)
            self._popout_texts.append(new_txt)

        ax.set_xlabel(src_ax.get_xlabel(), color=MUTED, fontsize=15, labelpad=8)
        ax.set_ylabel(src_ax.get_ylabel(), color=MUTED, fontsize=15, labelpad=10)
        title_artist = getattr(src_ax, "_left_title", src_ax.title)
        self._popout_title = ax.set_title(
            title_artist.get_text(), color=title_artist.get_color(),
            fontsize=20, loc="left", pad=14, fontweight="bold")

        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.10),
                      ncol=min(len(handles), 3), fontsize=15, labelcolor=TEXT,
                      facecolor=PANEL2, edgecolor=BORDER, framealpha=0.85,
                      borderpad=0.8, handlelength=2.2, columnspacing=2.0)

        self._popout_patches = []   # cleared by ax.clear(); rebuilt by caller
        self._popout_built_key = self._popout_key

    # ── COLLECT TAB ──────────────────────────
    def _build_collect_tab(self, parent):
        # ── Split pane: effort guide (top) + grip force (bottom) ─────────────
        pane = tk.PanedWindow(parent, orient=tk.VERTICAL,
                              bg=BG, sashwidth=5, sashrelief=tk.FLAT,
                              sashpad=2, opaqueresize=True)
        pane.pack(fill=tk.BOTH, expand=True)

        # ── Top pane: ramp overlay canvas + cue row ───────────────────────────
        overlay_f = tk.Frame(pane, bg=PANEL)
        self._build_ramp_overlay_canvas(overlay_f)

        # Small cue row below the graph
        cue_row = tk.Frame(overlay_f, bg=PANEL)
        cue_row.pack(fill=tk.X, padx=10, pady=(2, 4))
        self._cue_label = tk.Label(cue_row, text="READY",
                                    font=("Arial", 18, "bold"),
                                    fg=BORDER, bg=PANEL)
        self._cue_label.pack(side=tk.LEFT, padx=(0, 12))
        self._cue_sub = tk.Label(cue_row,
                                  text="MVC calibration required before recording",
                                  font=FONT_MONO_SM, fg=MUTED, bg=PANEL)
        self._cue_sub.pack(side=tk.LEFT)
        # Progress bar
        self._prog_canvas = tk.Canvas(overlay_f, height=3, bg=BORDER, highlightthickness=0)
        self._prog_canvas.pack(fill=tk.X)
        self._prog_fill = self._prog_canvas.create_rectangle(0, 0, 0, 3, fill=ACCENT, outline="")

        pane.add(overlay_f, stretch="always")

        # ── Bottom pane: grip force chart ────────────────────────────────────
        chart_f = tk.Frame(pane, bg=BG)
        pane.add(chart_f, stretch="always")
        self._fig = Figure(figsize=(6, 3.0), facecolor=BG, tight_layout=True)
        gs = GridSpec(1, 1, figure=self._fig)
        self._ax_force = self._fig.add_subplot(gs[0])

        # Dummy hidden axes kept so device-switch / sensor-toggle code
        # that references _ax_s0/s1/s2 doesn't crash
        self._ax_s0 = self._fig.add_axes([0,0,0,0])
        self._ax_s1 = self._fig.add_axes([0,0,0,0])
        self._ax_s2 = self._fig.add_axes([0,0,0,0])
        for _ax in (self._ax_s0, self._ax_s1, self._ax_s2):
            _ax.set_visible(False)

        self._ax_force.set_facecolor(PANEL2)
        self._ax_force.tick_params(colors=MUTED, labelsize=11)
        self._ax_force.set_title("GRIP FORCE — % MVC", color=MUTED, fontsize=11, loc="left", pad=4)
        self._ax_force.set_xlabel("Samples", color=MUTED, fontsize=11, labelpad=4)
        self._ax_force.set_ylabel("% MVC", color=MUTED, fontsize=11, labelpad=6)
        for spine in self._ax_force.spines.values():
            spine.set_color(BORDER); spine.set_linewidth(0.5)
        self._ax_force.set_xlim(0, HIST_LEN); self._ax_force.set_ylim(0, 120)

        self._line_s0,    = self._ax_s0.plot([], [], color=ACCENT,  lw=1.2)
        self._line_s1,    = self._ax_s1.plot([], [], color=ACCENT2, lw=1.2)
        self._line_s2,    = self._ax_s2.plot([], [], color=PURPLE,  lw=1.2)
        self._line_force, = self._ax_force.plot([], [], color=GREEN, lw=1.8)

        self._chart_canvas = FigureCanvasTkAgg(self._fig, master=chart_f)
        self._chart_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._anim = animation.FuncAnimation(
            self._fig, self._update_collect_charts,
            interval=80, blit=True, cache_frame_data=False)

        # Set initial sash at 50% after layout is computed
        self._collect_pane = pane
        parent.after(100, self._init_collect_sash)

        # Controls (outside the pane so they're never squeezed)
        ctrl = tk.Frame(parent, bg=PANEL)
        ctrl.pack(fill=tk.X, side=tk.BOTTOM)
        info_f = tk.Frame(ctrl, bg=PANEL); info_f.pack(side=tk.LEFT, padx=10, pady=6)
        self._proto_lbl   = self._info_stat(info_f, "PHASE",    "—")
        self._elapsed_lbl = self._info_stat(info_f, "ELAPSED",  "00:00")
        self._wins_lbl    = self._info_stat(info_f, "WINDOWS",  "0")
        self._mvc_lbl     = self._info_stat(info_f, "MVC",      "—")

        # ── Sensor count toggle ──────────────────────────────────────────
        sensor_f = tk.Frame(ctrl, bg=PANEL)
        sensor_f.pack(side=tk.LEFT, padx=16, pady=6)
        tk.Label(sensor_f, text="SENSORS", font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack()
        sc_row = tk.Frame(sensor_f, bg=PANEL)
        sc_row.pack()
        self._sensor_count_var = tk.IntVar(value=NUM_SENSORS)
        self._sensor_btns = {}
        for n in [1, 2, 3]:
            b = tk.Button(sc_row, text=str(n),
                          font=("Arial", 10, "bold"),
                          bg=ACCENT if n == NUM_SENSORS else PANEL2,
                          fg=BG     if n == NUM_SENSORS else MUTED,
                          relief=tk.FLAT, width=3, pady=4,
                          command=lambda v=n: self._set_sensor_count(v))
            b.pack(side=tk.LEFT, padx=2)
            self._sensor_btns[n] = b
        self._sensor_count_lbl = tk.Label(sensor_f,
            text=f"{NUM_SENSORS * (BINS_PER_SENSOR+1)} inputs",
            font=("Courier New", 8), fg=YELLOW, bg=PANEL)
        self._sensor_count_lbl.pack()

        btn_f = tk.Frame(ctrl, bg=PANEL)
        btn_f.pack(side=tk.RIGHT, padx=10, pady=6)
        btn_f.grid_columnconfigure(0, weight=1)
        btn_f.grid_columnconfigure(1, weight=1)

        self._mvc_btn = tk.Button(btn_f, text="① MVC CALIBRATE",
                                   font=("Arial",9,"bold"),
                                   bg=PANEL2, fg=YELLOW, relief=tk.FLAT,
                                   padx=8, pady=6, command=self._start_mvc,
                                   width=16)
        self._mvc_btn.grid(row=0, column=0, padx=3, pady=3, sticky="ew")

        self._start_btn = tk.Button(btn_f, text="② START PROTOCOL",
                                     font=("Arial",9,"bold"),
                                     bg=PANEL2, fg=GREEN, relief=tk.FLAT,
                                     padx=8, pady=6, command=self._start_protocol,
                                     state=tk.DISABLED, width=16)
        self._start_btn.grid(row=0, column=1, padx=3, pady=3, sticky="ew")

        # Ramp button retained for API compatibility but hidden — protocol
        # now always runs ramps directly from START PROTOCOL.
        self._ramp_btn  = tk.Button(btn_f, text="③ RAMP TRIALS",
                                     font=("Arial",9,"bold"),
                                     bg=PANEL2, fg=PURPLE, relief=tk.FLAT,
                                     padx=8, pady=6, command=self._start_ramp,
                                     state=tk.DISABLED, width=16)
        # Hidden — merged into START PROTOCOL
        # self._ramp_btn.grid(row=1, column=0, padx=3, pady=3, sticky="ew")

        self._stop_btn  = tk.Button(btn_f, text="■ STOP",
                                     font=("Arial",9,"bold"),
                                     bg=PANEL2, fg=RED, relief=tk.FLAT,
                                     padx=8, pady=6, command=self._stop_protocol,
                                     state=tk.DISABLED, width=16)
        self._stop_btn.grid(row=1, column=0, columnspan=2, padx=3, pady=3, sticky="ew")

    def _sync_sensor_ui(self, n):
        """Called on main thread when serial auto-detection reports a different
        sensor count than the current UI selection. Updates buttons and label
        without clearing the dataset — data collection may already be running.
        """
        self._sensor_count_var.set(n)
        for num, btn in self._sensor_btns.items():
            btn.configure(
                bg=ACCENT if num == n else PANEL2,
                fg=BG     if num == n else MUTED)
        self._sensor_count_lbl.configure(
            text=f"{n * (BINS_PER_SENSOR+1)} inputs  (auto-detected)")
        self._ax_s2.set_visible(n >= 3)

    def _set_sensor_count(self, n):
        """Change NUM_SENSORS at runtime. Resets dataset, model, and feature names.
        Safe to call before any collection has started, or to switch between experiments.
        Blocked while recording or training is active.
        """
        global NUM_SENSORS, FEATURE_NAMES, _latest_emg

        if STATE.recording:
            self._exo_log_line("Cannot change sensors while recording.", RED)                 if hasattr(self, "_exo_log") else None
            return
        if getattr(STATE, "training", False):
            return

        NUM_SENSORS   = n
        FEATURE_NAMES = _build_feature_names(n, BINS_PER_SENSOR)
        _latest_emg   = [0.0] * (n * (BINS_PER_SENSOR + 1))

        # Reset all data that depends on sensor count
        STATE.dataset.clear()
        STATE.level_window_counts = {lvl: 0 for lvl in FORCE_LEVELS}
        STATE.win_buf.clear()
        STATE.win_force_buf.clear()
        STATE.model         = None
        STATE.scaler        = None
        STATE.model_trained = False
        STATE.inferring     = False
        STATE.val_pred.clear()
        STATE.val_true.clear()

        # Update toggle button highlight
        for num, btn in self._sensor_btns.items():
            btn.configure(
                bg=ACCENT if num == n else PANEL2,
                fg=BG     if num == n else MUTED)

        # Update input count label
        self._sensor_count_lbl.configure(
            text=f"{n * (BINS_PER_SENSOR+1)} inputs")

        # Update collect chart titles to reflect active sensors
        sensor_colors = [ACCENT, ACCENT2, PURPLE]
        chart_titles  = [f"SENSOR {s} — Activity" for s in range(n)]
        # Axes always exist for up to 3 sensors — show/hide as needed
        all_axes = [self._ax_s0, self._ax_s1,
                    getattr(self, "_ax_s2", None)]
        all_lines = [self._line_s0, self._line_s1,
                     getattr(self, "_line_s2", None)]
        for i, (ax, line) in enumerate(zip(all_axes, all_lines)):
            if ax is None:
                continue
            if i < n:
                ax.set_title(f"SENSOR {i} — Activity",
                             color=MUTED, fontsize=8, loc="left", pad=2)
                ax.set_visible(True)
                if line:
                    line.set_data([], [])
            else:
                ax.set_visible(False)

        # Notify user
        self._cue_sub.configure(
            text=f"{n} sensor{'s' if n>1 else ''} selected "
                 f"({n*(BINS_PER_SENSOR+1)} inputs) — dataset cleared, retrain required")
        self._wins_lbl.configure(text="0")
        self._mvc_lbl.configure(text="—")
        self._start_btn.configure(state=tk.DISABLED)
        self._ramp_btn.configure(state=tk.DISABLED)

    def _init_collect_sash(self):
        """Set the collect tab pane sash to 60/40 split (top 1.5x taller than bottom)."""
        try:
            total = self._collect_pane.winfo_height()
            if total > 10:
                self._collect_pane.sash_place(0, 0, int(total * 0.60))
            else:
                self._collect_pane.after(100, self._init_collect_sash)
        except Exception:
            pass

    def _info_stat(self, parent, label, value):
        f = tk.Frame(parent, bg=PANEL); f.pack(side=tk.LEFT, padx=10)
        tk.Label(f, text=label, font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack()
        v = tk.Label(f, text=value, font=("Courier New",12,"bold"), fg=ACCENT, bg=PANEL)
        v.pack()
        return v

    def _update_collect_charts(self, frame):
        s0    = list(STATE.act0_hist)
        s1    = list(STATE.act1_hist)
        s2    = list(STATE.act2_hist)
        force = list(STATE.force_hist)

        # RMS lines kept in sync for data integrity even though axes are hidden
        self._line_s0.set_data(range(len(s0)), s0)
        self._line_s1.set_data(range(len(s1)), s1)
        self._line_s2.set_data(range(len(s2)), s2)
        self._line_force.set_data(range(len(force)), force)

        if force:
            self._ax_force.set_ylim(0, max(max(force) * 1.1, 120))

        return (self._line_force,)


    def _refresh_live_channel_labels(self):
        """Update the right-panel card titles based on active EMG device."""
        if ACTIVE_DEVICE == "mindrove":
            for i in range(8):
                self._sensor_name_labels[i].configure(text=f"CH {i}")
        else:
            for i in range(8):
                if i < 3:
                    self._sensor_name_labels[i].configure(text=f"SENSOR {i}")
                else:
                    self._sensor_name_labels[i].configure(text="—")


    # ── TRAIN TAB ────────────────────────────
    def _build_train_tab(self, parent):
        sc = tk.Canvas(parent, bg=BG, highlightthickness=0)
        sb = ttk.Scrollbar(parent, orient=tk.VERTICAL, command=sc.yview)
        sf = tk.Frame(sc, bg=BG)
        sf.bind("<Configure>", lambda e: sc.configure(scrollregion=sc.bbox("all")))
        sc.create_window((0,0), window=sf, anchor="nw")
        sc.configure(yscrollcommand=sb.set)
        sc.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)

        # Stat cards
        stats_f = tk.Frame(sf, bg=BG); stats_f.pack(fill=tk.X, padx=16, pady=12)
        self._t_n    = self._stat_card(stats_f, "Windows",     "0",  "collected")
        self._t_r2   = self._stat_card(stats_f, "CV R²",       "—",  "cross-validated")
        self._t_rmse = self._stat_card(stats_f, "CV %RMSE",    "—",  "normalized RMSE")
        self._t_r    = self._stat_card(stats_f, "Pearson r",   "—",  "correlation")
        self._t_r2_active = self._stat_card(stats_f, "CV R² (active)", "—", "excl. 0% MVC rest")

        # Model selector
        mod_f = tk.Frame(sf, bg=BG); mod_f.pack(fill=tk.X, padx=16)
        tk.Label(mod_f, text="REGRESSOR", font=("Arial",9,"bold"),
                 fg=MUTED, bg=BG).pack(anchor=tk.W)
        chip_row = tk.Frame(mod_f, bg=BG); chip_row.pack(fill=tk.X, pady=4)
        self._model_var   = tk.StringVar(value="XGBoost")
        self._model_chips = {}
        model_list = ["XGBoost","SVR","Random Forest","Gradient Boosting"]
        if not XGB_AVAILABLE:
            model_list[0] = "Gradient Boosting"
        if LGB_AVAILABLE:
            model_list.append("LightGBM")
        for m in model_list:
            btn = tk.Button(chip_row, text=m, font=FONT_MONO_SM,
                            bg=PANEL2, fg=MUTED, relief=tk.FLAT,
                            padx=8, pady=4,
                            command=lambda x=m: self._select_model(x))
            btn.pack(side=tk.LEFT, padx=2)
            self._model_chips[m] = btn
        self._select_model(model_list[0])

        # CV folds
        hp_f = tk.Frame(mod_f, bg=BG); hp_f.pack(fill=tk.X)
        lf = tk.Frame(hp_f, bg=BG); lf.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,8))
        tk.Label(lf, text="CV FOLDS", font=FONT_MONO_SM, fg=MUTED, bg=BG).pack(anchor=tk.W)
        self._cv_var = tk.StringVar(value="5")
        tk.Entry(lf, textvariable=self._cv_var, width=5,
                 bg=PANEL2, fg=TEXT, font=FONT_MONO_SM, relief=tk.FLAT,
                 insertbackground=TEXT, highlightbackground=BORDER,
                 highlightthickness=1).pack(fill=tk.X)

        self._train_btn = tk.Button(mod_f, text="⬡  TRAIN MODEL",
                                     font=("Arial",11,"bold"),
                                     bg=PANEL2, fg=ACCENT, relief=tk.FLAT,
                                     pady=8, command=self._run_training)
        self._train_btn.pack(fill=tk.X, pady=8)

        # Scatter plot: predicted vs actual
        sp_f = tk.Frame(sf, bg=BG); sp_f.pack(fill=tk.X, padx=16)
        tk.Label(sp_f, text="PREDICTED vs ACTUAL FORCE", font=("Arial",9,"bold"),
                 fg=MUTED, bg=BG).pack(anchor=tk.W)
        self._scatter_fig = Figure(figsize=(5,2.8), facecolor=BG, tight_layout=True)
        self._scatter_ax  = self._scatter_fig.add_subplot(111)
        self._scatter_ax.set_facecolor(PANEL2)
        self._scatter_ax.spines[:].set_color(BORDER)
        self._scatter_ax.tick_params(colors=MUTED, labelsize=7)
        self._scatter_ax.set_xlabel("Actual Force (% MVC)", color=MUTED, fontsize=8)
        self._scatter_ax.set_ylabel("Predicted Force (% MVC)", color=MUTED, fontsize=8)
        self._scatter_canvas = FigureCanvasTkAgg(self._scatter_fig, master=sp_f)
        self._scatter_canvas.get_tk_widget().pack(fill=tk.X)

        # Feature importance
        fi_f = tk.Frame(sf, bg=BG); fi_f.pack(fill=tk.X, padx=16, pady=8)
        tk.Label(fi_f, text="FEATURE IMPORTANCE (SHAP)", font=("Arial",9,"bold"),
                 fg=MUTED, bg=BG).pack(anchor=tk.W)
        self._fi_fig = Figure(figsize=(5,2.8), facecolor=BG, tight_layout=True)
        self._fi_ax  = self._fi_fig.add_subplot(111)
        self._fi_ax.set_facecolor(PANEL2)
        self._fi_canvas = FigureCanvasTkAgg(self._fi_fig, master=fi_f)
        self._fi_canvas.get_tk_widget().pack(fill=tk.X)

        # Log
        log_f = tk.Frame(sf, bg=BG); log_f.pack(fill=tk.X, padx=16, pady=(0,16))
        tk.Label(log_f, text="TRAINING LOG", font=("Arial",9,"bold"),
                 fg=MUTED, bg=BG).pack(anchor=tk.W)
        self._log_text = tk.Text(log_f, height=8, bg=PANEL2, fg=TEXT,
                                  font=FONT_MONO_SM, relief=tk.FLAT,
                                  state=tk.DISABLED, wrap=tk.WORD)
        self._log_text.pack(fill=tk.X)

    def _stat_card(self, parent, label, value, sub):
        f = tk.Frame(parent, bg=PANEL,
                     highlightbackground=BORDER, highlightthickness=1)
        f.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        tk.Label(f, text=label, font=("Arial",8,"bold"),
                 fg=MUTED, bg=PANEL).pack(anchor=tk.W, padx=8, pady=(8,0))
        v = tk.Label(f, text=value, font=("Arial",20,"bold"),
                     fg=ACCENT, bg=PANEL)
        v.pack(anchor=tk.W, padx=8)
        tk.Label(f, text=sub, font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack(anchor=tk.W, padx=8, pady=(0,8))
        return v

    # ── VALIDATE TAB ─────────────────────────
    def _build_validate_tab(self, parent):
        # ── TOP CONTROLS BAR (always fully visible) ─────────────────────────
        top = tk.Frame(parent, bg=PANEL,
                       highlightbackground=BORDER, highlightthickness=1)
        top.pack(fill=tk.X, padx=16, pady=(10,4))

        # Row 1: duration + buttons + progress
        row1 = tk.Frame(top, bg=PANEL); row1.pack(fill=tk.X, padx=8, pady=(6,4))

        # Duration entry
        tk.Label(row1, text="DURATION (s)", font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT, padx=(0,2))
        self._val_dur_var = tk.StringVar(value="45")
        tk.Entry(row1, textvariable=self._val_dur_var, width=5,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT,
                 highlightbackground=BORDER, highlightthickness=1
                 ).pack(side=tk.LEFT, padx=(0,12))

        # Start / Stop button
        self._val_btn = tk.Button(row1, text="▶  START VALIDATION",
                                   font=("Arial", 10, "bold"),
                                   bg=PANEL2, fg=ACCENT, relief=tk.FLAT,
                                   padx=12, pady=5,
                                   command=self._toggle_validate)
        self._val_btn.pack(side=tk.LEFT, padx=(0,6))

        # Clear button
        tk.Button(row1, text="⟳  CLEAR", font=("Arial", 10, "bold"),
                  bg=PANEL2, fg=YELLOW, relief=tk.FLAT,
                  padx=12, pady=5,
                  command=self._clear_val).pack(side=tk.LEFT, padx=(0,6))

        # Send to Deploy button (hidden until recording completes)
        self._send_deploy_btn = tk.Button(row1, text="→  SEND TO DEPLOY",
                                           font=("Arial", 10, "bold"),
                                           bg="#1a3a1a", fg=GREEN, relief=tk.FLAT,
                                           padx=12, pady=5,
                                           state=tk.DISABLED,
                                           command=self._send_to_deploy)
        self._send_deploy_btn.pack(side=tk.LEFT, padx=(0,16))

        # Progress bar + time label inline
        self._val_prog_lbl = tk.Label(row1, text="READY",
                                       font=FONT_MONO_SM, fg=MUTED, bg=PANEL, width=26, anchor=tk.W)
        self._val_prog_lbl.pack(side=tk.LEFT, padx=(0,6))
        prog_track = tk.Canvas(row1, height=8, bg=BORDER, highlightthickness=0)
        prog_track.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,8))
        self._val_prog_bar   = prog_track.create_rectangle(0, 0, 0, 8, fill=ACCENT, outline="")
        self._val_prog_track = prog_track

        # ── Row 2: smoothing controls ────────────────────────────────────────
        row2 = tk.Frame(top, bg=PANEL); row2.pack(fill=tk.X, padx=8, pady=(0,6))

        # EMA toggle + slider
        self._ema_btn = tk.Button(row2, text="⬡  EMA OFF",
                                   font=FONT_MONO_SM, bg=PANEL2, fg=MUTED, relief=tk.FLAT,
                                   highlightbackground=BORDER, highlightthickness=1,
                                   padx=8, pady=3, command=self._toggle_ema)
        self._ema_btn.pack(side=tk.LEFT, padx=(0,4))
        tk.Label(row2, text="α", font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack(side=tk.LEFT)
        self._ema_alpha_var = tk.DoubleVar(value=0.2)
        tk.Scale(row2, variable=self._ema_alpha_var,
                 from_=0.05, to=0.95, resolution=0.05,
                 orient=tk.HORIZONTAL, length=90,
                 bg=PANEL, fg=TEXT, troughcolor=PANEL2,
                 highlightthickness=0, bd=0, showvalue=0,
                 command=lambda v: setattr(STATE, "ema_alpha", float(v))
                 ).pack(side=tk.LEFT)
        self._ema_alpha_lbl = tk.Label(row2, text="0.20",
                                        font=FONT_MONO_SM, fg=ACCENT, bg=PANEL, width=4)
        self._ema_alpha_lbl.pack(side=tk.LEFT, padx=(2, 16))
        def _update_alpha_lbl(*_):
            self._ema_alpha_lbl.configure(text=f"{self._ema_alpha_var.get():.2f}")
        self._ema_alpha_var.trace_add("write", _update_alpha_lbl)

        # Median toggle + slider
        self._med_btn = tk.Button(row2, text="⬡  MEDIAN OFF",
                                   font=FONT_MONO_SM, bg=PANEL2, fg=MUTED, relief=tk.FLAT,
                                   highlightbackground=BORDER, highlightthickness=1,
                                   padx=8, pady=3, command=self._toggle_median)
        self._med_btn.pack(side=tk.LEFT, padx=(0,4))
        tk.Label(row2, text="win", font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack(side=tk.LEFT)
        self._med_win_var = tk.IntVar(value=7)
        tk.Scale(row2, variable=self._med_win_var,
                 from_=3, to=21, resolution=2,
                 orient=tk.HORIZONTAL, length=90,
                 bg=PANEL, fg=TEXT, troughcolor=PANEL2,
                 highlightthickness=0, bd=0, showvalue=0,
                 command=lambda v: setattr(STATE, "median_window", int(v))
                 ).pack(side=tk.LEFT)
        self._med_win_lbl = tk.Label(row2, text="7",
                                      font=FONT_MONO_SM, fg=PURPLE, bg=PANEL, width=3)
        self._med_win_lbl.pack(side=tk.LEFT, padx=(2, 0))
        def _update_win_lbl(*_):
            self._med_win_lbl.configure(text=str(self._med_win_var.get()))
        self._med_win_var.trace_add("write", _update_win_lbl)

        # ── METRICS ROW ──────────────────────────────────────────────────────
        metrics_f = tk.Frame(parent, bg=PANEL,
                              highlightbackground=BORDER, highlightthickness=1)
        metrics_f.pack(fill=tk.X, padx=16, pady=(0,4))
        tk.Label(metrics_f, text="LIVE VALIDATION METRICS",
                 font=("Arial", 9, "bold"), fg=MUTED, bg=PANEL).pack(
                     anchor=tk.W, padx=8, pady=(6,2))
        tk.Label(metrics_f, text="RAW MODEL",
                 font=("Courier New", 8, "bold"), fg=MUTED, bg=PANEL).pack(anchor=tk.W, padx=8)
        row = tk.Frame(metrics_f, bg=PANEL); row.pack(fill=tk.X, padx=8, pady=(0,2))
        self._live_r2    = self._metric_card(row, "R²",        "—", ACCENT)
        self._live_rmse  = self._metric_card(row, "%RMSE",     "—", YELLOW)
        self._live_r     = self._metric_card(row, "Pearson r", "—", GREEN)
        self._live_lag   = self._metric_card(row, "Lag (ms)",  "—", ACCENT2)

        self._filt_label = tk.Label(metrics_f, text="FILTERED OUTPUT",
                                     font=("Courier New", 8, "bold"), fg=PURPLE, bg=PANEL)
        self._filt_row = tk.Frame(metrics_f, bg=PANEL)
        self._filt_r2    = self._metric_card(self._filt_row, "R²",        "—", PURPLE)
        self._filt_rmse  = self._metric_card(self._filt_row, "%RMSE",     "—", PURPLE)
        self._filt_r     = self._metric_card(self._filt_row, "Pearson r", "—", PURPLE)
        self._filt_lag   = self._metric_card(self._filt_row, "Lag (ms)",  "—", PURPLE)
        tk.Frame(metrics_f, bg=PANEL, height=4).pack()

        # ── VALIDATION OVERLAY CHART ──────────────────────────────────────────
        # Exact same structure as the collect tab effort guide:
        #   • Fixed x-axis = full protocol duration, never moves
        #   • Red solid path = target effort in Newtons (guide)
        #   • Cyan line + dot = model predicted force, streams left→right
        #   • Green line      = ground truth (GDX grip scale), streams left→right
        #   • Cyan dashed playhead + now-band, driven by _val_poll at 100 ms
        val_overlay_f = tk.Frame(parent, bg=PANEL)
        val_overlay_f.pack(fill=tk.X, padx=16, pady=(0, 2))
        self._build_val_overlay_canvas(val_overlay_f)

        # ── SCATTER CHART (compact fixed height) ─────────────────────────────
        sc_f = tk.Frame(parent, bg=BG); sc_f.pack(fill=tk.X, padx=16, pady=(0,4))
        tk.Label(sc_f, text="SCATTER — PREDICTED vs ACTUAL",
                 font=("Arial", 9, "bold"), fg=MUTED, bg=BG).pack(anchor=tk.W)
        self._lval_fig = Figure(figsize=(7, 1.9), facecolor=BG, tight_layout=True)
        self._lval_ax  = self._lval_fig.add_subplot(111)
        self._lval_ax.set_facecolor(PANEL2)
        self._lval_ax.spines[:].set_color(BORDER)
        self._lval_ax.tick_params(colors=MUTED, labelsize=7)
        self._lval_canvas = FigureCanvasTkAgg(self._lval_fig, master=sc_f)
        self._lval_canvas.get_tk_widget().pack(fill=tk.X)


    def _metric_card(self, parent, label, value, color):
        f = tk.Frame(parent, bg=PANEL2,
                     highlightbackground=BORDER, highlightthickness=1)
        f.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4, pady=4)
        tk.Label(f, text=label, font=FONT_MONO_SM, fg=MUTED, bg=PANEL2).pack(pady=(6,0))
        v = tk.Label(f, text=value, font=("Courier New",16,"bold"),
                     fg=color, bg=PANEL2)
        v.pack(pady=(0,6))
        return v

    # ── RIGHT PANEL ──────────────────────────
    def _build_right(self, parent):
        frame = tk.Frame(parent, bg=PANEL, width=260)
        frame.pack_propagate(False)

        # Live sensor readouts
        hdr = tk.Frame(frame, bg=PANEL); hdr.pack(fill=tk.X, padx=8, pady=(8,4))
        tk.Label(hdr, text="LIVE SENSORS", font=("Arial",9,"bold"),
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT)
        tk.Frame(hdr, bg=BORDER, height=1).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=6)

        grid = tk.Frame(frame, bg=PANEL); grid.pack(fill=tk.X, padx=8)
        self._sensor_vals = {}
        self._sensor_bars = {}
        self._sensor_name_labels = {}
        sensor_colors = [ACCENT, ACCENT2, PURPLE, GREEN, YELLOW, "#4cc9f0", "#f72585", "#90be6d"]

        # 8 live cards so MindRove can show all 8 channels at once.
        # For uMyo, only the first 1–3 cards are actively used.
        for i in range(8):
            r, c = divmod(i, 2)
            card = tk.Frame(grid, bg=PANEL2,
                            highlightbackground=BORDER, highlightthickness=1)
            card.grid(row=r, column=c, sticky="nsew", padx=2, pady=2)
            grid.grid_columnconfigure(c, weight=1)

            name = tk.Label(card, text=f"CH {i}", font=FONT_MONO_SM,
                            fg=MUTED, bg=PANEL2)
            name.pack(pady=(6,0))
            val = tk.Label(card, text="—", font=("Courier New",12,"bold"),
                           fg=sensor_colors[i % len(sensor_colors)], bg=PANEL2)
            val.pack()
            bc = tk.Canvas(card, height=3, bg=BORDER, highlightthickness=0)
            bc.pack(fill=tk.X, padx=6, pady=(2,8))
            bar = bc.create_rectangle(0,0,0,3, fill=sensor_colors[i % len(sensor_colors)], outline="")

            self._sensor_name_labels[i] = name
            self._sensor_vals[i] = val
            self._sensor_bars[i] = (bc, bar)

        # GDX force readout
        force_hdr = tk.Frame(frame, bg=PANEL); force_hdr.pack(fill=tk.X, padx=8, pady=(12,4))
        tk.Label(force_hdr, text="GRIP FORCE", font=("Arial",9,"bold"),
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT)
        tk.Frame(force_hdr, bg=BORDER, height=1).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        self._force_val = tk.Label(frame, text="0.00 N",
                                    font=("Courier New",14,"bold"),
                                    fg=GREEN, bg=PANEL)
        self._force_val.pack(padx=8, pady=(4,0), anchor=tk.W)
        self._force_sub = tk.Label(frame, text="(calibrate MVC)",
                                    font=FONT_MONO_SM,
                                    fg=MUTED, bg=PANEL)
        self._force_sub.pack(padx=8, pady=(0,4), anchor=tk.W)
        force_bc = tk.Canvas(frame, height=5, bg=BORDER, highlightthickness=0)
        force_bc.pack(fill=tk.X, padx=8, pady=(0,8))
        self._force_bar = force_bc.create_rectangle(0,0,0,5, fill=GREEN, outline="")
        self._force_bar_c = force_bc

        # MVC status
        mvc_hdr = tk.Frame(frame, bg=PANEL); mvc_hdr.pack(fill=tk.X, padx=8, pady=(4,4))
        tk.Label(mvc_hdr, text="MVC STATUS", font=("Arial",9,"bold"),
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT)
        tk.Frame(mvc_hdr, bg=BORDER, height=1).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        self._mvc_status = tk.Label(frame, text="Not calibrated",
                                     font=FONT_MONO_SM, fg=YELLOW,
                                     bg=PANEL, wraplength=240, justify=tk.LEFT)
        self._mvc_status.pack(padx=8, anchor=tk.W)

        # Feature snapshot
        snap_hdr = tk.Frame(frame, bg=PANEL); snap_hdr.pack(fill=tk.X, padx=8, pady=(12,4))
        tk.Label(snap_hdr, text="FEATURE SNAPSHOT", font=("Arial",9,"bold"),
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT)
        tk.Frame(snap_hdr, bg=BORDER, height=1).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=6)

        self._snap_vals = {}
        # uMyo snapshot items (default)
        umyo_snap_items = [
            ("S0 ACTIVITY",   "s0_act"),
            ("S1 ACTIVITY",   "s1_act"),
            ("S0 MEAN BIN",   "s0_b0"),
            ("S1 MEAN BIN",   "s1_b0"),
            ("DIFF MEAN",     "diff_mean"),
            ("RATIO MEAN",    "ratio_mean"),
            ("TOTAL ACT",     "total_act"),
            ("S0 SLOPE",      "s0_slope"),
            ("S1 SLOPE",      "s1_slope"),
        ]
        # MindRove snapshot items — one row per channel for RMS
        mr_snap_items = [(f"CH{ch} RMS", f"ch{ch}_rms") for ch in range(MR_N_CHANNELS)] + [
            ("CH0 WL",  "ch0_wl"),
            ("CH0 ZC",  "ch0_zc"),
        ]
        snap_items = umyo_snap_items   # default; swapped by _select_device if needed
        for label, key in snap_items:
            row = tk.Frame(frame, bg=PANEL); row.pack(fill=tk.X, padx=8)
            tk.Label(row, text=label, font=FONT_MONO_SM, fg=MUTED,
                     bg=PANEL, width=13, anchor=tk.W).pack(side=tk.LEFT)
            v = tk.Label(row, text="—", font=FONT_MONO_SM,
                         fg=TEXT, bg=PANEL, anchor=tk.E)
            v.pack(side=tk.RIGHT)
            tk.Frame(row, bg=BORDER, height=1).pack(fill=tk.X, pady=2)
            self._snap_vals[key] = v

        # Predicted force
        pf_hdr = tk.Frame(frame, bg=PANEL); pf_hdr.pack(fill=tk.X, padx=8, pady=(12,4))
        tk.Label(pf_hdr, text="PREDICTED FORCE", font=("Arial",9,"bold"),
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT)
        tk.Frame(pf_hdr, bg=BORDER, height=1).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        self._pred_val = tk.Label(frame, text="—",
                                   font=("Courier New",22,"bold"),
                                   fg=ACCENT, bg=PANEL)
        self._pred_val.pack(padx=8, pady=4, anchor=tk.W)
        pred_bc = tk.Canvas(frame, height=5, bg=BORDER, highlightthickness=0)
        pred_bc.pack(fill=tk.X, padx=8, pady=(0,8))
        self._pred_bar   = pred_bc.create_rectangle(0,0,0,5, fill=ACCENT, outline="")
        self._pred_bar_c = pred_bc

        return frame

    # ════════════════════════════════════════
    #  CONNECTION
    # ════════════════════════════════════════
    def _set_sim(self, sim):
        self._sim_mode = sim
        self._sim_btn.configure(bg=ACCENT if sim else PANEL2,
                                 fg=BG    if sim else MUTED)
        self._hw_btn.configure( bg=ACCENT if not sim else PANEL2,
                                 fg=BG    if not sim else MUTED)

    # ════════════════════════════════════════
    #  DEVICE SELECTION
    # ════════════════════════════════════════
    def _select_device(self, device):
        """Switch between 'umyo' and 'mindrove'. Resets dataset and model."""
        global ACTIVE_DEVICE, FEATURE_NAMES, _latest_emg

        if STATE.connected_emg:
            self._log("Disconnect current device before switching.", "err")
            return

        ACTIVE_DEVICE = device

        # Update button highlights
        self._dev_umyo_btn.configure(
            bg=ACCENT if device == "umyo" else PANEL2,
            fg=BG     if device == "umyo" else MUTED)
        self._dev_mr_btn.configure(
            bg=PURPLE if device == "mindrove" else PANEL2,
            fg=BG     if device == "mindrove" else MUTED)

        # Show/hide the correct connect button
        if device == "umyo":
            self._mr_btn.pack_forget()
            self._emg_btn.pack(fill=tk.X, pady=(6,4))
            # Show sim/hw toggle (uMyo only)
            info = (f"uMyo: {NUM_SENSORS} sensor(s) × "
                    f"(1 act + {BINS_PER_SENSOR} bins) = "
                    f"{NUM_SENSORS*(BINS_PER_SENSOR+1)} raw inputs → "
                    f"{len(FEATURE_NAMES)} features")
        else:
            self._emg_btn.pack_forget()
            self._mr_btn.pack(fill=tk.X, pady=(6,4))
            info = (f"MindRove: {MR_N_CHANNELS} ch × "
                    f"{len(MR_FEATURES_PER_CH)} features "
                    f"(RMS,MAV,WL,ZC,SSC,VAR,MNF,MDF,4×BP) "
                    f"+ 3 cross-ch = {len(MR_FEATURE_NAMES)} total")

        self._feat_info_lbl.configure(text=info)

        # Reset data and model (feature vectors change shape)
        STATE.dataset.clear()
        STATE.level_window_counts = {lvl: 0 for lvl in FORCE_LEVELS}
        STATE.win_buf.clear()
        STATE.win_force_buf.clear()
        STATE.model         = None
        STATE.scaler        = None
        STATE.model_trained = False
        STATE.inferring     = False
        STATE.val_pred.clear()
        STATE.val_true.clear()

        self._log(f"Device set to: {device.upper()}  ({info})", "info")
        self._refresh_live_channel_labels()

        # Update collect chart titles
        if device == "mindrove":
            self._ax_s0.set_title("CH 0–2 RMS", color=MUTED, fontsize=8, loc="left", pad=2)
            self._ax_s1.set_title("CH 3–5 RMS", color=MUTED, fontsize=8, loc="left", pad=2)
            self._ax_s2.set_title("CH 6–7 RMS", color=MUTED, fontsize=8, loc="left", pad=2)
            self._ax_s2.set_visible(True)
        else:
            self._ax_s0.set_title("SENSOR 0 — Activity", color=MUTED, fontsize=8, loc="left", pad=2)
            self._ax_s1.set_title("SENSOR 1 — Activity", color=MUTED, fontsize=8, loc="left", pad=2)
            self._ax_s2.set_title("SENSOR 2 — Activity", color=MUTED, fontsize=8, loc="left", pad=2)

    def _toggle_mindrove(self):
        """Connect or disconnect the MindRove board."""
        if STATE.connected_emg:
            # Disconnect
            self._stop_event.set()
            disconnect_mindrove()
            STATE.connected_emg = False
            self._mr_btn.configure(text="⬡  CONNECT MINDROVE", fg=PURPLE)
            self._set_dot(self._conn_dot, MUTED)
            self._conn_lbl.configure(text="EMG: DISC")
            self._log("MindRove disconnected.", "warn")
            return

        # Connect
        if self._sim_mode:
            # Simulate MindRove with 8-channel fake data
            self._stop_event.clear()
            self._sim_thread = threading.Thread(
                target=self._sim_loop_mindrove, daemon=True)
            self._sim_thread.start()
            STATE.connected_emg = True
            self._mr_btn.configure(text="⬡  DISCONNECT", fg=RED)
            self._set_dot(self._conn_dot, YELLOW)
            self._conn_lbl.configure(text="EMG: SIM (MR)")
            return

        self._conn_status.configure(text="Connecting to MindRove…")
        self.update()

        def _do():
            ok, msg = connect_mindrove()
            def _finish():
                if ok:
                    self._stop_event.clear()
                    threading.Thread(
                        target=_mindrove_reader_thread,
                        args=(self._stop_event, STATE.data_queue),
                        daemon=True).start()
                    STATE.connected_emg = True
                    self._mr_btn.configure(text="⬡  DISCONNECT", fg=RED)
                    self._set_dot(self._conn_dot, GREEN)
                    self._conn_lbl.configure(text="EMG: MindRove")
                    self._conn_status.configure(
                        text=f"MindRove @ {MR_SAMPLING_RATE} Hz")
                    self._log(f"MindRove connected. SR={MR_SAMPLING_RATE} Hz  "
                              f"Channels={MR_N_CHANNELS}  "
                              f"Features={len(MR_FEATURE_NAMES)}", "ok")
                else:
                    self._conn_status.configure(text=f"MindRove failed: {msg[:60]}")
                    self._set_dot(self._conn_dot, RED)
                    self._log(f"MindRove connect failed: {msg}", "err")
            self.after(0, _finish)

        threading.Thread(target=_do, daemon=True).start()

    def _refresh_ports(self):
        ports = list_ports()
        self._port_cb["values"] = ports
        if ports: self._port_var.set(ports[0])

    def _toggle_emg(self):
        if STATE.connected_emg:
            self._stop_event.set()
            if emg_serial and emg_serial.is_open:
                try: emg_serial.close()
                except: pass
            STATE.connected_emg = False
            self._emg_btn.configure(text="⬡  CONNECT EMG", fg=ACCENT)
            self._set_dot(self._conn_dot, MUTED)
            self._conn_lbl.configure(text="EMG: DISC")
            return

        if self._sim_mode:
            self._stop_event.clear()
            self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
            self._sim_thread.start()
            STATE.connected_emg = True
            self._emg_btn.configure(text="⬡  DISCONNECT", fg=RED)
            self._set_dot(self._conn_dot, YELLOW)
            self._conn_lbl.configure(text="EMG: SIM")
            self._conn_status.configure(text="Simulation active")
        else:
            port = self._port_var.get()
            if not port:
                self._conn_status.configure(text="No port selected"); return
            self._conn_status.configure(text="Connecting… (2s boot wait)")
            self.update()
            if open_emg_serial(port):
                self._stop_event.clear()
                threading.Thread(target=self._serial_reader, daemon=True).start()
                STATE.connected_emg = True
                self._emg_btn.configure(text="⬡  DISCONNECT", fg=RED)
                self._set_dot(self._conn_dot, GREEN)
                self._conn_lbl.configure(text="EMG: LIVE")
                self._conn_status.configure(text=f"← {port}")
                self._log(f"EMG connected: {port}", "ok")
            else:
                self._conn_status.configure(text="Connection failed")
                self._log(f"EMG connect failed on {port}", "err")

    def _on_force_src_change(self, _event=None):
        src = self._force_src_var.get()
        if src == "LSL Stream":
            self._gdx_btn.configure(text="⬡  SCAN & CONNECT LSL")
            self._lsl_pick_frame.pack(fill=tk.X, pady=(2, 0))
            self._refresh_lsl_streams()
        else:
            self._gdx_btn.configure(text="⬡  CONNECT GDX")
            self._lsl_pick_frame.pack_forget()

    def _refresh_lsl_streams(self):
        self._lsl_stream_cb.configure(values=["Scanning…"], state="disabled")
        self._lsl_stream_var.set("Scanning…")
        self.update()

        def _scan():
            streams = get_lsl_streams()   # list of (name, type, n_ch)
            labels = [f"{n}  [{t}]" for n, t, _ in streams] if streams else ["(none found)"]
            self._lsl_names = [n for n, _, _ in streams]  # raw names for connect

            def _update():
                self._lsl_stream_cb.configure(values=labels, state="readonly")
                if labels:
                    self._lsl_stream_var.set(labels[0])
            self.after(0, _update)

        threading.Thread(target=_scan, daemon=True).start()

    def _connect_force(self):
        src = self._force_src_var.get()
        if src == "GDX (USB)":
            self._connect_gdx()
        else:
            self._connect_lsl()

    def _connect_gdx(self):
        self._gdx_btn.configure(text="Connecting…", state=tk.DISABLED)
        self.update()

        def _do_connect():
            ok, msg = setup_gdx()
            def _finish():
                if ok:
                    STATE.connected_gdx = True
                    self._set_dot(self._gdx_dot, GREEN)
                    self._gdx_lbl.configure(text="GDX: LIVE")
                    self._gdx_btn.configure(text="✔  GDX CONNECTED", fg=GREEN, state=tk.NORMAL)
                    self._log("GDX connected.", "ok")
                    # setup_gdx() opens both scales if both are plugged in;
                    # refresh the robot tab's status display to match.
                    self._robot_refresh_gdx_status()
                else:
                    self._set_dot(self._gdx_dot, RED)
                    self._gdx_btn.configure(text="⬡  CONNECT GDX", state=tk.NORMAL)
                    self._log(f"GDX failed: {msg}", "err")
            self.after(0, _finish)

        threading.Thread(target=_do_connect, daemon=True).start()

    def _connect_lsl(self):
        idx = self._lsl_stream_cb.current()
        names = getattr(self, "_lsl_names", [])
        if not names or idx < 0 or idx >= len(names):
            self._log("No LSL stream selected.", "err")
            return
        stream_name = names[idx]
        self._gdx_btn.configure(text="Connecting…", state=tk.DISABLED)
        self.update()

        def _do_connect():
            ok, msg = connect_lsl_force(stream_name)
            def _finish():
                if ok:
                    STATE.connected_gdx = True
                    self._set_dot(self._gdx_dot, GREEN)
                    self._gdx_lbl.configure(text="LSL: LIVE")
                    self._gdx_btn.configure(
                        text=f"✔  LSL: {stream_name}", fg=GREEN, state=tk.NORMAL)
                    self._log(f"LSL force stream connected: {stream_name}", "ok")
                else:
                    self._set_dot(self._gdx_dot, RED)
                    self._gdx_btn.configure(text="⬡  SCAN & CONNECT LSL", state=tk.NORMAL)
                    self._log(f"LSL connect failed: {msg}", "err")
            self.after(0, _finish)

        threading.Thread(target=_do_connect, daemon=True).start()

    def _set_dot(self, dot, color):
        dot.delete("all")
        dot.create_oval(1,1,7,7, fill=color, outline="")

    # ════════════════════════════════════════
    #  SIMULATION THREAD
    # ════════════════════════════════════════
    def _sim_loop(self):
        while not self._stop_event.is_set():
            emg = SIM.step(STATE.current_level)
            force = STATE.current_level * 0.5 + random.gauss(0, 1.5)
            force = max(0, force)
            try:
                STATE.data_queue.put_nowait({"emg": emg, "force": force})
            except queue.Full:
                pass
            time.sleep(0.05)

    def _sim_loop_mindrove(self):
        """Simulate 8-channel MindRove EMG data for testing without hardware."""
        t = 0.0
        while not self._stop_event.is_set():
            t += 0.002   # 500 Hz
            activation = STATE.current_level / 100.0
            sample = []
            for ch in range(MR_N_CHANNELS):
                # Simulate muscle activation + spatial gradient across channels
                spatial = 0.7 + ch * 0.04
                sig = (activation * spatial * 2000 *
                       (0.8 + 0.2 * math.sin(2 * math.pi * 120 * t)) +
                       random.gauss(0, 50 + activation * 200))
                sample.append(sig)
            force = STATE.current_level * 0.5 + random.gauss(0, 1.5)
            force = max(0, force)
            try:
                STATE.data_queue.put_nowait({"emg": sample, "force": force})
            except queue.Full:
                pass
            time.sleep(0.002)

    def _serial_reader(self):
        """Read EMG from serial port and push into the queue — same path as sim."""
        _rx_total   = 0   # total lines received
        _rx_ok      = 0   # lines parsed as EMG
        _rx_skip    = 0   # lines skipped (status/debug)
        _rx_fail    = 0   # lines that failed to parse

        while not self._stop_event.is_set():
            try:
                raw = emg_serial.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    continue

                _rx_total += 1

                # ── Status / debug lines from Arduino ─────────────────────
                if (raw.startswith("#")        or
                    raw.startswith("dev_count") or
                    raw.startswith("Invalid")   or
                    raw.startswith("Scanning")  or
                    raw.startswith("COUNT:")):
                    _rx_skip += 1
                    self.after(0, lambda l=raw:
                        self._conn_status.configure(text="← " + l[:60]))
                    continue

                # ── Try to parse as EMG data ───────────────────────────────
                emg, n = _parse_emg_line(raw)
                if emg is not None:
                    _rx_ok += 1
                    if n != self._sensor_count_var.get():
                        self.after(0, lambda v=n: self._sync_sensor_ui(v))
                    try:
                        STATE.data_queue.put_nowait({
                            "emg":   emg,
                            "force": _gdx_last_force[0],
                        })
                    except queue.Full:
                        pass
                    # Update status every 50 good packets
                    if _rx_ok % 50 == 0:
                        self.after(0, lambda ok=_rx_ok, f=_rx_fail, s=n:
                            self._conn_status.configure(
                                text=f"← RX {ok} pkts | {s} sensor(s) | {f} parse errors"))
                else:
                    _rx_fail += 1
                    # Show first few failures so we can diagnose format issues
                    if _rx_fail <= 5:
                        print(f"[EMG PARSE FAIL #{_rx_fail}] raw={raw[:80]!r}")
                        self.after(0, lambda l=raw, f=_rx_fail:
                            self._conn_status.configure(
                                text=f"← parse error #{f}: {l[:50]}"))

            except Exception as e:
                if not self._stop_event.is_set():
                    print(f"[EMG READER ERR] {e}")
                    time.sleep(0.05)

    # ════════════════════════════════════════
    #  DATA CONSUMER  (main thread)
    # ════════════════════════════════════════
    def _start_consumer(self):
        self._consume()
        self._refresh_force_only()

    def _refresh_force_only(self):
        """Refresh the GDX force display even if no EMG packets are arriving yet."""
        force = _gdx_last_force[0]
        try:
            force_pct = (force / (force_mvc_reference + 1e-9) * 100.0) if force_mvc_reference else None
            STATE.live_force_pct = float(force_pct) if force_pct is not None else 0.0
            if force_pct is not None:
                self._force_val.configure(text=f"{force_pct:.1f}% MVC")
                self._force_sub.configure(text=f"({force:.2f} N)")
            else:
                self._force_val.configure(text=f"{force:.2f} N")
                self._force_sub.configure(text="(calibrate MVC)")
        except Exception:
            pass
        self.after(DISPLAY_UPDATE_MS, self._refresh_force_only)

    def _consume(self):
        n = 0
        max_items = 200 if ACTIVE_DEVICE == "mindrove" else 50
        while n < max_items:
            try:
                d = STATE.data_queue.get_nowait()
                self._ingest(d["emg"], d.get("force", _gdx_last_force[0]))
                n += 1
            except queue.Empty:
                # Queue empty — in hardware mode pull directly from the
                # background threads so the display always updates.
                if STATE.connected_emg and not self._sim_mode:
                    live_emg = get_mindrove_emg() if ACTIVE_DEVICE == "mindrove" else get_emg()
                    self._ingest(live_emg, _gdx_last_force[0])
                break
        self.after(20 if ACTIVE_DEVICE == "mindrove" else 50, self._consume)

    def _ingest(self, emg, force):
        now = time.time()
        do_live_ui = (now - self._last_live_ui_ts) * 1000.0 >= DISPLAY_UPDATE_MS
        do_force_ui = (now - self._last_force_ui_ts) * 1000.0 >= DISPLAY_UPDATE_MS
        # Always keep live_force_pct current for the ramp overlay.
        # In hardware mode: normalize raw force by MVC reference.
        # In sim mode (no force_mvc_reference): use current_level directly as % MVC.
        if force_mvc_reference:
            STATE.live_force_pct = min(130.0, float(force / (force_mvc_reference + 1e-9) * 100.0))
        else:
            STATE.live_force_pct = float(STATE.current_level)
        # ── Live display history ──────────────────────────────────────────────
        if ACTIVE_DEVICE == "mindrove":
            # emg is a list of 8 raw µV values (one per channel)
            emg_arr  = np.array(emg, dtype=float)
            global mr_session_baseline
            if not STATE.recording and not STATE.inferring and len(emg_arr) == MR_N_CHANNELS:
                mr_session_baseline = (
                    (1.0 - MR_BASELINE_ALPHA) * mr_session_baseline
                    + MR_BASELINE_ALPHA * emg_arr
                )

            # Group channels into three bundles for the compact center plot.
            # All 8 individual channels are still shown numerically on the right.
            bundle0 = emg_arr[0:3]
            bundle1 = emg_arr[3:6]
            bundle2 = emg_arr[6:8]
            act0 = float(np.sqrt(np.mean(bundle0 ** 2))) if len(bundle0) else 0.0
            act1 = float(np.sqrt(np.mean(bundle1 ** 2))) if len(bundle1) else 0.0
            act2 = float(np.sqrt(np.mean(bundle2 ** 2))) if len(bundle2) else 0.0

            mvc_scale = (mvc_reference[0] + 1e-9) if mvc_reference is not None else 5000.0
            STATE.act0_hist.append(min(1.0, act0 / mvc_scale))
            STATE.act1_hist.append(min(1.0, act1 / mvc_scale))
            STATE.act2_hist.append(min(1.0, act2 / mvc_scale))
            STATE.force_hist.append(_force_to_pct_mvc(force))
            STATE.sample_count += 1

            # Right panel: show all 8 raw MindRove channels live.
            if do_live_ui:
                for i in range(MR_N_CHANNELS):
                    raw_val = float(emg_arr[i])
                    raw_abs = abs(raw_val)
                    norm    = min(1.0, raw_abs / mvc_scale)
                    self._sensor_vals[i].configure(text=f"{raw_val:.1f} µV")
                    bc, bar = self._sensor_bars[i]
                    w = max(2, bc.winfo_width())
                    bc.coords(bar, 0, 0, max(2, int(norm * w)), 3)
                self._last_live_ui_ts = now
        else:
            # Original uMyo path ──────────────────────────────────────────────
            stride = BINS_PER_SENSOR + 1
            act0 = emg[0]
            act1 = emg[stride]          if NUM_SENSORS > 1 else act0
            act2 = emg[stride * 2]      if NUM_SENSORS > 2 else act0
            mvc0 = mvc_reference[0]          if mvc_reference is not None else None
            mvc1 = mvc_reference[stride]     if (mvc_reference is not None and NUM_SENSORS > 1) else mvc0
            mvc2 = mvc_reference[stride * 2] if (mvc_reference is not None and NUM_SENSORS > 2) else mvc0
            STATE.act0_hist.append(act0 if mvc0 is None else min(1.0, act0 / (mvc0 + 1e-9)))
            STATE.act1_hist.append(act1 if mvc1 is None else min(1.0, act1 / (mvc1 + 1e-9)))
            STATE.act2_hist.append(act2 if mvc2 is None else min(1.0, act2 / (mvc2 + 1e-9)))
            STATE.force_hist.append(_force_to_pct_mvc(force))
            STATE.sample_count += 1

            acts = ([act0] if NUM_SENSORS == 1
                    else [act0, act1] if NUM_SENSORS == 2
                    else [act0, act1, act2])
            if do_live_ui:
                for i, act in enumerate(acts):
                    mvc_i = mvc_reference[i * stride] if mvc_reference is not None else None
                    norm = min(1.0, act / (mvc_i + 1e-9)) if mvc_i is not None else min(1.0, act / 500)
                    if i in self._sensor_vals:
                        self._sensor_vals[i].configure(text=f"{act:.3f}")
                        bc, bar = self._sensor_bars[i]
                        w = max(2, bc.winfo_width())
                        bc.coords(bar, 0, 0, max(2, int(norm * w)), 3)

                # Grey out unused cards when not in MindRove mode.
                for i in range(len(acts), 8):
                    if i in self._sensor_vals:
                        self._sensor_vals[i].configure(text="—")
                        bc, bar = self._sensor_bars[i]
                        bc.coords(bar, 0, 0, 0, 3)
                self._last_live_ui_ts = now

        # Update force display
        if do_force_ui:
            force_pct = (force / (force_mvc_reference + 1e-9) * 100.0) if force_mvc_reference else None
            if force_pct is not None:
                self._force_val.configure(text=f"{force_pct:.1f}% MVC")
                self._force_sub.configure(text=f"({force:.2f} N)")
            else:
                self._force_val.configure(text=f"{force:.2f} N")
                self._force_sub.configure(text="(calibrate MVC)")
            w = self._force_bar_c.winfo_width()
            max_f = 60.0
            self._force_bar_c.coords(self._force_bar, 0, 0,
                                      max(2, int((force/max_f)*w)), 5)
            self._last_force_ui_ts = now

        # ── Window buffer and feature extraction ─────────────────────────────
        STATE.win_buf.append(list(emg))
        STATE.win_force_buf.append(force)

        # MindRove needs more samples per window (raw signal, not pre-binned)
        if ACTIVE_DEVICE == "mindrove":
            win_size = MR_WINDOW_SAMPLES
        else:
            win_size = int(self._win_var.get() or 10)

        if len(STATE.win_buf) >= win_size:
            feats = extract_features(STATE.win_buf[-win_size:])
            if feats is not None:
                if (now - self._last_snapshot_ts) * 1000.0 >= SNAPSHOT_UPDATE_MS:
                    self._update_snapshot(feats)
                    self._last_snapshot_ts = now

                # Recording mode → add to dataset
                if STATE.recording and STATE.win_label is not None:
                    mean_force = float(np.mean(STATE.win_force_buf[-win_size:]))
                    mean_force_pct = (
                        float(mean_force / (force_mvc_reference + 1e-9) * 100.0)
                        if force_mvc_reference is not None else None
                    )
                    STATE.dataset.append({
                        "features": feats,
                        "force":    mean_force,
                        "force_pct_mvc": mean_force_pct,
                        "level":    int(round(STATE.win_label)),
                        "phase":    STATE.current_phase_type,
                        "group":    STATE.protocol_run_id * 1000 + STATE.protocol_idx
                    })
                    self._on_window_added()

                # Inference mode → predict and display
                if STATE.inferring and STATE.model_trained:
                    self._run_inference(feats, force)

            # Slide window with 50% overlap
            keep = win_size // 2
            STATE.win_buf       = STATE.win_buf[-keep:]
            STATE.win_force_buf = STATE.win_force_buf[-keep:]

    def _update_snapshot(self, feats):
        names = MR_FEATURE_NAMES if ACTIVE_DEVICE == "mindrove" else FEATURE_NAMES
        feat_dict = dict(zip(names, feats))
        for key, widget in self._snap_vals.items():
            v = feat_dict.get(key, 0)
            widget.configure(text=f"{v:.4f}")

    def _on_window_added(self):
        n = len(STATE.dataset)
        self._n_lbl.configure(text=f"{n} WINDOWS")
        self._wins_lbl.configure(text=str(n))
        self._n_lbl.configure(text=f"{n} WINDOWS")
        last_level = STATE.dataset[-1]["level"] if STATE.dataset else None
        if last_level in STATE.level_window_counts:
            STATE.level_window_counts[last_level] += 1
            self._level_counts[last_level].set(str(STATE.level_window_counts[last_level]))

    # ════════════════════════════════════════
    #  MVC CALIBRATION
    # ════════════════════════════════════════
    def _start_mvc(self):
        if not STATE.connected_emg:
            self._log("Connect EMG first.", "err"); return
        self._mvc_btn.configure(state=tk.DISABLED)
        self._cue_label.configure(text="MAX SQUEEZE", fg=RED)
        self._cue_sub.configure(text=f"Hold maximum grip for {MVC_DURATION:.0f}s")
        self._log(f"MVC calibration starting — squeeze as hard as possible!", "info")
        threading.Thread(target=self._mvc_thread, daemon=True).start()

    def _mvc_thread(self):
        global mvc_reference, force_mvc_reference
        if ACTIVE_DEVICE == "mindrove":
            n_vals = MR_N_CHANNELS
        else:
            n_vals = NUM_SENSORS * (BINS_PER_SENSOR + 1)
        peak = np.zeros(n_vals)
        peak_force = 0.0
        t0   = time.time()
        _last_emg = [0.0] * n_vals
        while time.time() - t0 < MVC_DURATION:
            # Drain queue to get latest EMG sample (don't block, just peek)
            if self._sim_mode:
                emg = SIM.step(100)
                curr_force = max(0.0, 0.5 * 100 + random.gauss(0, 1.0))
            else:
                curr_force = _gdx_last_force[0]
                try:
                    d = STATE.data_queue.get_nowait()
                    _last_emg = d["emg"]
                    curr_force = d.get("force", curr_force)
                    # Put it back so _consume still sees it
                    try: STATE.data_queue.put_nowait(d)
                    except queue.Full: pass
                except queue.Empty:
                    pass
                emg = _last_emg
            for j, v in enumerate(emg):
                if v > peak[j]:
                    peak[j] = v
            if curr_force > peak_force:
                peak_force = float(curr_force)
            elapsed = time.time() - t0
            pct = elapsed / MVC_DURATION
            self.after(0, lambda p=pct: (
                self._prog_canvas.coords(
                    self._prog_fill, 0, 0,
                    int(p * self._prog_canvas.winfo_width()), 4)
            ))
            time.sleep(0.03)

        mvc_reference = peak
        force_mvc_reference = peak_force if peak_force > 1e-6 else None
        self.after(0, self._mvc_done)

    def _mvc_done(self):
        if ACTIVE_DEVICE == "mindrove":
            summary = " | ".join([f"CH{ch}:{mvc_reference[ch]:.0f}µV"
                                   for ch in range(MR_N_CHANNELS)])
        else:
            summary = " | ".join([f"S{s}:{mvc_reference[s*(BINS_PER_SENSOR+1)]:.1f}"
                                   for s in range(NUM_SENSORS)])
        self._mvc_status.configure(text=f"Calibrated ✔\n{summary}\nForce MVC: {(force_mvc_reference if force_mvc_reference is not None else 0):.2f} N", fg=GREEN)
        self._mvc_lbl.configure(text="CALIB ✔")
        self._cue_label.configure(text="READY", fg=ACCENT)
        self._cue_sub.configure(text="MVC calibrated — start protocol when ready")
        self._log(f"MVC done. Peaks: {summary} | Force MVC: {(force_mvc_reference if force_mvc_reference is not None else 0):.2f} N", "ok")
        self._mvc_btn.configure(state=tk.NORMAL)
        self._start_btn.configure(state=tk.NORMAL)
        self._ramp_btn.configure(state=tk.NORMAL)
        self._prog_canvas.coords(self._prog_fill, 0,0,0,4)
        # Keep robot tab MVC field in sync so it shows the calibrated reference.
        if force_mvc_reference and hasattr(self, "_robot_mvc_force_var"):
            self._robot_mvc_force_var.set(f"{force_mvc_reference:.2f}")

    # ════════════════════════════════════════
    #  RAMP OVERLAY GRAPH
    # ════════════════════════════════════════
    # Overlay constants
    _OVERLAY_PLAYHEAD_X = 0.35   # unused now — kept for reference only

    def _build_ramp_overlay_canvas(self, parent):
        """
        Idle view  : overview of full protocol (ramps + holds) on fixed time axis.
        Active view: same fixed axis, playhead moves left→right.
        """
        self._add_popout_button(parent, "collect")

        # ── Pre-compute effort path from the full protocol sequence ───────────
        # Ramp/effort-graph trials only — no discrete hold-level plateaus are
        # recorded (see _build_full_protocol_seq), so this guide only needs
        # to trace the ramp shape.
        reps      = int(self._reps_var.get() if hasattr(self, "_reps_var") else NUM_REPS)
        rest_s    = float(self._rest_var.get() if hasattr(self, "_rest_var") else REST_SEC)
        ramp_half = RAMP_SEC / 2.0

        # Build waypoints by walking the same logic as _build_ramp_seq
        t_path, y_path = [0.0], [0.0]
        t_cursor = 0.0

        # ── ramp up / ramp down, one per rep, peak height varies per RAMP_PEAKS ─
        ramp_labels = []   # (t_centre, label_str)
        for i in range(reps):
            peak = RAMP_PEAKS[i % len(RAMP_PEAKS)]
            t_cursor += rest_s;    t_path.append(t_cursor); y_path.append(0.0)
            t_cursor += ramp_half; t_path.append(t_cursor); y_path.append(float(peak))
            ramp_labels.append((t_cursor - ramp_half / 2, f"Ramp {i+1} ({peak}%)"))
            t_cursor += RAMP_PEAK_HOLD_SEC; t_path.append(t_cursor); y_path.append(float(peak))
            t_cursor += ramp_half; t_path.append(t_cursor); y_path.append(0.0)
            t_cursor += rest_s;    t_path.append(t_cursor); y_path.append(0.0)

        hold_label_coords = []   # no hold plateaus recorded anymore

        self._overlay_total_t   = t_cursor
        self._overlay_t_path    = t_path
        self._overlay_y_path    = y_path
        self._overlay_reps      = reps
        self._overlay_rest_s    = rest_s
        self._overlay_ramp_half = ramp_half
        self._overlay_live_t    = []
        self._overlay_live_y    = []
        self._overlay_active    = False

        # ── Figure & axes ─────────────────────────────────────────────────────
        fig = Figure(figsize=(6, 3.0), facecolor=PANEL)
        fig.subplots_adjust(top=0.82, bottom=0.28, left=0.09, right=0.94)
        ax  = fig.add_subplot(111)
        self._overlay_ax  = ax
        self._overlay_fig = fig
        self._overlay_style_ax(ax)

        # ── Reference path (red) ──────────────────────────────────────────────
        self._overlay_ref_line, = ax.plot(
            t_path, y_path, color=RED, lw=3.5, zorder=3,
            label="Target effort", solid_capstyle="round", solid_joinstyle="round")

        # Horizontal guide lines at each hold level + 100%
        for lvl in [25, 50, 75, 100]:
            ax.axhline(lvl, color=MUTED, lw=0.5, linestyle=":", zorder=1, alpha=0.5)

        # 100% MVC label — inside plot, left
        self._overlay_mvc_txt = ax.text(
            0.01, 102, "100% MVC", color=MUTED, fontsize=8,
            transform=ax.get_yaxis_transform(), va="bottom", ha="left")

        # ── Segment labels ────────────────────────────────────────────────────
        self._overlay_rep_texts = []

        # Ramp labels centred at peak of each triangle
        for t_c, lbl in ramp_labels:
            tx = ax.text(t_c, 104, lbl, color=MUTED, fontsize=8,
                         ha="center", va="bottom", fontweight="bold", clip_on=False)
            self._overlay_rep_texts.append(tx)

        # Hold level labels centred inside each flat section
        for t_c, lbl, lvl in hold_label_coords:
            tx = ax.text(t_c, lvl + 3, lbl, color=MUTED, fontsize=8,
                         ha="center", va="bottom", fontweight="bold", clip_on=False)
            self._overlay_rep_texts.append(tx)

        # ── Live trace & playhead ─────────────────────────────────────────────
        self._overlay_live_line, = ax.plot(
            [], [], color=YELLOW, lw=2.2, zorder=4,
            label="Your effort", alpha=0.9, solid_capstyle="round")

        self._overlay_live_dot, = ax.plot(
            [], [], 'o', color=YELLOW, ms=7, zorder=6, alpha=1.0)

        self._overlay_playhead = ax.axvline(
            x=-1, color=ACCENT, lw=2.0, linestyle="--", zorder=5, alpha=0.8)

        self._overlay_now_span_lines = []

        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.28),
                  ncol=2, fontsize=10, labelcolor=TEXT,
                  facecolor=PANEL2, edgecolor=BORDER, framealpha=0.85,
                  borderpad=0.8, handlelength=2.0, columnspacing=2.0)

        self._overlay_set_idle_view()

        cv = FigureCanvasTkAgg(fig, master=parent)
        cv.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=0, pady=(6, 0))
        self._overlay_canvas = cv

    def _build_val_overlay_canvas(self, parent):
        """
        Validation overlay — identical structure to _build_ramp_overlay_canvas:
          • Fixed x-axis (full protocol duration, never moves)
          • Red solid guide path in Newtons (converted from % MVC via MVC ref)
          • Segment labels (Ramp 1/2/3, 25%/50%/75%/100%) same positions
          • Cyan line + dot  = model predicted force streaming left→right
          • Green line       = ground truth GDX force streaming left→right
          • Cyan dashed playhead + now-band driven by _val_poll at 100 ms
        Built at tab-construction time; guide data populated when START pressed.
        """
        self._add_popout_button(parent, "validate")

        # ── Reuse the pre-computed path from the collect overlay ──────────────
        # _overlay_t_path/_overlay_y_path are in % MVC; convert to Newtons
        # lazily in _toggle_validate once MVC is calibrated.

        # ── Figure — same margins as collect overlay ──────────────────────────
        fig = Figure(figsize=(7, 3.0), facecolor=PANEL)
        fig.subplots_adjust(top=0.82, bottom=0.28, left=0.09, right=0.94)
        ax  = fig.add_subplot(111)
        self._val_fig = fig
        self._val_ax  = ax

        # ── Axes styling — same as _overlay_style_ax ─────────────────────────
        ax.set_facecolor(PANEL2)
        for sp in ax.spines.values():
            sp.set_color(BORDER); sp.set_linewidth(0.6)
        ax.tick_params(colors=MUTED, labelsize=11)
        ax.set_xlabel("Time (s)", color=MUTED, fontsize=11, labelpad=4)
        ax.set_ylabel("% MVC", color=MUTED, fontsize=11, labelpad=6)
        ax.set_xlim(0, 1); ax.set_ylim(0, 130)   # updated on START

        # ── Red guide path (drawn at START) ───────────────────────────────────
        self._val_guide_line, = ax.plot(
            [], [], color=RED, lw=3.5, zorder=3,
            label="Target", solid_capstyle="round", solid_joinstyle="round")

        # Horizontal dotted lines at 25/50/75/100% of MVC force (drawn at START)
        self._val_h_guides = []   # axhline objects, cleared and redrawn each START

        # Segment labels (redrawn at START once MVC force is known)
        self._val_seg_texts = []

        # ── Green ground truth (GDX grip force, streams left→right) ──────────
        self._val_true_line, = ax.plot(
            [], [], color=GREEN, lw=2.2, zorder=4,
            label="Ground truth", alpha=0.9, solid_capstyle="round")

        # ── Cyan predicted force (model output, streams left→right) ──────────
        self._val_pred_line, = ax.plot(
            [], [], color=ACCENT, lw=2.2, zorder=5,
            label="Predicted", alpha=0.9, solid_capstyle="round")

        # Green dot at leading edge of ground truth (easier to track than prediction)
        self._val_live_dot, = ax.plot(
            [], [], 'o', color=GREEN, ms=8, zorder=7, alpha=1.0)

        # ── Playhead + now-band (same pattern as collect overlay) ─────────────
        self._val_playhead = ax.axvline(
            x=-1, color=ACCENT, lw=2.0, linestyle="--", zorder=6, alpha=0.8)
        self._val_now_span_lines = []

        # Legend — same style as collect overlay, placed below x-axis
        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.28),
                  ncol=3, fontsize=10, labelcolor=TEXT,
                  facecolor=PANEL2, edgecolor=BORDER, framealpha=0.85,
                  borderpad=0.8, handlelength=2.0, columnspacing=1.5)

        ax.set_title("VALIDATION GUIDE  —  press START VALIDATION to begin",
                     color=MUTED, fontsize=10, loc="left", pad=10)

        self._val_canvas = FigureCanvasTkAgg(fig, master=parent)
        self._val_canvas.get_tk_widget().pack(fill=tk.X)

    def _val_overlay_draw_guide(self, dur):
        """Draw the red guide path + segment labels + h-lines in % MVC.
        Called once at START VALIDATION after MVC is confirmed calibrated.
        _overlay_y_path is already in % MVC, so no Newton conversion needed.
        """
        ax = self._val_ax

        for ln in self._val_h_guides:
            try: ln.remove()
            except Exception: pass
        self._val_h_guides.clear()
        for tx in self._val_seg_texts:
            try: tx.remove()
            except Exception: pass
        self._val_seg_texts.clear()

        if not hasattr(self, "_overlay_t_path"):
            return
        gt = np.array(self._overlay_t_path)
        gf = np.array(self._overlay_y_path)  # already % MVC
        if not len(gt):
            return

        # Red guide path
        self._val_guide_line.set_data(gt, gf)

        # Horizontal dotted lines at 25/50/75/100% MVC
        for pct in [25, 50, 75, 100]:
            ln = ax.axhline(pct, color=MUTED, lw=0.5, linestyle=":", zorder=1, alpha=0.5)
            self._val_h_guides.append(ln)

        tx = ax.text(0.01, 102, "100% MVC", color=MUTED, fontsize=8,
                     transform=ax.get_yaxis_transform(), va="bottom", ha="left")
        self._val_seg_texts.append(tx)

        reps      = getattr(self, "_overlay_reps",      3)
        rest_s    = getattr(self, "_overlay_rest_s",    REST_SEC)
        ramp_half = getattr(self, "_overlay_ramp_half", RAMP_SEC / 2.0)

        t_c = 0.0
        for i in range(reps):
            t_c += rest_s
            ramp_peak = t_c + ramp_half
            tx = ax.text(ramp_peak, 104, f"Ramp {i+1}",
                         color=MUTED, fontsize=8, ha="center", va="bottom",
                         fontweight="bold", clip_on=False)
            self._val_seg_texts.append(tx)
            t_c += ramp_half + RAMP_PEAK_HOLD_SEC + ramp_half
            t_c += rest_s

        ax.set_xlim(0, dur)
        ax.set_ylim(0, 135)
        ax.set_title("VALIDATION — follow the red guide  |  recording…",
                     color=YELLOW, fontsize=10, loc="left", pad=10, fontweight="bold")

    def _overlay_style_ax(self, ax):
        """Apply dark-theme styling to the overlay axes."""
        ax.set_facecolor(PANEL2)
        for sp in ax.spines.values():
            sp.set_color(BORDER); sp.set_linewidth(0.6)
        ax.tick_params(colors=MUTED, labelsize=11)
        ax.set_xlabel("Time (s)", color=MUTED, fontsize=11, labelpad=4)
        ax.set_ylabel("% MVC",    color=MUTED, fontsize=11, labelpad=6)
        ax.set_ylim(-5, 112)

    def _build_perf_overlay_canvas(self, parent):
        """Performance chart — identical structure to validation overlay."""
        popout_bar = self._add_popout_button(parent, "performance")

        fig = Figure(figsize=(7, 3.0), facecolor=PANEL)
        fig.subplots_adjust(top=0.82, bottom=0.28, left=0.09, right=0.94)
        ax  = fig.add_subplot(111)
        self._perf_fig = fig
        self._perf_ax  = ax

        ax.set_facecolor(PANEL2)
        for sp in ax.spines.values():
            sp.set_color(BORDER); sp.set_linewidth(0.6)
        ax.tick_params(colors=MUTED, labelsize=11)
        ax.set_xlabel("Time (s)", color=MUTED, fontsize=11, labelpad=4)
        ax.set_ylabel("% MVC", color=MUTED, fontsize=11, labelpad=6)
        ax.set_xlim(0, 1); ax.set_ylim(0, 5)

        # Red guide path
        self._perf_guide_line, = ax.plot(
            [], [], color=RED, lw=3.5, zorder=3,
            label="Target", solid_capstyle="round", solid_joinstyle="round")

        # Horizontal dotted level lines (drawn at START)
        self._perf_h_guides  = []
        self._perf_seg_texts = []

        # Green ground truth
        self._perf_true_line, = ax.plot(
            [], [], color=GREEN, lw=2.2, zorder=4,
            label="Ground truth", alpha=0.9, solid_capstyle="round")

        # Purple predicted/filtered
        self._perf_pred_line, = ax.plot(
            [], [], color=PURPLE, lw=2.2, zorder=5,
            label="Predicted", alpha=0.9, solid_capstyle="round")

        # Green dot at leading edge of ground truth
        self._perf_live_dot, = ax.plot(
            [], [], 'o', color=GREEN, ms=8, zorder=7, alpha=1.0)

        # Prediction hidden by default so the participant tracks the red
        # guide by feel during a trial, revealed afterward for review. Grip
        # reading (their own live grip force) stays on by default.
        self._make_visibility_toggle(
            popout_bar, "PREDICTION", PURPLE, [self._perf_pred_line],
            lambda: self._perf_canvas.draw_idle(), "performance", initial=False)
        self._make_visibility_toggle(
            popout_bar, "GRIP READING", GREEN,
            [self._perf_true_line, self._perf_live_dot],
            lambda: self._perf_canvas.draw_idle(), "performance", initial=True)

        # Playhead + now-band
        self._perf_playhead = ax.axvline(
            x=-1, color=ACCENT, lw=2.0, linestyle="--", zorder=6, alpha=0.8)
        self._perf_now_span_lines = []

        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.28),
                  ncol=3, fontsize=10, labelcolor=TEXT,
                  facecolor=PANEL2, edgecolor=BORDER, framealpha=0.85,
                  borderpad=0.8, handlelength=2.0, columnspacing=1.5)

        ax.set_title("PERFORMANCE GUIDE  —  press RECORD SESSION to begin",
                     color=MUTED, fontsize=10, loc="left", pad=10)

        self._perf_canvas = FigureCanvasTkAgg(fig, master=parent)
        self._perf_canvas.get_tk_widget().pack(fill=tk.X)

    def _perf_overlay_draw_guide(self, dur):
        """Draw red guide + labels on the performance chart in % MVC."""
        ax = self._perf_ax
        for ln in self._perf_h_guides:
            try: ln.remove()
            except Exception: pass
        self._perf_h_guides.clear()
        for tx in self._perf_seg_texts:
            try: tx.remove()
            except Exception: pass
        self._perf_seg_texts.clear()

        if not hasattr(self, "_overlay_t_path"):
            return
        gt = np.array(self._overlay_t_path)
        gf = np.array(self._overlay_y_path)  # already % MVC
        if not len(gt):
            return
        self._perf_guide_line.set_data(gt, gf)

        for pct in [25, 50, 75, 100]:
            ln = ax.axhline(pct, color=MUTED, lw=0.5, linestyle=":", zorder=1, alpha=0.5)
            self._perf_h_guides.append(ln)

        tx = ax.text(0.01, 102, "100% MVC", color=MUTED, fontsize=8,
                     transform=ax.get_yaxis_transform(), va="bottom", ha="left")
        self._perf_seg_texts.append(tx)

        reps      = getattr(self, "_overlay_reps",      3)
        rest_s    = getattr(self, "_overlay_rest_s",    REST_SEC)
        ramp_half = getattr(self, "_overlay_ramp_half", RAMP_SEC / 2.0)

        t_c = 0.0
        for i in range(reps):
            t_c += rest_s
            ramp_peak = t_c + ramp_half
            tx = ax.text(ramp_peak, 104, f"Ramp {i+1}",
                         color=MUTED, fontsize=8, ha="center", va="bottom",
                         fontweight="bold", clip_on=False)
            self._perf_seg_texts.append(tx)
            t_c += ramp_half + RAMP_PEAK_HOLD_SEC + ramp_half + rest_s

        ax.set_xlim(0, dur)
        ax.set_ylim(0, 135)
        ax.set_title("PERFORMANCE — follow the red guide  |  recording…",
                     color=YELLOW, fontsize=10, loc="left", pad=10, fontweight="bold")

    def _overlay_set_idle_view(self):
        """Show full protocol timeline (overview). x-axis always locked to full duration."""
        ax = self._overlay_ax
        ax.set_xlim(0, self._overlay_total_t)
        ax.set_title("EFFORT GUIDE  —  press START PROTOCOL to begin",
                     color=MUTED, fontsize=10, loc="left", pad=10)

    def _overlay_set_active_view(self):
        """Switch to active recording mode. x-axis stays locked to full duration."""
        ax = self._overlay_ax
        ax.set_xlim(0, self._overlay_total_t)
        ax.set_title(
            "FOLLOW THE RED LINE  —  squeeze to match",
            color=YELLOW, fontsize=11, loc="left", pad=10, fontweight="bold")

    def _update_ramp_overlay(self, elapsed, total_dur):
        """
        Call every protocol tick (50 ms) with elapsed seconds since start.
        Call with (None, None) to reset to idle.

        Active mode: x-axis slides so the playhead stays at _OVERLAY_PLAYHEAD_X
        of the visible window, giving the user a clear view of upcoming effort.
        """
        try:
            ax = self._overlay_ax
        except AttributeError:
            return

        if elapsed is None:
            # ── Reset to idle overview ────────────────────────────────────────
            self._overlay_active = False
            self._overlay_playhead.set_xdata([-1])
            for ln in self._overlay_now_span_lines:
                try: ln.remove()
                except Exception: pass
            self._overlay_now_span_lines.clear()
            self._overlay_live_t.clear()
            self._overlay_live_y.clear()
            self._overlay_live_line.set_data([], [])
            self._overlay_live_dot.set_data([], [])
            self._overlay_set_idle_view()
        else:
            t = min(float(elapsed), self._overlay_total_t)

            # First tick — switch to active title
            if not self._overlay_active:
                self._overlay_active = True
                self._overlay_set_active_view()

            # ── Fixed x-axis — never changes ──────────────────────────────────
            # x-axis is locked to [0, total_duration] set at build time.
            # Only the playhead and live trace move.

            # Move playhead
            self._overlay_playhead.set_xdata([t])

            # "Now" band — remove previous and redraw at current t
            for ln in self._overlay_now_span_lines:
                try: ln.remove()
                except Exception: pass
            self._overlay_now_span_lines.clear()
            band_w = self._overlay_total_t * 0.008
            span = ax.axvspan(t - band_w, t + band_w,
                              color=ACCENT, alpha=0.08, zorder=2)
            self._overlay_now_span_lines.append(span)

            # ── Live effort trace — full history, all points ───────────────────
            self._overlay_live_t.append(t)
            live_pct = float(STATE.live_force_pct)
            self._overlay_live_y.append(live_pct)
            self._overlay_live_line.set_data(self._overlay_live_t,
                                             self._overlay_live_y)

            # Current-value dot
            self._overlay_live_dot.set_data([t], [live_pct])

        try:
            self._overlay_canvas.draw_idle()
        except Exception:
            pass
        self._popout_sync("collect")

    # ════════════════════════════════════════
    #  PROTOCOL  (held contractions)
    # ════════════════════════════════════════
    def _build_seq(self):
        reps = int(self._reps_var.get() or NUM_REPS)
        hold = float(self._hold_var.get() or HOLD_SEC)
        rest = float(self._rest_var.get() or REST_SEC)
        seq  = []
        for rep in range(reps):
            for lvl in FORCE_LEVELS:
                seq.append({"label": 0,   "duration": rest, "record": True,
                             "text": "RELAX", "color": MUTED, "phase_type": "rest"})
                seq.append({"label": lvl, "duration": hold, "record": True,
                             "text": f"{lvl}% MVC", "color": self._lvl_color(lvl),
                             "phase_type": "hold"})
        return seq

    def _build_ramp_seq(self):
        """Continuous ramp trials for regression data.
        Each rep does:
          relax -> ramp 0→peak% MVC -> hold peak (RAMP_PEAK_HOLD_SEC) -> ramp peak→0% MVC -> relax
        The brief hold at peak avoids an immediate ramp-down right at max
        effort, which is a more natural contraction shape than snapping
        straight back down.
        The peak level varies rep-to-rep (see RAMP_PEAKS) so the training set
        sees a range of ramp heights instead of always going to 100%. If more
        reps are requested than there are entries in RAMP_PEAKS, the list
        cycles (e.g. rep 4 repeats the 50% peak again).
        During the ramp, STATE.current_level is updated continuously so every
        window gets an approximate percent-MVC label instead of a string token.
        """
        reps = int(self._reps_var.get() or NUM_REPS)
        rest = float(self._rest_var.get() or REST_SEC)
        ramp_half = RAMP_SEC / 2.0
        seq  = []
        for rep in range(reps):
            peak = RAMP_PEAKS[rep % len(RAMP_PEAKS)]
            seq.append({"label": 0, "duration": rest, "record": True,
                        "text": "RELAX", "color": MUTED,
                        "phase_type": "rest", "ramp": False})
            seq.append({"label": 0, "duration": ramp_half, "record": True,
                        "text": "RAMP UP ↑", "color": ACCENT2,
                        "phase_type": "ramp_up", "ramp": True,
                        "start_level": 0, "end_level": peak})
            seq.append({"label": peak, "duration": RAMP_PEAK_HOLD_SEC, "record": True,
                        "text": f"HOLD  {peak}% MVC", "color": PURPLE,
                        "phase_type": "hold", "ramp": False})
            seq.append({"label": peak, "duration": ramp_half, "record": True,
                        "text": "RAMP DOWN ↓", "color": PURPLE,
                        "phase_type": "ramp_down", "ramp": True,
                        "start_level": peak, "end_level": 0})
            seq.append({"label": 0, "duration": rest, "record": True,
                        "text": "RELAX", "color": MUTED,
                        "phase_type": "rest", "ramp": False})
        return seq

    def _build_full_protocol_seq(self):
        """Full training protocol — ramp/effort-graph trials only.

        Continuous ramp up/down reps (see _build_ramp_seq) are the sole
        source of training data: no discrete hold-level plateaus are
        recorded. Every window gets a continuously-varying %MVC label
        instead of being bucketed into a fixed 25/50/75/100% step.
        """
        return self._build_ramp_seq()

    def _guide_path_newtons(self, t_offset=0.0):
        """Return (t_arr, force_N_arr) for the full protocol effort guide
        in Newtons, starting at t_offset seconds.
        Uses the pre-computed overlay path (pct MVC) scaled by the calibrated
        MVC force.  Safe to call even if MVC is not yet calibrated (returns
        empty arrays).
        """
        global force_mvc_reference
        mvc_n = force_mvc_reference if force_mvc_reference else 0.0
        if mvc_n <= 0 or not hasattr(self, "_overlay_t_path"):
            return np.array([]), np.array([])
        t_arr = np.array(self._overlay_t_path) + t_offset
        f_arr = np.array(self._overlay_y_path) / 100.0 * mvc_n
        return t_arr, f_arr

    def _lvl_color(self, lvl):
        return [MUTED, ACCENT2, YELLOW, GREEN, ACCENT][FORCE_LEVELS.index(lvl)]

    def _start_protocol(self):
        if not STATE.connected_emg:
            self._log("Connect EMG first.", "err"); return
        # Full protocol: 3× ramps then holds at 25/50/75/100% MVC
        STATE.protocol_seq = self._build_full_protocol_seq()
        self._ramp_total_duration = sum(p["duration"] for p in STATE.protocol_seq)
        self._ramp_elapsed = 0.0
        self._ramp_t0 = None
        self._run_protocol()

    def _start_ramp(self):
        if not STATE.connected_emg:
            self._log("Connect EMG first.", "err"); return
        STATE.protocol_seq = self._build_ramp_seq()
        self._ramp_total_duration = sum(p["duration"] for p in STATE.protocol_seq)
        self._ramp_elapsed = 0.0
        self._ramp_t0 = None
        self._run_protocol()

    def _run_protocol(self):
        STATE.protocol_idx     = 0
        STATE.protocol_run_id += 1
        STATE.protocol_running = True
        STATE.recording        = True
        STATE.current_phase_type = "idle"
        self._start_btn.configure(state=tk.DISABLED)
        self._ramp_btn.configure( state=tk.DISABLED)
        self._stop_btn.configure( state=tk.NORMAL)
        self._set_dot(self._rec_dot, RED)
        self._rec_lbl.configure(text="RECORDING")
        self._elapsed_start = time.time()
        self._ramp_t0 = self._elapsed_start   # overlay start reference
        self._tick_elapsed()
        self._log(f"Protocol started: {len(STATE.protocol_seq)} phases.", "info")
        self._run_phase()

    def _run_phase(self):
        if not STATE.protocol_running: return
        if STATE.protocol_idx >= len(STATE.protocol_seq):
            self._stop_protocol(completed=True); return

        phase = STATE.protocol_seq[STATE.protocol_idx]
        STATE.current_phase_type = phase.get("phase_type", "hold")
        STATE.win_label = int(round(float(phase.get("label", 0)))) if phase["record"] else None
        STATE.current_level = float(phase.get("label", 0))

        self._cue_label.configure(text=phase["text"], fg=phase["color"])
        total = len(STATE.protocol_seq)
        self._proto_lbl.configure(text=f"{STATE.protocol_idx+1}/{total}")

        start = time.time()

        def tick():
            if not STATE.protocol_running: return
            elapsed = time.time() - start
            remain  = max(0, phase["duration"] - elapsed)
            pct = min(1.0, elapsed / phase["duration"])

            if phase.get("ramp", False):
                start_level = float(phase.get("start_level", 0))
                end_level   = float(phase.get("end_level", 100))
                STATE.current_level = start_level + (end_level - start_level) * pct
                STATE.win_label = int(round(STATE.current_level))
                self._cue_label.configure(
                    text=f"{phase['text']}  {int(round(STATE.current_level))}% MVC",
                    fg=phase["color"])
                self._cue_sub.configure(
                    text=f"Track the ramp smoothly • {remain:.1f}s remaining")
            else:
                STATE.current_level = float(phase.get("label", 0))
                STATE.win_label = int(round(STATE.current_level)) if phase["record"] else None
                self._cue_sub.configure(text=f"{remain:.1f}s remaining")

            w = self._prog_canvas.winfo_width()
            self._prog_canvas.coords(self._prog_fill, 0, 0, int(pct*w), 4)

            # Update the ramp overlay playhead
            if getattr(self, "_ramp_t0", None) is not None:
                t_abs = time.time() - self._ramp_t0
                total_dur = getattr(self, "_ramp_total_duration", 1.0)
                self._update_ramp_overlay(t_abs, total_dur)

            if elapsed < phase["duration"]:
                self._protocol_timer = self.after(50, tick)
            else:
                STATE.protocol_idx += 1
                self._prog_canvas.coords(self._prog_fill, 0,0,0,4)
                self._run_phase()
        tick()

    def _stop_protocol(self, completed=False):
        STATE.protocol_running = False
        STATE.recording        = False
        STATE.win_label        = None
        STATE.current_phase_type = "idle"
        STATE.current_level = 0
        if self._protocol_timer: self.after_cancel(self._protocol_timer)
        self._start_btn.configure(state=tk.NORMAL)
        self._ramp_btn.configure( state=tk.NORMAL)
        self._stop_btn.configure( state=tk.DISABLED)
        self._set_dot(self._rec_dot, MUTED)
        self._rec_lbl.configure(text="IDLE")
        self._cue_label.configure(
            text="DONE" if completed else "STOPPED",
            fg=GREEN if completed else YELLOW)
        # Reset overlay playhead to idle state
        self._ramp_t0 = None
        self._update_ramp_overlay(None, None)
        self._cue_sub.configure(
            text=f"Protocol complete! {len(STATE.dataset)} windows recorded."
            if completed else "Stopped.")
        if self._elapsed_id: self.after_cancel(self._elapsed_id)
        self._log(f"Protocol {'complete' if completed else 'stopped'}. "
                  f"{len(STATE.dataset)} windows.", "ok" if completed else "warn")

    def _tick_elapsed(self):
        if not STATE.protocol_running: return
        t = int(time.time() - self._elapsed_start)
        self._elapsed_lbl.configure(text=f"{t//60:02d}:{t%60:02d}")
        self._elapsed_id = self.after(500, self._tick_elapsed)

    # ════════════════════════════════════════
    #  TRAINING
    # ════════════════════════════════════════
    def _select_model(self, name):
        STATE.model_name = name
        for n, btn in self._model_chips.items():
            btn.configure(bg=PANEL2 if n!=name else PANEL,
                          fg=MUTED  if n!=name else ACCENT)

    def _run_training(self):
        if len(STATE.dataset) < 10:
            self._log("Need at least 10 windows.", "err"); return
        self._train_btn.configure(text="TRAINING…", state=tk.DISABLED)
        self._log(f"Training {STATE.model_name} on {len(STATE.dataset)} windows…", "info")
        threading.Thread(target=self._train_thread, daemon=True).start()

    def _train_thread(self):
        try:
            X_full      = np.array([d["features"] for d in STATE.dataset])
            y_full      = np.array([d["force"]    for d in STATE.dataset])
            levels_full = np.array([d.get("level", -1) for d in STATE.dataset])
            # Group by contraction/rest phase (not just row) so a fold split
            # never separates overlapping windows drawn from the same
            # continuous contraction — that leakage is what previously
            # inflated CV R² (windows from one hold showing up in both
            # train and test are near-duplicates, not independent samples).
            groups_full = np.array([d.get("group", i) for i, d in enumerate(STATE.dataset)])

            # ── Balance the 0%-MVC rest cluster ─────────────────────────────
            # Rest windows are recorded during every inter-level pause, so
            # they typically outnumber any single active level several times
            # over. Left uncapped, the regressor can post a high R² mostly by
            # nailing the near-zero cluster while staying mediocre across the
            # 25-100% graded range that actually matters for proportional
            # control. Cap rest windows at roughly 1x total active windows.
            rest_mask = levels_full == 0
            n_active  = int((~rest_mask).sum())
            n_rest    = int(rest_mask.sum())
            cap_rest  = max(n_active, 20)
            if n_rest > cap_rest:
                bal_rng    = np.random.RandomState(42)
                rest_idx   = np.where(rest_mask)[0]
                keep_rest  = bal_rng.choice(rest_idx, size=cap_rest, replace=False)
                active_idx = np.where(~rest_mask)[0]
                keep_idx   = np.sort(np.concatenate([keep_rest, active_idx]))
            else:
                keep_idx = np.arange(len(STATE.dataset))
            n_dropped_rest = n_rest - int((levels_full[keep_idx] == 0).sum())

            X      = X_full[keep_idx]
            y      = y_full[keep_idx]
            levels = levels_full[keep_idx]
            groups = groups_full[keep_idx]
            n_groups = len(np.unique(groups))

            cv_n = int(self._cv_var.get() or 5)
            cv_n = max(2, min(cv_n, n_groups))
            cv   = GroupKFold(n_splits=cv_n)

            def make_model(name):
                if name == "XGBoost" and XGB_AVAILABLE:
                    return XGBRegressor(n_estimators=300, learning_rate=0.05,
                                        max_depth=5, subsample=0.8,
                                        random_state=42, verbosity=0)
                elif name == "LightGBM" and LGB_AVAILABLE:
                    return LGBMRegressor(n_estimators=300, learning_rate=0.05,
                                         max_depth=5, subsample=0.8,
                                         random_state=42, verbosity=-1)
                elif name == "SVR":
                    return SVR(kernel="rbf", C=100, gamma="scale", epsilon=0.5)
                elif name == "Random Forest":
                    return RandomForestRegressor(n_estimators=300, random_state=42)
                else:
                    return GradientBoostingRegressor(n_estimators=300, learning_rate=0.05,
                                                      max_depth=4, random_state=42)

            active_feat_names = (MR_FEATURE_NAMES if ACTIVE_DEVICE == "mindrove"
                                 else FEATURE_NAMES)

            # Proper CV: fit scaler only inside each fold
            cv_pipe = Pipeline([
                ("scaler", StandardScaler()),
                ("model", make_model(STATE.model_name))
            ])

            y_pred_cv = cross_val_predict(cv_pipe, X, y, groups=groups, cv=cv)
            r2_cv   = float(r2_score(y, y_pred_cv))
            rmse_cv = float(np.sqrt(mean_squared_error(y, y_pred_cv)))
            nrmse   = rmse_cv / (y.max() - y.min() + 1e-9) * 100
            r_cv, _ = pearsonr(y, y_pred_cv)

            # Same metrics restricted to non-rest windows — the global R²
            # above is easy to inflate via the large near-zero rest cluster;
            # this number reflects accuracy within the graded 25-100% range,
            # which is what actually matters for proportional control.
            active_mask = levels != 0
            if active_mask.sum() >= 5 and np.ptp(y[active_mask]) > 1e-9:
                y_a, yp_a = y[active_mask], y_pred_cv[active_mask]
                r2_active   = float(r2_score(y_a, yp_a))
                rmse_active = float(np.sqrt(mean_squared_error(y_a, yp_a)))
                nrmse_active = rmse_active / (np.ptp(y_a) + 1e-9) * 100
                r_active, _  = pearsonr(y_a, yp_a)
                r_active = float(r_active)
            else:
                r2_active = nrmse_active = r_active = None

            # Final model fit on all data for deployment
            scaler = StandardScaler()
            Xs = scaler.fit_transform(X)
            clf_final = make_model(STATE.model_name)
            clf_final.fit(Xs, y)
            STATE.scaler      = scaler
            STATE.model       = clf_final
            STATE.model_trained = True

            # SHAP feature importance from the actual selected model
            shap_rng = np.random.RandomState(42)
            bg_n = min(100, len(Xs))
            eval_n = min(250, len(Xs))
            bg_idx = shap_rng.choice(len(Xs), size=bg_n, replace=False)
            eval_idx = shap_rng.choice(len(Xs), size=eval_n, replace=False)
            X_bg = Xs[bg_idx]
            X_eval = Xs[eval_idx]
            explainer = shap.Explainer(clf_final, X_bg, feature_names=active_feat_names)
            shap_values = explainer(X_eval)
            shap_importance = np.abs(shap_values.values).mean(axis=0)
            importances = sorted(zip(active_feat_names, shap_importance),
                                 key=lambda x: -x[1])

            self.after(0, lambda: self._train_done(
                r2_cv, nrmse, r_cv, y, y_pred_cv, importances, n_groups, cv_n,
                n_dropped_rest, r2_active, nrmse_active, r_active))
        except Exception as e:
            import traceback
            self.after(0, lambda: self._log(f"Training error: {e}\n{traceback.format_exc()}", "err"))
            self.after(0, lambda: self._train_btn.configure(
                text="⬡  TRAIN MODEL", state=tk.NORMAL))

    def _train_done(self, r2, nrmse, r_val, y, y_pred, importances, n_groups=None, cv_n=None,
                     n_dropped_rest=0, r2_active=None, nrmse_active=None, r_active=None):
        # This runs inside a self.after() callback, so an uncaught exception
        # here is swallowed by Tkinter silently — the button would stay
        # stuck on "TRAINING..." with no visible error. Guarantee it's
        # always restored, and surface the actual error if something breaks.
        try:
            self._train_done_impl(r2, nrmse, r_val, y, y_pred, importances, n_groups, cv_n,
                                   n_dropped_rest, r2_active, nrmse_active, r_active)
        except Exception as e:
            import traceback
            self._log(f"Training UI update failed: {e}\n{traceback.format_exc()}", "err")
        finally:
            self._train_btn.configure(text="⬡  RETRAIN MODEL", state=tk.NORMAL)

    def _train_done_impl(self, r2, nrmse, r_val, y, y_pred, importances, n_groups, cv_n,
                          n_dropped_rest, r2_active, nrmse_active, r_active):
        r2_c   = GREEN  if r2   > 0.80 else YELLOW if r2   > 0.60 else RED
        rmse_c = GREEN  if nrmse < 10  else YELLOW if nrmse < 20  else RED
        r_c    = GREEN  if r_val > 0.90 else YELLOW if r_val > 0.75 else RED
        if n_groups is not None:
            self._log(f"Grouped CV: {cv_n} folds split across {n_groups} contraction/rest "
                       f"phases (no phase split across train/test).", "info")
        if n_dropped_rest:
            self._log(f"Balanced classes: dropped {n_dropped_rest} excess 0%-MVC rest "
                       f"windows from training so the rest cluster doesn't dominate the fit.", "info")

        self._t_r2.configure(  text=f"{r2:.3f}",   fg=r2_c)
        self._t_rmse.configure(text=f"{nrmse:.1f}%", fg=rmse_c)
        self._t_r.configure(   text=f"{r_val:.3f}", fg=r_c)
        self._t_n.configure(   text=str(len(STATE.dataset)))

        if r2_active is not None:
            r2a_c = GREEN if r2_active > 0.80 else YELLOW if r2_active > 0.60 else RED
            self._t_r2_active.configure(text=f"{r2_active:.3f}", fg=r2a_c)
            self._log(f"  ↳ excl. rest (25-100% MVC only): R²={r2_active:.3f}  "
                      f"%RMSE={nrmse_active:.1f}%  r={r_active:.3f}", "info")
        else:
            self._t_r2_active.configure(text="—", fg=MUTED)

        STATE.cv_r2         = r2
        STATE.cv_rmse       = nrmse
        STATE.cv_pearson    = r_val
        STATE.cv_model_name = STATE.model_name
        self._log(f"✔ {STATE.model_name}  R²={r2:.3f}  %RMSE={nrmse:.1f}%  r={r_val:.3f}", "ok")

        # Scatter plot
        self._scatter_ax.clear()
        self._scatter_ax.set_facecolor(PANEL2)
        self._scatter_ax.spines[:].set_color(BORDER)
        self._scatter_ax.tick_params(colors=MUTED, labelsize=7)
        y_pct      = _arr_to_pct_mvc(y)
        y_pred_pct = _arr_to_pct_mvc(y_pred)
        self._scatter_ax.scatter(y_pct, y_pred_pct, alpha=0.4, s=8, color=ACCENT)
        mn, mx = min(y_pct.min(), y_pred_pct.min()), max(y_pct.max(), y_pred_pct.max())
        self._scatter_ax.plot([mn,mx],[mn,mx], color=GREEN, lw=1, linestyle="--", label="ideal")
        self._scatter_ax.set_xlabel("Actual (% MVC)", color=MUTED, fontsize=7)
        self._scatter_ax.set_ylabel("Predicted (% MVC)", color=MUTED, fontsize=7)
        self._scatter_ax.set_title(f"R²={r2:.3f}  r={r_val:.3f}",
                                    color=TEXT, fontsize=8)
        self._scatter_ax.legend(fontsize=7, labelcolor=TEXT,
                                 facecolor=PANEL2, edgecolor=BORDER)
        self._scatter_fig.patch.set_facecolor(BG)
        self._scatter_canvas.draw()

        # Feature importance (top 15)
        self._fi_ax.clear()
        self._fi_ax.set_facecolor(PANEL2)
        top = importances[:15]
        names  = [x[0] for x in top]
        values = [x[1] for x in top]
        bar_colors = [ACCENT if v == max(values) else
                      GREEN  if v > 0.05 else MUTED for v in values]
        self._fi_ax.barh(range(len(names)), values, color=bar_colors)
        self._fi_ax.set_yticks(range(len(names)))
        self._fi_ax.set_yticklabels(names, color=TEXT, fontsize=7)
        self._fi_ax.set_xlabel("Importance", color=MUTED, fontsize=7)
        self._fi_ax.set_title("Feature Importance (SHAP)", color=MUTED, fontsize=8)
        self._fi_ax.tick_params(colors=MUTED, labelsize=7)
        self._fi_ax.spines[:].set_color(BORDER)
        self._fi_fig.patch.set_facecolor(BG)
        self._fi_canvas.draw()

        self._switch_tab("validate")

    # ════════════════════════════════════════
    #  VALIDATION  (real-time)
    # ════════════════════════════════════════
    def _toggle_ema(self):
        if STATE.inferring:
            self._log("Stop validation before changing filters.", "warn")
            return
        STATE.ema_enabled = not STATE.ema_enabled
        STATE._ema_last   = None
        if STATE.ema_enabled:
            self._ema_btn.configure(text="◈  EMA ON",
                                    fg=ACCENT, bg=PANEL2,
                                    highlightbackground=ACCENT)
        else:
            self._ema_btn.configure(text="⬡  EMA OFF",
                                    fg=MUTED, bg=PANEL2,
                                    highlightbackground=BORDER)
        self._update_filtered_row_visibility()

    def _toggle_median(self):
        if STATE.inferring:
            self._log("Stop validation before changing filters.", "warn")
            return
        STATE.median_enabled = not STATE.median_enabled
        if STATE.median_enabled:
            self._med_btn.configure(text="◈  MEDIAN ON",
                                    fg=PURPLE, bg=PANEL2,
                                    highlightbackground=PURPLE)
        else:
            self._med_btn.configure(text="⬡  MEDIAN OFF",
                                    fg=MUTED, bg=PANEL2,
                                    highlightbackground=BORDER)
        self._update_filtered_row_visibility()

    def _update_filtered_row_visibility(self):
        # Smoothing controls are always visible.
        # Only the FILTERED METRICS row appears when a filter is active.
        any_active = STATE.ema_enabled or STATE.median_enabled
        if any_active:
            self._filt_label.pack(anchor=tk.W, padx=8)
            self._filt_row.pack(fill=tk.X, padx=8, pady=(0,8))
        else:
            self._filt_label.pack_forget()
            self._filt_row.pack_forget()
            for w in [self._filt_r2, self._filt_rmse,
                      self._filt_r, self._filt_delta]:
                w.configure(text="—", fg=PURPLE)

    def _run_inference(self, feats, true_force):
        """Called from _ingest when inferring=True. Predicts force and updates live charts/metrics."""
        if not STATE.model or not STATE.scaler: return
        try:
            x  = feats.reshape(1, -1)
            xs = STATE.scaler.transform(x)
            pred = float(STATE.model.predict(xs)[0])
        except Exception:
            return

        STATE.val_pred.append(pred)
        STATE.val_true.append(true_force)

        # ── Apply smoothing filters to display pred ──────────────────────────
        display_pred = pred
        _deploy_t = time.time() - getattr(self, "_deploy_t0", time.time())

        # Median filter — replace with median of last N raw predictions
        if STATE.median_enabled and len(STATE.val_pred) >= STATE.median_window:
            window = list(STATE.val_pred)[-STATE.median_window:]
            display_pred = float(np.median(window))

        # EMA — blend smoothed value with raw (or already-median'd) value
        if STATE.ema_enabled:
            alpha = STATE.ema_alpha
            if STATE._ema_last is None:
                STATE._ema_last = display_pred
            STATE._ema_last = alpha * display_pred + (1.0 - alpha) * STATE._ema_last
            display_pred = STATE._ema_last

        # Feed deploy capture if active
        if STATE.deploy_recording:
            self._deploy_record_sample(pred, display_pred, true_force, _deploy_t)

        # Feed performance capture if active
        if STATE.performance_recording:
            _perf_t = time.time() - getattr(self, "_perf_t0", time.time())
            self._perf_record_sample(pred, display_pred, true_force, _perf_t)

        # Cache latest predictions for exoskeleton streaming / plotting
        self._latest_pred_force_raw = float(pred)
        self._latest_pred_force_filt = float(display_pred)
        self._latest_true_force = float(true_force)

        # Update predicted force readout (shows smoothed value)
        pred_pct = (display_pred / (force_mvc_reference + 1e-9) * 100.0) if force_mvc_reference else None
        if pred_pct is not None:
            self._pred_val.configure(text=f"{pred_pct:.1f}% MVC  ({display_pred:.2f} N)")
        else:
            self._pred_val.configure(text=f"{display_pred:.2f} N")
        w = self._pred_bar_c.winfo_width()
        self._pred_bar_c.coords(self._pred_bar, 0, 0,
                                 max(2, int((display_pred / 60.0) * w)), 5)

        # Update live metrics and charts every 5 windows
        if len(STATE.val_pred) >= 5:
            yp_raw = np.array(STATE.val_pred)
            yt     = np.array(STATE.val_true)

            # Build smoothed series for the plot
            yp = yp_raw.copy()
            if STATE.median_enabled and len(yp) >= STATE.median_window:
                from scipy.signal import medfilt
                yp = medfilt(yp, kernel_size=STATE.median_window)
            if STATE.ema_enabled:
                alpha = STATE.ema_alpha
                ema = np.zeros_like(yp)
                ema[0] = yp[0]
                for k in range(1, len(yp)):
                    ema[k] = alpha * yp[k] + (1 - alpha) * ema[k - 1]
                yp = ema

            yp_raw_arr = yp_raw  # keep raw for metrics

            try:
                ts_lag = list(STATE.deploy_timestamps)
                if len(ts_lag) != len(yt):
                    ts_lag = None

                # ── Raw metrics ──────────────────────────────────────────────
                r2    = float(r2_score(yt, yp_raw_arr))
                rmse  = float(np.sqrt(mean_squared_error(yt, yp_raw_arr)))
                nrmse = rmse / (yt.max() - yt.min() + 1e-9) * 100
                r, _  = pearsonr(yt, yp_raw_arr) if len(yt) > 2 else (0, 0)
                lag_s = compute_signal_lag_seconds(yt, yp_raw_arr, ts_lag)

                # ── Filtered metrics (only when a filter is on) ──────────────
                any_filter = STATE.ema_enabled or STATE.median_enabled
                if any_filter:
                    r2_f    = float(r2_score(yt, yp))
                    rmse_f  = float(np.sqrt(mean_squared_error(yt, yp)))
                    nrmse_f = rmse_f / (yt.max() - yt.min() + 1e-9) * 100
                    r_f, _  = pearsonr(yt, yp) if len(yt) > 2 else (0, 0)
                    lag_f_s = compute_signal_lag_seconds(yt, yp, ts_lag)
            except Exception:
                return

            STATE.live_r2      = r2
            STATE.live_rmse    = nrmse
            STATE.live_pearson = r
            STATE.live_lag_s   = lag_s

            if any_filter:
                STATE.live_filt_r2 = r2_f
                STATE.live_filt_rmse = nrmse_f
                STATE.live_filt_pearson = r_f
                STATE.live_filt_lag_s = lag_f_s
            else:
                STATE.live_filt_r2 = r2
                STATE.live_filt_rmse = nrmse
                STATE.live_filt_pearson = r
                STATE.live_filt_lag_s = lag_s

            r2_c  = GREEN  if r2    > 0.80 else YELLOW if r2    > 0.60 else RED
            rm_c  = GREEN  if nrmse < 10   else YELLOW if nrmse < 20   else RED
            r_c   = GREEN  if r     > 0.90 else YELLOW if r     > 0.75 else RED
            lag_ms = lag_s * 1000.0 if lag_s is not None else None

            self._live_r2.configure(   text=f"{r2:.3f}",     fg=r2_c)
            self._live_rmse.configure( text=f"{nrmse:.1f}%", fg=rm_c)
            self._live_r.configure(    text=f"{r:.3f}",      fg=r_c)
            self._live_lag.configure(  text=(f"{lag_ms:+.0f}" if lag_ms is not None else "—"))

            # Update filtered metrics row if active
            if any_filter:
                r2f_c = GREEN  if r2_f    > 0.80 else YELLOW if r2_f    > 0.60 else RED
                rmf_c = GREEN  if nrmse_f < 10   else YELLOW if nrmse_f < 20   else RED
                rf_c  = GREEN  if r_f     > 0.90 else YELLOW if r_f     > 0.75 else RED
                lagf_ms = lag_f_s * 1000.0 if lag_f_s is not None else None
                self._filt_r2.configure(   text=f"{r2_f:.3f}",     fg=r2f_c)
                self._filt_rmse.configure( text=f"{nrmse_f:.1f}%", fg=rmf_c)
                self._filt_r.configure(    text=f"{r_f:.3f}",      fg=rf_c)
                self._filt_lag.configure(  text=(f"{lagf_ms:+.0f}" if lagf_ms is not None else "—"), fg=PURPLE)

            # ── Update live scatter (throttled redraw) ───────────────────────
            now2 = time.time()
            if (now2 - self._last_val_scatter_ts) * 1000.0 >= VAL_SCATTER_UPDATE_MS:
                self._lval_ax.clear()
                self._lval_ax.set_facecolor(PANEL2)
                self._lval_ax.spines[:].set_color(BORDER)
                self._lval_ax.tick_params(colors=MUTED, labelsize=7)
                yt_pct = _arr_to_pct_mvc(yt)
                yp_pct = _arr_to_pct_mvc(yp)
                self._lval_ax.scatter(yt_pct, yp_pct, alpha=0.5, s=6, color=ACCENT)
                mn = min(yt_pct.min(), yp_pct.min()); mx = max(yt_pct.max(), yp_pct.max())
                self._lval_ax.plot([mn, mx], [mn, mx], color=GREEN,
                                    lw=1, linestyle="--")
                self._lval_ax.set_xlabel("Actual (% MVC)",    color=MUTED, fontsize=7)
                self._lval_ax.set_ylabel("Predicted (% MVC)", color=MUTED, fontsize=7)
                self._lval_ax.set_title(f"Live  R²={r2:.3f}  r={r:.3f}",
                                         color=TEXT, fontsize=8)
                self._lval_fig.patch.set_facecolor(BG)
                self._lval_canvas.draw_idle()
                self._last_val_scatter_ts = now2

    def _toggle_validate(self):
        if STATE.performance_recording:
            self._log("Stop the Performance tab recording before starting validation.", "warn"); return
        if not STATE.model_trained:
            self._log("Train a model first.", "err"); return
        if not STATE.connected_emg and not self._sim_mode:
            self._log("Connect EMG (or enable Simulate) first.", "err"); return
        if STATE.inferring:
            STATE.inferring = False
            if hasattr(self, "_val_poll_id"):
                self.after_cancel(self._val_poll_id)
            self._val_btn.configure(text="▶  START VALIDATION", fg=ACCENT)
            self._val_prog_lbl.configure(text="STOPPED", fg=YELLOW)
            self._val_prog_track.coords(self._val_prog_bar, 0, 0, 0, 6)
            self._log("Validation stopped.", "warn")
            return
        try:
            dur = float(self._val_dur_var.get())
        except ValueError:
            dur = 20.0
        STATE.deploy_raw_pred   = []
        STATE.deploy_filt_pred  = []
        STATE.deploy_true       = []
        STATE.deploy_timestamps = []
        STATE.live_filt_r2 = None
        STATE.live_filt_rmse = None
        STATE.live_filt_pearson = None
        STATE.live_lag_s = None
        STATE.live_filt_lag_s = None
        STATE.deploy_duration   = dur
        STATE._ema_last         = None
        # Flush pre-validation samples so the first prediction uses only
        # data captured after the user clicked Start, not stale buffer data.
        STATE.win_buf.clear()
        STATE.win_force_buf.clear()
        STATE.deploy_recording  = True
        STATE.inferring         = True
        self._deploy_t0         = time.time()
        self._send_deploy_btn.configure(state=tk.DISABLED, bg=PANEL2)
        self._val_btn.configure(text="■  STOP", fg=RED)
        self._val_prog_lbl.configure(text="RECORDING  0.0s / " + str(int(dur)) + "s", fg=RED)
        self._val_prog_track.coords(self._val_prog_bar, 0, 0, 0, 6)
        self._log("Validation started -- " + str(int(dur)) + "s window.", "info")

        # Reset all streaming lines and playhead
        self._val_pred_line.set_data([], [])
        self._val_true_line.set_data([], [])
        self._val_live_dot.set_data([], [])
        self._val_playhead.set_xdata([-1])
        for ln in self._val_now_span_lines:
            try: ln.remove()
            except Exception: pass
        self._val_now_span_lines.clear()

        # Draw red guide + labels + h-lines, lock axes
        self._val_overlay_draw_guide(dur)
        self._val_canvas.draw_idle()
        self._popout_sync("validate")

        self._val_poll_id = self.after(100, self._val_poll)

    def _val_poll(self):
        if not STATE.inferring:
            return
        elapsed = time.time() - self._deploy_t0
        dur     = STATE.deploy_duration
        pct     = min(elapsed / dur, 1.0)
        w       = self._val_prog_track.winfo_width()
        self._val_prog_track.coords(self._val_prog_bar, 0, 0, int(pct * w), 6)
        self._val_prog_lbl.configure(
            text="RECORDING  " + f"{elapsed:.1f}" + "s / " + f"{dur:.0f}" + "s")

        # ── Stream pred + true into chart ─────────────────────────────────────
        # deploy_timestamps are relative to _deploy_t0, so use directly
        ts  = list(STATE.deploy_timestamps)
        yp_raw = list(STATE.deploy_filt_pred if STATE.ema_enabled or STATE.median_enabled
                      else STATE.deploy_raw_pred)
        yt_raw = list(STATE.deploy_true)
        n   = min(len(ts), len(yp_raw), len(yt_raw))
        if n > 0:
            mvc_n = force_mvc_reference or 1.0
            yp = [v / mvc_n * 100.0 for v in yp_raw[:n]]
            yt = [v / mvc_n * 100.0 for v in yt_raw[:n]]
            self._val_pred_line.set_data(ts[:n], yp)
            self._val_true_line.set_data(ts[:n], yt)
            self._val_live_dot.set_data([ts[n-1]], [yt[n-1]])
            data_max = max(max(yp), max(yt))
            if data_max * 1.1 > self._val_ax.get_ylim()[1]:
                self._val_ax.set_ylim(0, max(135, data_max * 1.15))

        # ── Playhead + now-band (fixed x-axis so band_w is stable) ───────────
        self._val_playhead.set_xdata([elapsed])
        for ln in self._val_now_span_lines:
            try: ln.remove()
            except Exception: pass
        self._val_now_span_lines.clear()
        try:
            band_w = dur * 0.008
            span = self._val_ax.axvspan(elapsed - band_w, elapsed + band_w,
                                         color=ACCENT, alpha=0.08, zorder=2)
            self._val_now_span_lines.append(span)
        except Exception:
            pass
        self._val_canvas.draw_idle()
        self._popout_sync("validate")

        # ── Check completion ──────────────────────────────────────────────────
        if elapsed >= dur:
            STATE.inferring        = False
            STATE.deploy_recording = False
            self._val_btn.configure(text="▶  START VALIDATION", fg=ACCENT)
            n_win = len(STATE.deploy_true)
            self._val_prog_lbl.configure(
                text="COMPLETE  " + f"{dur:.0f}" + "s  |  " + str(n_win) + " windows",
                fg=GREEN)
            self._val_ax.set_title("VALIDATION — complete",
                                    color=GREEN, fontsize=10, loc="left", pad=10,
                                    fontweight="bold")
            # Park playhead off-screen
            self._val_playhead.set_xdata([-1])
            for ln in self._val_now_span_lines:
                try: ln.remove()
                except Exception: pass
            self._val_now_span_lines.clear()
            self._val_canvas.draw_idle()
            self._popout_sync("validate")
            self._log("Validation complete -- " + str(n_win) + " windows captured.", "ok")
            self._val_autosave_csv()
            self._send_deploy_btn.configure(state=tk.NORMAL, bg="#1a3a1a")
            return
        self._val_poll_id = self.after(100, self._val_poll)

    def _val_autosave_csv(self):
        """Auto-save validation recording to CSV immediately after capture."""
        import os, csv as _csv
        yp_raw = list(STATE.deploy_raw_pred)
        yp_f   = list(STATE.deploy_filt_pred)
        yt     = list(STATE.deploy_true)
        ts     = list(STATE.deploy_timestamps)
        if len(yt) < 5:
            self._last_val_csv = None
            return
        # Save to Desktop/PropControl_Runs/
        folder = os.path.join(os.path.expanduser("~"), "Desktop", "PropControl_Runs")
        os.makedirs(folder, exist_ok=True)
        fname = "val_" + time.strftime("%Y%m%d_%H%M%S") + ".csv"
        path  = os.path.join(folder, fname)
        try:
            with open(path, "w", newline="") as f:
                w = _csv.writer(f)
                w.writerow(["time_s", "true_force_N", "raw_pred_N", "filt_pred_N"])
                for t, y, rp, fp in zip(ts, yt, yp_raw, yp_f):
                    w.writerow([f"{t:.4f}", f"{y:.4f}", f"{rp:.4f}", f"{fp:.4f}"])
            self._last_val_csv = path
            self._log("Auto-saved: " + path, "ok")
            # Show filename on validate progress label
            self._val_prog_lbl.configure(
                text="SAVED: " + fname, fg=GREEN)
        except Exception as e:
            self._last_val_csv = None
            self._log("CSV save error: " + str(e), "err")

    def _send_to_deploy(self):
        """Send the exact in-memory validation run straight to Deploy."""
        self._send_deploy_btn.configure(state=tk.DISABLED)
        self._val_handoff_to_deploy()

    def _do_deploy_render(self):
        """Load from auto-saved CSV and render deploy tab."""
        import csv as _csv, os
        path = getattr(self, "_last_val_csv", None)
        if not path or not os.path.exists(path):
            self._log("No saved CSV found. Try recording again.", "err")
            self._send_deploy_btn.configure(state=tk.NORMAL, bg="#1a3a1a")
            return
        try:
            rows = []
            with open(path, newline="") as f:
                reader = _csv.DictReader(f)
                for row in reader:
                    rows.append(row)
            ts     = np.array([float(r["time_s"])       for r in rows])
            yt     = np.array([float(r["true_force_N"]) for r in rows])
            yp_raw = np.array([float(r["raw_pred_N"])   for r in rows])
            yp_f   = np.array([float(r["filt_pred_N"])  for r in rows])
            self._log("Loaded " + str(len(yt)) + " rows from " + os.path.basename(path), "ok")
        except Exception as e:
            self._log("CSV load error: " + str(e), "err")
            self._send_deploy_btn.configure(state=tk.NORMAL, bg="#1a3a1a")
            return
        if len(yt) < 5:
            self._log("Not enough rows in CSV.", "warn")
            self._send_deploy_btn.configure(state=tk.NORMAL, bg="#1a3a1a")
            return
        # Force geometry to resolve before drawing
        self._dep_canvas.get_tk_widget().update_idletasks()
        self._deploy_populate(yp_raw, yp_f, yt, ts)

    def _val_handoff_to_deploy(self):
        yp_raw = np.array(STATE.deploy_raw_pred)
        yp_f   = np.array(STATE.deploy_filt_pred)
        yt     = np.array(STATE.deploy_true)
        ts     = np.array(STATE.deploy_timestamps)
        if len(yt) < 5:
            self._log("Not enough data for deploy.", "warn")
            return
        # Switch tab first so the canvas is visible/sized before drawing
        self._switch_tab("deploy")
        self.after(50, lambda: self._deploy_populate(yp_raw, yp_f, yt, ts))


    # ─────────────────────────────────────────────────────────────────────────
    # DEPLOY TAB — receives data from Validate, shows results, exports
    # ─────────────────────────────────────────────────────────────────────────

    def _build_deploy_tab(self, parent):
        from matplotlib.gridspec import GridSpec as GS

        # ── Header label ─────────────────────────────────────────────────────
        hdr = tk.Frame(parent, bg=PANEL,
                       highlightbackground=BORDER, highlightthickness=1)
        hdr.pack(fill=tk.X, padx=16, pady=(12, 4))
        tk.Label(hdr, text="RESULTS",
                 font=("Arial", 10, "bold"), fg=MUTED, bg=PANEL
                 ).pack(anchor=tk.W, padx=10, pady=(8, 2))
        tk.Label(hdr,
                 text="Run a timed validation in the  03 VALIDATE  tab, "
                      "then press  \u2192 SEND TO DEPLOY  to populate this view.",
                 font=FONT_MONO_SM, fg=MUTED, bg=PANEL, justify=tk.LEFT
                 ).pack(anchor=tk.W, padx=10, pady=(0, 8))

        # ── Status label ─────────────────────────────────────────────────────
        self._dep_status = tk.Label(parent, text="WAITING FOR DATA",
                                     font=("Arial", 11, "bold"), fg=MUTED, bg=BG)
        self._dep_status.pack(anchor=tk.W, padx=20, pady=(4, 0))

        # ── Metrics panel ────────────────────────────────────────────────────
        met_f = tk.Frame(parent, bg=PANEL,
                         highlightbackground=BORDER, highlightthickness=1)
        met_f.pack(fill=tk.X, padx=16, pady=(4, 4))

        # Offline CV row
        self._dep_offline_lbl = tk.Label(met_f, text="OFFLINE MODEL  (CV)",
                                          font=("Courier New", 8, "bold"),
                                          fg=MUTED, bg=PANEL)
        self._dep_offline_lbl.pack(anchor=tk.W, padx=8, pady=(6, 0))
        off_row = tk.Frame(met_f, bg=PANEL); off_row.pack(fill=tk.X, padx=8, pady=(0, 4))
        self._dep_r2   = self._metric_card(off_row, "R\u00b2",        "\u2014", ACCENT)
        self._dep_rmse = self._metric_card(off_row, "%RMSE",     "\u2014", YELLOW)
        self._dep_r    = self._metric_card(off_row, "Pearson r", "\u2014", GREEN)
        self._dep_n    = self._metric_card(off_row, "Windows",   "\u2014", MUTED)

        # Real-time row
        tk.Label(met_f, text="REAL-TIME VALIDATION",
                 font=("Courier New", 8, "bold"), fg=PURPLE, bg=PANEL
                 ).pack(anchor=tk.W, padx=8)
        rt_row = tk.Frame(met_f, bg=PANEL); rt_row.pack(fill=tk.X, padx=8, pady=(0, 6))
        self._dep_rt_r2   = self._metric_card(rt_row, "R\u00b2",        "\u2014", PURPLE)
        self._dep_rt_rmse = self._metric_card(rt_row, "%RMSE",     "\u2014", PURPLE)
        self._dep_rt_r    = self._metric_card(rt_row, "Pearson r", "\u2014", PURPLE)
        self._dep_rt_delta= self._metric_card(rt_row, "\u0394R\u00b2 vs CV", "\u2014", PURPLE)

        # ── Charts ───────────────────────────────────────────────────────────
        chart_f = tk.Frame(parent, bg=BG)
        chart_f.pack(fill=tk.BOTH, expand=True, padx=16, pady=(0, 4))

        self._dep_fig = Figure(figsize=(8, 3.8), facecolor=BG, tight_layout=True)
        gs = GS(1, 2, figure=self._dep_fig, wspace=0.35)
        self._dep_ax_ts = self._dep_fig.add_subplot(gs[0, 0])
        self._dep_ax_sc = self._dep_fig.add_subplot(gs[0, 1])
        for ax in [self._dep_ax_ts, self._dep_ax_sc]:
            ax.set_facecolor(PANEL2)
            for sp in ax.spines.values(): sp.set_color(BORDER)
            ax.tick_params(colors=MUTED, labelsize=7)
        self._dep_ax_ts.set_title("Predicted vs Ground Truth", color=MUTED, fontsize=8)
        self._dep_ax_ts.set_xlabel("Time (s)",   color=MUTED, fontsize=7)
        self._dep_ax_ts.set_ylabel("% MVC",      color=MUTED, fontsize=7)
        self._dep_ax_sc.set_title("Scatter: Predicted vs Actual", color=MUTED, fontsize=8)
        self._dep_ax_sc.set_xlabel("Actual (% MVC)", color=MUTED, fontsize=7)
        self._dep_ax_sc.set_ylabel("Predicted (% MVC)", color=MUTED, fontsize=7)
        self._dep_canvas = FigureCanvasTkAgg(self._dep_fig, master=chart_f)
        self._dep_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # ── Export buttons ────────────────────────────────────────────────────
        btn_f = tk.Frame(parent, bg=BG); btn_f.pack(fill=tk.X, padx=16, pady=(0, 8))
        self._dep_export_csv = tk.Button(btn_f, text="\u21e9  DOWNLOAD CSV",
                                          font=("Arial", 10, "bold"),
                                          bg=PANEL2, fg=ACCENT, relief=tk.FLAT,
                                          pady=7, state=tk.DISABLED,
                                          command=self._deploy_export_csv)
        self._dep_export_csv.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        self._dep_export_png = tk.Button(btn_f, text="\u21e9  DOWNLOAD GRAPHS (PNG)",
                                          font=("Arial", 10, "bold"),
                                          bg=PANEL2, fg=GREEN, relief=tk.FLAT,
                                          pady=7, state=tk.DISABLED,
                                          command=self._deploy_export_png)
        self._dep_export_png.pack(side=tk.LEFT, fill=tk.X, expand=True)

    # ── Called from validate tab after CSV is saved ───────────────────────────
    def _deploy_record_sample(self, raw_pred, filt_pred, true_force, t):
        STATE.deploy_raw_pred.append(raw_pred)
        STATE.deploy_filt_pred.append(filt_pred)
        STATE.deploy_true.append(true_force)
        STATE.deploy_timestamps.append(t)

    def _do_deploy_render(self):
        """Load CSV saved by validate tab and render everything."""
        import csv as _csv, os
        path = getattr(self, "_last_val_csv", None)
        if not path or not os.path.exists(path):
            self._log("No saved CSV found — record first in Validate tab.", "err")
            self._send_deploy_btn.configure(state=tk.NORMAL, bg="#1a3a1a")
            return
        try:
            rows = []
            with open(path, newline="") as f:
                for row in _csv.DictReader(f):
                    rows.append(row)
            ts     = np.array([float(r["time_s"])       for r in rows])
            yt     = np.array([float(r["true_force_N"]) for r in rows])
            yp_raw = np.array([float(r["raw_pred_N"])   for r in rows])
            yp_f   = np.array([float(r["filt_pred_N"])  for r in rows])
            self._log("Loaded " + str(len(yt)) + " rows from " + os.path.basename(path), "ok")
        except Exception as e:
            self._log("CSV load error: " + str(e), "err")
            self._send_deploy_btn.configure(state=tk.NORMAL, bg="#1a3a1a")
            return
        if len(yt) < 5:
            self._log("Not enough rows in CSV.", "warn")
            self._send_deploy_btn.configure(state=tk.NORMAL, bg="#1a3a1a")
            return
        self._dep_canvas.get_tk_widget().update_idletasks()
        self._deploy_populate(yp_raw, yp_f, yt, ts)

    def _deploy_populate(self, yp_raw, yp_f, yt, ts):
        """Populate metrics, charts, enable export. Uses yp_f as the real-time series."""
        try:
            rng = yt.max() - yt.min() + 1e-9

            # Offline CV reference (from training)
            r2_cv   = STATE.cv_r2      if STATE.cv_r2      is not None else 0.0
            rmse_cv = STATE.cv_rmse    if STATE.cv_rmse    is not None else 0.0
            r_cv    = STATE.cv_pearson if STATE.cv_pearson is not None else 0.0
            mn_cv   = STATE.cv_model_name or STATE.model_name or "model"

            # Real-time metrics (use exact Validate filtered metrics when available)
            r2_rt = (STATE.live_filt_r2 if STATE.live_filt_r2 is not None
                     else float(r2_score(yt, yp_f)))
            if STATE.live_filt_rmse is not None:
                nrmse_rt = STATE.live_filt_rmse
            else:
                rmse_rt = float(np.sqrt(mean_squared_error(yt, yp_f)))
                nrmse_rt = rmse_rt / rng * 100
            if STATE.live_filt_pearson is not None:
                r_rt = STATE.live_filt_pearson
            else:
                try:
                    r_rt = float(pearsonr(yt, yp_f)[0])
                except Exception:
                    r_rt = 0.0
            delta = r2_rt - r2_cv

            # Store for export
            self._dep_data = {
                "ts": ts, "yt": yt, "yp_raw": yp_raw, "yp_f": yp_f,
                "r2_cv": r2_cv, "rmse_cv": rmse_cv, "r_cv": r_cv, "mn_cv": mn_cv,
                "r2_rt": r2_rt, "nrmse_rt": nrmse_rt, "r_rt": r_rt, "delta": delta,
            }

            # ── Update metric cards ──────────────────────────────────────────
            def col(v, hi, mid): return GREEN if v > hi else YELLOW if v > mid else RED

            self._dep_offline_lbl.configure(
                text="OFFLINE MODEL  (" + mn_cv + ", CV)")
            self._dep_r2.configure(  text=f"{r2_cv:.3f}",  fg=col(r2_cv,  0.80, 0.60))
            self._dep_rmse.configure(text=f"{rmse_cv:.1f}%",
                                      fg=GREEN if rmse_cv < 10 else YELLOW if rmse_cv < 20 else RED)
            self._dep_r.configure(   text=f"{r_cv:.3f}",   fg=col(r_cv,   0.90, 0.75))
            self._dep_n.configure(   text=str(len(yt)),     fg=TEXT)

            self._dep_rt_r2.configure(  text=f"{r2_rt:.3f}",   fg=PURPLE)
            self._dep_rt_rmse.configure(text=f"{nrmse_rt:.1f}%", fg=PURPLE)
            self._dep_rt_r.configure(   text=f"{r_rt:.3f}",    fg=PURPLE)
            self._dep_rt_delta.configure(text=f"{delta:+.3f}",
                                          fg=GREEN if delta >= 0 else RED)

            # ── Draw charts ──────────────────────────────────────────────────
            for ax in [self._dep_ax_ts, self._dep_ax_sc]:
                ax.clear()
                ax.set_facecolor(PANEL2)
                for sp in ax.spines.values(): sp.set_color(BORDER)
                ax.tick_params(colors=MUTED, labelsize=7)

            # Convert to % MVC for display
            mvc_n = force_mvc_reference or 1.0
            yt_pct   = yt   / mvc_n * 100.0
            yp_f_pct = yp_f / mvc_n * 100.0

            # Time-series
            self._dep_ax_ts.plot(ts, yt_pct,   color=GREEN,  lw=1.5,
                                  label="Ground truth", alpha=0.9)
            self._dep_ax_ts.plot(ts, yp_f_pct, color=PURPLE, lw=1.5,
                                  label=f"Real-time  R\u00b2={r2_rt:.3f}")
            self._dep_ax_ts.set_xlabel("Time (s)",   color=MUTED, fontsize=7)
            self._dep_ax_ts.set_ylabel("% MVC",      color=MUTED, fontsize=7)
            self._dep_ax_ts.set_title("Predicted vs Ground Truth",
                                       color=MUTED, fontsize=8)
            self._dep_ax_ts.legend(fontsize=7, labelcolor=TEXT,
                                    facecolor=PANEL2, edgecolor=BORDER)

            # Scatter
            all_v = np.concatenate([yt_pct, yp_f_pct])
            mn, mx = all_v.min(), all_v.max()
            self._dep_ax_sc.scatter(yt_pct, yp_f_pct, s=10, color=PURPLE, alpha=0.6,
                                     label=f"r={r_rt:.3f}")
            self._dep_ax_sc.plot([mn, mx], [mn, mx], color=GREEN,
                                  lw=1.2, linestyle="--", label="Ideal")
            self._dep_ax_sc.set_xlabel("Actual (% MVC)",    color=MUTED, fontsize=7)
            self._dep_ax_sc.set_ylabel("Predicted (% MVC)", color=MUTED, fontsize=7)
            self._dep_ax_sc.set_title("Scatter: Predicted vs Actual",
                                       color=MUTED, fontsize=8)
            self._dep_ax_sc.legend(fontsize=7, labelcolor=TEXT,
                                    facecolor=PANEL2, edgecolor=BORDER)

            self._dep_fig.patch.set_facecolor(BG)
            self._dep_fig.tight_layout()
            self._dep_canvas.draw()

            # ── Status + enable exports ──────────────────────────────────────
            self._dep_status.configure(
                text=f"COMPLETE  \u2014  {len(yt)} windows  |  "
                     f"{ts[-1]:.0f}s  |  R\u00b2={r2_rt:.3f}",
                fg=GREEN)
            self._dep_export_csv.configure(state=tk.NORMAL)
            self._dep_export_png.configure(state=tk.NORMAL)

        except Exception as e:
            import traceback
            self._log("Deploy render error: " + str(e) + "\n" + traceback.format_exc(), "err")

    def _deploy_export_csv(self):
        import os
        d = getattr(self, "_dep_data", None)
        if not d:
            self._log("No data to export.", "warn"); return
        from tkinter import filedialog
        folder = filedialog.askdirectory(title="Select folder to save CSV")
        if not folder: return
        fname = "deploy_" + time.strftime("%Y%m%d_%H%M%S") + ".csv"
        path  = os.path.join(folder, fname)
        import csv as _csv
        try:
            with open(path, "w", newline="") as f:
                w = _csv.writer(f)
                w.writerow(["time_s", "true_force_N", "raw_pred_N", "filt_pred_N"])
                for t, y, rp, fp in zip(d["ts"], d["yt"], d["yp_raw"], d["yp_f"]):
                    w.writerow([f"{t:.4f}", f"{y:.4f}", f"{rp:.4f}", f"{fp:.4f}"])
            # Also write a metrics summary row
            meta_path = os.path.join(folder,
                "deploy_" + time.strftime("%Y%m%d_%H%M%S") + "_metrics.csv")
            with open(meta_path, "w", newline="") as f:
                w = _csv.writer(f)
                w.writerow(["metric", "offline_cv", "realtime_validation"])
                w.writerow(["R2",          f"{d['r2_cv']:.4f}",   f"{d['r2_rt']:.4f}"])
                w.writerow(["pct_RMSE",    f"{d['rmse_cv']:.2f}", f"{d['nrmse_rt']:.2f}"])
                w.writerow(["Pearson_r",   f"{d['r_cv']:.4f}",    f"{d['r_rt']:.4f}"])
                w.writerow(["delta_R2",    "",                     f"{d['delta']:+.4f}"])
                w.writerow(["model",       d["mn_cv"],             "real-time"])
                w.writerow(["n_windows",   "",                     str(len(d["yt"]))])
            self._log("Exported: " + fname, "ok")
            from tkinter import messagebox
            messagebox.showinfo("Saved",
                "Data:    " + fname + "\n"
                "Metrics: " + os.path.basename(meta_path))
        except Exception as e:
            self._log("CSV export error: " + str(e), "err")

    def _deploy_export_png(self):
        import os
        d = getattr(self, "_dep_data", None)
        if not d:
            self._log("No data to export.", "warn"); return
        from tkinter import filedialog
        folder = filedialog.askdirectory(title="Select folder to save PNGs")
        if not folder: return
        base = "deploy_" + time.strftime("%Y%m%d_%H%M%S")
        try:
            # Time-series figure
            _mvc_n = force_mvc_reference or 1.0
            _yt_pct   = np.asarray(d["yt"])   / _mvc_n * 100.0
            _yp_f_pct = np.asarray(d["yp_f"]) / _mvc_n * 100.0
            fig_ts, ax_ts = plt.subplots(figsize=(9, 3.5), facecolor="#0d1117")
            ax_ts.set_facecolor("#161b22")
            ax_ts.plot(d["ts"], _yt_pct,   color="#39d353", lw=1.5,
                       label="Ground Truth (GDX)", alpha=0.9)
            ax_ts.plot(d["ts"], _yp_f_pct, color="#a855f7", lw=1.8,
                       label="Real-Time  R\u00b2=" + f"{d['r2_rt']:.3f}")
            ax_ts.set_xlabel("Time (s)", color="#c9d1d9")
            ax_ts.set_ylabel("Grip Force (% MVC)", color="#c9d1d9")
            ax_ts.set_title(
                "Force Prediction  |  "
                "Offline CV R\u00b2=" + f"{d['r2_cv']:.3f}" +
                "  Real-Time R\u00b2=" + f"{d['r2_rt']:.3f}",
                color="#c9d1d9")
            ax_ts.tick_params(colors="#586069")
            ax_ts.spines[:].set_color("#2a3040")
            ax_ts.legend(facecolor="#161b22", edgecolor="#2a3040", labelcolor="#c9d1d9")
            fig_ts.tight_layout()
            ts_path = os.path.join(folder, base + "_timeseries.png")
            fig_ts.savefig(ts_path, dpi=150, bbox_inches="tight")
            plt.close(fig_ts)

            # Scatter figure
            fig_sc, ax_sc = plt.subplots(figsize=(5, 5), facecolor="#0d1117")
            ax_sc.set_facecolor("#161b22")
            all_v = np.concatenate([_yt_pct, _yp_f_pct])
            mn, mx = float(all_v.min()), float(all_v.max())
            ax_sc.scatter(_yt_pct, _yp_f_pct, s=14, color="#a855f7", alpha=0.65,
                          label="Real-Time  r=" + f"{d['r_rt']:.3f}")
            ax_sc.plot([mn, mx], [mn, mx], color="#39d353",
                       lw=1.2, linestyle="--", label="Ideal (y=x)")
            # Annotation box
            txt = (
                "OFFLINE (" + d["mn_cv"] + "):\n"
                "  R2=" + f"{d['r2_cv']:.3f}" +
                "  r=" + f"{d['r_cv']:.3f}" +
                "  RMSE=" + f"{d['rmse_cv']:.1f}%\n"
                "REAL-TIME:\n"
                "  R2=" + f"{d['r2_rt']:.3f}" +
                "  r=" + f"{d['r_rt']:.3f}" +
                "  RMSE=" + f"{d['nrmse_rt']:.1f}%\n"
                "dR2=" + f"{d['delta']:+.3f}"
            )
            ax_sc.text(0.03, 0.97, txt, transform=ax_sc.transAxes,
                       fontsize=7, verticalalignment="top",
                       fontfamily="monospace", color="#c9d1d9",
                       bbox=dict(boxstyle="round,pad=0.4", facecolor="#161b22",
                                 edgecolor="#2a3040", alpha=0.9))
            ax_sc.set_xlabel("Actual Force (% MVC)", color="#c9d1d9")
            ax_sc.set_ylabel("Predicted Force (% MVC)", color="#c9d1d9")
            ax_sc.set_title("Predicted vs Actual", color="#c9d1d9")
            ax_sc.tick_params(colors="#586069")
            ax_sc.spines[:].set_color("#2a3040")
            ax_sc.legend(facecolor="#161b22", edgecolor="#2a3040", labelcolor="#c9d1d9")
            fig_sc.tight_layout()
            sc_path = os.path.join(folder, base + "_scatter.png")
            fig_sc.savefig(sc_path, dpi=150, bbox_inches="tight")
            plt.close(fig_sc)

            self._log("PNGs saved to " + folder, "ok")
            from tkinter import messagebox
            messagebox.showinfo("Saved",
                os.path.basename(ts_path) + "\n" + os.path.basename(sc_path))
        except Exception as e:
            self._log("PNG export error: " + str(e), "err")

    def _clear_val(self):
        STATE.val_pred.clear()
        STATE.val_true.clear()
        STATE.live_filt_r2 = None
        STATE.live_filt_rmse = None
        STATE.live_filt_pearson = None
        STATE.live_lag_s = None
        STATE.live_filt_lag_s = None
        self._val_pred_line.set_data([], [])
        self._val_true_line.set_data([], [])
        self._val_guide_line.set_data([], [])
        self._val_live_dot.set_data([], [])
        self._val_playhead.set_xdata([-1])
        for ln in self._val_now_span_lines:
            try: ln.remove()
            except Exception: pass
        self._val_now_span_lines.clear()
        for ln in getattr(self, "_val_h_guides", []):
            try: ln.remove()
            except Exception: pass
        self._val_h_guides = []
        for tx in getattr(self, "_val_seg_texts", []):
            try: tx.remove()
            except Exception: pass
        self._val_seg_texts = []
        self._val_ax.set_xlim(0, 1)
        self._val_ax.set_ylim(0, 5)
        self._val_ax.set_title("VALIDATION GUIDE  —  press START VALIDATION to begin",
                                color=MUTED, fontsize=10, loc="left", pad=10)
        self._val_canvas.draw_idle()
        self._popout_sync("validate")
        for w in [self._live_r2, self._live_rmse, self._live_r, self._live_lag]:
            w.configure(text="—")
        for w in [self._filt_r2, self._filt_rmse, self._filt_r, self._filt_lag]:
            w.configure(text="—", fg=PURPLE)

    # ════════════════════════════════════════
    #  DATA MANAGEMENT
    # ════════════════════════════════════════
    def _clear_dataset(self):
        if not messagebox.askyesno("Clear", "Clear all collected data?"): return
        STATE.dataset.clear()
        STATE.level_window_counts = {lvl: 0 for lvl in FORCE_LEVELS}
        for lvl in FORCE_LEVELS: self._level_counts[lvl].set("0")
        self._n_lbl.configure(text="0 WINDOWS")
        self._wins_lbl.configure(text="0")
        self._log("Dataset cleared.", "warn")

    def _export_csv(self):
        if not STATE.dataset:
            self._log("No data to export.", "err"); return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV","*.csv"),("All","*.*")],
            initialfile=f"{self._subject_var.get()}_emg_force.csv")
        if not path: return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            active_feature_names = MR_FEATURE_NAMES if ACTIVE_DEVICE == "mindrove" else FEATURE_NAMES
            w.writerow(["force_N", "force_pct_mvc", "level_pct", "phase", "group_id"] + active_feature_names)
            for d in STATE.dataset:
                w.writerow([d["force"], d.get("force_pct_mvc", ""), d["level"], d.get("phase", "hold"),
                            d.get("group", "")] + d["features"].tolist())
        self._log(f"Exported {len(STATE.dataset)} windows → {os.path.basename(path)}", "ok")

    def _load_csv(self):
        path = filedialog.askopenfilename(filetypes=[("CSV","*.csv"),("All","*.*")])
        if not path: return
        try:
            df = pd.read_csv(path)
            active_feature_names = MR_FEATURE_NAMES if ACTIVE_DEVICE == "mindrove" else FEATURE_NAMES
            # Bump the run id so this file's groups can't collide with groups already
            # in STATE.dataset (from a live session or an earlier loaded file).
            STATE.protocol_run_id += 1
            has_group_col = "group_id" in df.columns
            loaded = 0
            for row_i, row in df.iterrows():
                feats = np.array([float(row.get(f, 0)) for f in active_feature_names])
                raw_group = row.get("group_id") if has_group_col else None
                # Legacy CSVs (no group_id column) have no real contraction grouping
                # recorded — fall back to one group per row rather than guessing.
                group_key = int(raw_group) if pd.notna(raw_group) else row_i
                STATE.dataset.append({
                    "features": feats,
                    "force":    float(row.get("force_N", 0)),
                    "force_pct_mvc": float(row.get("force_pct_mvc")) if pd.notna(row.get("force_pct_mvc", np.nan)) else None,
                    "level":    int(float(row.get("level_pct", 0))),
                    "phase":    str(row.get("phase", "hold")),
                    "group":    STATE.protocol_run_id * 1_000_000 + group_key
                })
                loaded += 1
            self._log(f"Loaded {loaded} windows from {os.path.basename(path)}", "ok")
            self._on_window_added()
        except Exception as e:
            self._log(f"Load error: {e}", "err")

    def _save_model(self):
        if not STATE.model_trained:
            self._log("Train a model first.", "err"); return
        import pickle
        path = filedialog.asksaveasfilename(
            defaultextension=".pkl",
            filetypes=[("Pickle","*.pkl"),("All","*.*")],
            initialfile=f"{self._subject_var.get()}_{STATE.model_name}.pkl")
        if not path: return
        with open(path, "wb") as f:
            active_feature_names = MR_FEATURE_NAMES if ACTIVE_DEVICE == "mindrove" else FEATURE_NAMES
            pickle.dump({"model": STATE.model, "scaler": STATE.scaler,
                         "mvc": mvc_reference, "features": active_feature_names}, f)
        self._log(f"Model saved → {os.path.basename(path)}", "ok")

    def _load_model(self):
        import pickle
        path = filedialog.askopenfilename(filetypes=[("Pickle","*.pkl"),("All","*.*")])
        if not path: return
        try:
            with open(path, "rb") as f:
                d = pickle.load(f)
            global mvc_reference
            STATE.model       = d["model"]
            STATE.scaler      = d["scaler"]
            mvc_reference     = d.get("mvc")
            STATE.model_trained = True
            self._log(f"Model loaded from {os.path.basename(path)}", "ok")
            if mvc_reference is not None:
                self._mvc_status.configure(text="MVC loaded from model ✔", fg=GREEN)
        except Exception as e:
            self._log(f"Load error: {e}", "err")

    # ════════════════════════════════════════
    #  LOGGING
    # ════════════════════════════════════════
    def _log(self, msg, level=""):
        ts = time.strftime("%H:%M:%S")
        colors = {"ok": GREEN, "warn": YELLOW, "err": RED, "info": ACCENT, "": TEXT}
        color  = colors.get(level, TEXT)
        line   = f"[{ts}] {msg}\n"
        try:
            self._log_text.configure(state=tk.NORMAL)
            self._log_text.insert(tk.END, line)
            self._log_text.tag_add(ts, f"end-{len(line)+1}c", "end-1c")
            self._log_text.tag_configure(ts, foreground=color)
            self._log_text.see(tk.END)
            self._log_text.configure(state=tk.DISABLED)
        except Exception:
            pass


    # ════════════════════════════════════════
    #  PERFORMANCE TAB  (05)
    # ════════════════════════════════════════
    def _build_performance_tab(self, parent):
        # Top controls
        top = tk.Frame(parent, bg=PANEL,
                       highlightbackground=BORDER, highlightthickness=1)
        top.pack(fill=tk.X, padx=16, pady=(10,4))

        # Row 1: inputs + record/clear buttons + progress
        row1 = tk.Frame(top, bg=PANEL); row1.pack(fill=tk.X, padx=8, pady=(6,2))
        tk.Label(row1, text="USER", font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack(side=tk.LEFT, padx=(0,2))
        self._perf_user_var = tk.StringVar(value="SUB_001")
        tk.Entry(row1, textvariable=self._perf_user_var, width=10,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER, highlightthickness=1).pack(side=tk.LEFT, padx=(0,6))

        tk.Label(row1, text="SESSION", font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack(side=tk.LEFT, padx=(0,2))
        self._perf_session_var = tk.StringVar(value="")
        tk.Entry(row1, textvariable=self._perf_session_var, width=10,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER, highlightthickness=1).pack(side=tk.LEFT, padx=(0,6))

        tk.Label(row1, text="DURATION (s)", font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack(side=tk.LEFT, padx=(0,2))
        self._perf_dur_var = tk.StringVar(value="45")
        tk.Entry(row1, textvariable=self._perf_dur_var, width=4,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER, highlightthickness=1).pack(side=tk.LEFT, padx=(0,8))

        self._perf_btn = tk.Button(row1, text="▶  RECORD SESSION",
                                   font=("Arial", 10, "bold"), bg=PANEL2, fg=ACCENT,
                                   relief=tk.FLAT, padx=10, pady=4,
                                   command=self._toggle_performance)
        self._perf_btn.pack(side=tk.LEFT, padx=(0,4))

        tk.Button(row1, text="⟳  CLEAR TABLE", font=("Arial",10,"bold"),
                  bg=PANEL2, fg=YELLOW, relief=tk.FLAT, padx=10, pady=4,
                  command=self._clear_perf_sessions).pack(side=tk.LEFT, padx=(0,4))

        # Row 1b: Export button on its own line so it is never clipped
        row1b = tk.Frame(top, bg=PANEL); row1b.pack(fill=tk.X, padx=8, pady=(0,2))
        tk.Button(row1b, text="⇩  EXPORT SUMMARY (MEAN ± STD)", font=("Arial",10,"bold"),
                  bg=PANEL2, fg=GREEN, relief=tk.FLAT, padx=10, pady=4,
                  command=self._perf_export_summary_csv).pack(side=tk.LEFT, padx=(0,4))
        tk.Button(row1b, text="⇩  EXPORT ALL DATA", font=("Arial",10,"bold"),
                  bg=PANEL2, fg=ACCENT, relief=tk.FLAT, padx=10, pady=4,
                  command=self._perf_export_all_data_csv).pack(side=tk.LEFT, padx=(0,4))
        tk.Button(row1b, text="⇩  EXPORT PNG (ALL SESSIONS)", font=("Arial",10,"bold"),
                  bg=PANEL2, fg=PURPLE, relief=tk.FLAT, padx=10, pady=4,
                  command=self._perf_export_png).pack(side=tk.LEFT, padx=(0,4))

        # Row 2: progress indicator on its own line
        row2 = tk.Frame(top, bg=PANEL); row2.pack(fill=tk.X, padx=8, pady=(0,6))
        self._perf_prog_lbl = tk.Label(row2, text="READY", font=FONT_MONO_SM,
                                       fg=MUTED, bg=PANEL, anchor=tk.W)
        self._perf_prog_lbl.pack(side=tk.LEFT, padx=(0,8))
        self._perf_prog_track = tk.Canvas(row2, height=8, bg=BORDER, highlightthickness=0)
        self._perf_prog_track.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,8))
        self._perf_prog_bar = self._perf_prog_track.create_rectangle(0, 0, 0, 8, fill=ACCENT, outline="")

        # Summary metrics
        sum_f = tk.Frame(parent, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        sum_f.pack(fill=tk.X, padx=16, pady=(0,4))
        tk.Label(sum_f, text="OVERALL AVERAGE ACROSS RECORDED SESSIONS",
                 font=("Arial",9,"bold"), fg=MUTED, bg=PANEL).pack(anchor=tk.W, padx=8, pady=(6,2))
        row = tk.Frame(sum_f, bg=PANEL); row.pack(fill=tk.X, padx=8, pady=(0,8))
        self._perf_avg_r2 = self._metric_card(row, "R²", "—", ACCENT)
        self._perf_avg_rmse = self._metric_card(row, "%RMSE", "—", YELLOW)
        self._perf_avg_r = self._metric_card(row, "Pearson r", "—", GREEN)
        self._perf_avg_lag = self._metric_card(row, "Lag (ms)", "—", PURPLE)

        # Current session charts
        perf_overlay_f = tk.Frame(parent, bg=PANEL)
        perf_overlay_f.pack(fill=tk.X, padx=16, pady=(0, 4))
        self._build_perf_overlay_canvas(perf_overlay_f)

        # Session results table
        table_wrap = tk.Frame(parent, bg=BG)
        table_wrap.pack(fill=tk.BOTH, expand=True, padx=16, pady=(0,8))
        tk.Label(table_wrap, text="INDIVIDUAL SESSION RESULTS",
                 font=("Arial",9,"bold"), fg=MUTED, bg=BG).pack(anchor=tk.W)
        cols = ("user", "session", "n", "r2", "rmse", "pearson", "lag")
        self._perf_table = ttk.Treeview(table_wrap, columns=cols, show="headings", height=10)
        for col, title, width in [
            ("user","User",110), ("session","Session",130), ("n","Windows",70),
            ("r2","R²",70), ("rmse","%RMSE",80), ("pearson","Pearson r",90), ("lag","Lag (ms)",80)
        ]:
            self._perf_table.heading(col, text=title)
            self._perf_table.column(col, width=width, anchor=tk.CENTER)
        style = ttk.Style()
        try:
            style.theme_use("default")
        except Exception:
            pass
        style.configure("Treeview", background=PANEL2, fieldbackground=PANEL2, foreground=TEXT, rowheight=24)
        style.configure("Treeview.Heading", background=PANEL, foreground=MUTED)
        self._perf_table.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        perf_sb = ttk.Scrollbar(table_wrap, orient=tk.VERTICAL, command=self._perf_table.yview)
        self._perf_table.configure(yscrollcommand=perf_sb.set)
        perf_sb.pack(side=tk.RIGHT, fill=tk.Y)

    def _perf_record_sample(self, raw_pred, filt_pred, true_force, t):
        STATE.performance_raw_pred.append(raw_pred)
        STATE.performance_filt_pred.append(filt_pred)
        STATE.performance_true.append(true_force)
        STATE.performance_timestamps.append(t)

    def _perf_default_session_id(self):
        return f"session_{len(STATE.performance_sessions) + 1:02d}"

    def _toggle_performance(self):
        if STATE.inferring and not STATE.performance_recording:
            self._log("Stop validation/exoskeleton streaming before recording a performance session.", "warn")
            return
        if not STATE.model_trained:
            self._log("Train a model first.", "err"); return
        if not STATE.connected_emg and not self._sim_mode:
            self._log("Connect EMG (or enable Simulate) first.", "err"); return

        if STATE.performance_recording:
            self._finish_performance_session(stopped_early=True)
            return

        try:
            dur = float(self._perf_dur_var.get())
        except ValueError:
            dur = 20.0
        user_id = (self._perf_user_var.get() or "").strip() or "SUB_001"
        session_id = (self._perf_session_var.get() or "").strip() or self._perf_default_session_id()
        self._perf_user_var.set(user_id)
        self._perf_session_var.set(session_id)

        STATE.performance_raw_pred = []
        STATE.performance_filt_pred = []
        STATE.performance_true = []
        STATE.performance_timestamps = []
        STATE.performance_duration = dur
        STATE.performance_recording = True
        STATE.inferring = True
        STATE._ema_last = None
        self._perf_t0 = time.time()
        self._perf_btn.configure(text="■  STOP SESSION", fg=RED)
        self._perf_prog_lbl.configure(text=f"RECORDING  {user_id} / {session_id}", fg=RED)
        self._perf_prog_track.coords(self._perf_prog_bar, 0, 0, 0, 8)
        self._log(f"Performance session started: {user_id} / {session_id}  ({dur:.0f}s)", "info")

        # Reset chart
        self._perf_pred_line.set_data([], [])
        self._perf_true_line.set_data([], [])
        self._perf_live_dot.set_data([], [])
        self._perf_playhead.set_xdata([-1])
        for ln in self._perf_now_span_lines:
            try: ln.remove()
            except Exception: pass
        self._perf_now_span_lines.clear()

        # Draw guide + lock axes
        self._perf_overlay_draw_guide(dur)
        self._perf_canvas.draw_idle()
        self._popout_sync("performance")

        self._perf_poll_id = self.after(100, self._perf_poll)

    def _perf_poll(self):
        if not STATE.performance_recording:
            return
        elapsed = time.time() - self._perf_t0
        dur = STATE.performance_duration
        pct = min(elapsed / dur, 1.0)
        w = self._perf_prog_track.winfo_width()
        self._perf_prog_track.coords(self._perf_prog_bar, 0, 0, int(pct * w), 8)
        self._perf_prog_lbl.configure(text=f"RECORDING  {elapsed:.1f}s / {dur:.0f}s")

        # ── Stream pred + true directly from perf buffers ─────────────────────
        ts  = list(STATE.performance_timestamps)
        yp_raw = list(STATE.performance_filt_pred if STATE.ema_enabled or STATE.median_enabled
                      else STATE.performance_raw_pred)
        yt_raw = list(STATE.performance_true)
        n   = min(len(ts), len(yp_raw), len(yt_raw))
        if n > 0:
            mvc_n = force_mvc_reference or 1.0
            t_rel = [ts[i] - ts[0] for i in range(n)]
            yp = [v / mvc_n * 100.0 for v in yp_raw[:n]]
            yt = [v / mvc_n * 100.0 for v in yt_raw[:n]]
            self._perf_pred_line.set_data(t_rel, yp)
            self._perf_true_line.set_data(t_rel, yt)
            self._perf_live_dot.set_data([t_rel[-1]], [yt[n-1]])
            data_max = max(max(yp), max(yt))
            if data_max * 1.1 > self._perf_ax.get_ylim()[1]:
                self._perf_ax.set_ylim(0, max(135, data_max * 1.15))

        # ── Playhead + now-band ───────────────────────────────────────────────
        self._perf_playhead.set_xdata([elapsed])
        for ln in self._perf_now_span_lines:
            try: ln.remove()
            except Exception: pass
        self._perf_now_span_lines.clear()
        try:
            band_w = dur * 0.008
            span = self._perf_ax.axvspan(elapsed - band_w, elapsed + band_w,
                                          color=ACCENT, alpha=0.08, zorder=2)
            self._perf_now_span_lines.append(span)
        except Exception:
            pass
        self._perf_canvas.draw_idle()
        self._popout_sync("performance")

        if elapsed >= dur:
            self._finish_performance_session(stopped_early=False)
            return
        self._perf_poll_id = self.after(100, self._perf_poll)

    def _perf_update_current_plot(self):
        ts = np.asarray(STATE.performance_timestamps, dtype=float)
        yt = np.asarray(STATE.performance_true, dtype=float)
        yp = np.asarray(STATE.performance_filt_pred if (STATE.ema_enabled or STATE.median_enabled)
                        else STATE.performance_raw_pred, dtype=float)
        if len(ts) < 2 or len(yt) != len(yp):
            return
        mvc_n = force_mvc_reference or 1.0
        t_rel = ts - ts[0]
        yt_pct = yt / mvc_n * 100.0
        yp_pct = yp / mvc_n * 100.0
        self._perf_pred_line.set_data(t_rel, yp_pct)
        self._perf_true_line.set_data(t_rel, yt_pct)
        self._perf_live_dot.set_data([t_rel[-1]], [yt_pct[-1]])
        allv = np.concatenate([yt_pct, yp_pct])
        self._perf_ax.set_ylim(0, max(float(allv.max()) * 1.1, 135.0))
        self._perf_canvas.draw_idle()
        self._popout_sync("performance")

    def _finish_performance_session(self, stopped_early=False):
        if hasattr(self, "_perf_poll_id"):
            try:
                self.after_cancel(self._perf_poll_id)
            except Exception:
                pass
        STATE.performance_recording = False
        STATE.inferring = False
        self._perf_btn.configure(text="▶  RECORD SESSION", fg=ACCENT)
        self._perf_guide_line.set_data([], [])
        self._perf_live_dot.set_data([], [])
        self._perf_playhead.set_xdata([-1])
        for ln in self._perf_now_span_lines:
            try: ln.remove()
            except Exception: pass
        self._perf_now_span_lines.clear()
        for ln in getattr(self, "_perf_h_guides", []):
            try: ln.remove()
            except Exception: pass
        self._perf_h_guides = []
        for tx in getattr(self, "_perf_seg_texts", []):
            try: tx.remove()
            except Exception: pass
        self._perf_seg_texts = []
        self._perf_ax.set_xlim(0, 1)
        self._perf_ax.set_ylim(0, 5)
        self._perf_ax.set_title("PERFORMANCE GUIDE  —  press RECORD SESSION to begin",
                                 color=MUTED, fontsize=10, loc="left", pad=10)
        self._perf_canvas.draw_idle()
        self._popout_sync("performance")

        ts = np.asarray(STATE.performance_timestamps, dtype=float)
        yt = np.asarray(STATE.performance_true, dtype=float)
        yp_raw = np.asarray(STATE.performance_raw_pred, dtype=float)
        yp_f = np.asarray(STATE.performance_filt_pred, dtype=float)
        used_filtered = bool(STATE.ema_enabled or STATE.median_enabled)
        yp = yp_f if used_filtered else yp_raw

        if len(yt) < 5 or len(yt) != len(yp):
            self._perf_prog_lbl.configure(text="STOPPED — not enough data", fg=YELLOW)
            self._log("Performance session discarded: not enough data.", "warn")
            return

        try:
            r2 = float(r2_score(yt, yp))
            rmse = float(np.sqrt(mean_squared_error(yt, yp)))
            nrmse = rmse / (yt.max() - yt.min() + 1e-9) * 100
            pear = float(pearsonr(yt, yp)[0]) if len(yt) > 2 else 0.0
            lag_s = compute_signal_lag_seconds(yt, yp, ts, max_lag_seconds=0.30)
            lag_ms = None if lag_s is None else lag_s * 1000.0
        except Exception as e:
            self._perf_prog_lbl.configure(text=f"ERROR — {e}", fg=RED)
            self._log(f"Performance session metrics failed: {e}", "err")
            return

        user_id = (self._perf_user_var.get() or "").strip() or "SUB_001"
        session_id = (self._perf_session_var.get() or "").strip() or self._perf_default_session_id()
        summary = {
            "user_id": user_id,
            "session_id": session_id,
            "n_windows": int(len(yt)),
            "r2": r2,
            "pct_rmse": nrmse,
            "pearson_r": pear,
            "lag_ms": lag_ms,
            "used_output": "filtered" if used_filtered else "raw",
        }
        STATE.performance_sessions.append(summary)
        STATE.performance_session_traces.append({
            "ts": ts.copy(), "yt": yt.copy(), "yp_raw": yp_raw.copy(), "yp_f": yp_f.copy(),
            # Snapshot the guide shape as it was AT RECORDING TIME (see the
            # matching comment in the robot-session capture) so a later
            # batch PNG export doesn't redraw old sessions against whatever
            # the effort graph looks like now.
            "guide_t": list(getattr(self, "_overlay_t_path", [])),
            "guide_y": list(getattr(self, "_overlay_y_path", [])),
            "guide_total_t": getattr(self, "_overlay_total_t", 0.0),
        })
        self._perf_append_session_row(summary)
        self._perf_update_average_metrics()
        self._perf_save_session_csv(summary, ts, yt, yp_raw, yp_f)
        self._perf_update_current_plot()

        done_text = "STOPPED EARLY" if stopped_early else "COMPLETE"
        self._perf_prog_lbl.configure(
            text=f"{done_text}  {user_id}/{session_id}  |  R²={r2:.3f}  r={pear:.3f}",
            fg=GREEN if not stopped_early else YELLOW)
        self._log(f"Performance session saved: {user_id}/{session_id}  R²={r2:.3f}  %RMSE={nrmse:.1f}%  r={pear:.3f}  lag={lag_ms if lag_ms is not None else float('nan'):.0f} ms", "ok")

    def _perf_append_session_row(self, summary):
        lag_txt = "—" if summary["lag_ms"] is None else f"{summary['lag_ms']:.0f}"
        self._perf_table.insert("", tk.END, values=(
            summary["user_id"], summary["session_id"], summary["n_windows"],
            f"{summary['r2']:.3f}", f"{summary['pct_rmse']:.1f}",
            f"{summary['pearson_r']:.3f}", lag_txt
        ))

    def _perf_update_average_metrics(self):
        if not STATE.performance_sessions:
            for w in [self._perf_avg_r2, self._perf_avg_rmse, self._perf_avg_r, self._perf_avg_lag]:
                w.configure(text="—")
            return
        arr_r2 = np.array([s["r2"] for s in STATE.performance_sessions], dtype=float)
        arr_rmse = np.array([s["pct_rmse"] for s in STATE.performance_sessions], dtype=float)
        arr_r = np.array([s["pearson_r"] for s in STATE.performance_sessions], dtype=float)
        arr_lag = np.array([s["lag_ms"] for s in STATE.performance_sessions if s["lag_ms"] is not None], dtype=float)
        self._perf_avg_r2.configure(text=f"{arr_r2.mean():.3f}")
        self._perf_avg_rmse.configure(text=f"{arr_rmse.mean():.1f}%")
        self._perf_avg_r.configure(text=f"{arr_r.mean():.3f}")
        self._perf_avg_lag.configure(text=("—" if len(arr_lag)==0 else f"{arr_lag.mean():.0f}"))

    def _clear_perf_sessions(self):
        if not messagebox.askyesno("Clear", "Clear in-memory performance session table?"):
            return
        STATE.performance_sessions.clear()
        STATE.performance_detail_rows.clear()
        STATE.performance_session_traces.clear()
        for item in self._perf_table.get_children():
            self._perf_table.delete(item)
        self._perf_update_average_metrics()
        self._perf_prog_lbl.configure(text="READY", fg=MUTED)

    def _perf_save_session_csv(self, summary, ts, yt, yp_raw, yp_f):
        folder = os.path.join(os.path.expanduser("~"), "Desktop", "PropControl_Performance")
        os.makedirs(folder, exist_ok=True)
        detail_path = os.path.join(folder, "performance_sessions.csv")
        summary_path = os.path.join(folder, "performance_summary.csv")

        detail_exists = os.path.exists(detail_path)
        with open(detail_path, "a", newline="") as f:
            w = csv.writer(f)
            if not detail_exists:
                w.writerow(["user_id", "session_id", "used_output", "time_s",
                            "true_force_N", "raw_pred_N", "filt_pred_N"])
            for t, y, rp, fp in zip(ts, yt, yp_raw, yp_f):
                row = [summary["user_id"], summary["session_id"], summary["used_output"],
                       f"{t:.4f}", f"{y:.4f}", f"{rp:.4f}", f"{fp:.4f}"]
                w.writerow(row)
                STATE.performance_detail_rows.append({
                    "user_id": summary["user_id"],
                    "session_id": summary["session_id"],
                    "used_output": summary["used_output"],
                    "time_s": float(t),
                    "true_force_N": float(y),
                    "raw_pred_N": float(rp),
                    "filt_pred_N": float(fp),
                })

        summary_exists = os.path.exists(summary_path)
        with open(summary_path, "a", newline="") as f:
            w = csv.writer(f)
            if not summary_exists:
                w.writerow(["user_id", "session_id", "used_output", "n_windows",
                            "r2", "pct_rmse", "pearson_r", "lag_ms"])
            w.writerow([summary["user_id"], summary["session_id"], summary["used_output"],
                        summary["n_windows"], f"{summary['r2']:.6f}",
                        f"{summary['pct_rmse']:.6f}", f"{summary['pearson_r']:.6f}",
                        "" if summary["lag_ms"] is None else f"{summary['lag_ms']:.3f}"])

    def _perf_export_summary_csv(self):
        if not STATE.performance_sessions:
            messagebox.showwarning("No data", "No performance sessions recorded yet.")
            return

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        default_name = f"performance_summary_stats_{timestamp}.csv"
        path = filedialog.asksaveasfilename(
            title="Save Performance Summary",
            initialfile=default_name,
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return  # user cancelled

        def _metric_stats(values):
            arr = np.asarray(values, dtype=float)
            if arr.size == 0:
                return "", "", 0
            mean = float(arr.mean())
            std = float(arr.std(ddof=1)) if arr.size > 1 else 0.0
            return mean, std, int(arr.size)

        sessions = STATE.performance_sessions
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["scope", "scope_id", "metric", "mean", "std", "n_sessions"])

            overall_metrics = {
                "r2": [s["r2"] for s in sessions],
                "pct_rmse": [s["pct_rmse"] for s in sessions],
                "pearson_r": [s["pearson_r"] for s in sessions],
                "lag_ms": [s["lag_ms"] for s in sessions if s["lag_ms"] is not None],
            }
            for metric, vals in overall_metrics.items():
                mean, std, n = _metric_stats(vals)
                w.writerow(["overall", "all", metric,
                            "" if n == 0 else f"{mean:.6f}",
                            "" if n == 0 else f"{std:.6f}",
                            n])

            users = sorted(set(s["user_id"] for s in sessions))
            for user_id in users:
                user_sessions = [s for s in sessions if s["user_id"] == user_id]
                user_metrics = {
                    "r2": [s["r2"] for s in user_sessions],
                    "pct_rmse": [s["pct_rmse"] for s in user_sessions],
                    "pearson_r": [s["pearson_r"] for s in user_sessions],
                    "lag_ms": [s["lag_ms"] for s in user_sessions if s["lag_ms"] is not None],
                }
                for metric, vals in user_metrics.items():
                    mean, std, n = _metric_stats(vals)
                    w.writerow(["user", user_id, metric,
                                "" if n == 0 else f"{mean:.6f}",
                                "" if n == 0 else f"{std:.6f}",
                                n])

        self._log(f"Performance summary exported → {os.path.basename(path)}", "ok")
        messagebox.showinfo("Saved", f"Summary statistics saved to:\n{path}")


    def _perf_export_all_data_csv(self):
        """Export all currently recorded performance trial samples to a CSV chosen by the user."""
        if not STATE.performance_detail_rows:
            messagebox.showwarning("No data", "No performance trial samples recorded yet.")
            return

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        default_name = f"performance_all_data_{timestamp}.csv"
        path = filedialog.asksaveasfilename(
            title="Save All Performance Trial Data",
            initialfile=default_name,
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return

        try:
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["user_id", "session_id", "used_output", "time_s",
                            "true_force_N", "raw_pred_N", "filt_pred_N"])
                for row in STATE.performance_detail_rows:
                    w.writerow([
                        row["user_id"],
                        row["session_id"],
                        row["used_output"],
                        f"{row['time_s']:.4f}",
                        f"{row['true_force_N']:.4f}",
                        f"{row['raw_pred_N']:.4f}",
                        f"{row['filt_pred_N']:.4f}",
                    ])
            self._log(f"Performance all-data export → {os.path.basename(path)}", "ok")
            messagebox.showinfo("Saved", f"All performance trial data saved to:\n{path}")
        except Exception as e:
            self._log(f"Performance all-data export error: {e}", "err")
            messagebox.showerror("Export failed", str(e))

    def _perf_export_png(self):
        """Export a PNG of every recorded performance session in one go.

        Each session is auto-captured (as raw ts/yt/yp arrays, not a
        rendered image) the moment it finishes, in
        STATE.performance_session_traces — so this just batch-renders all
        of them now. Each PNG always shows the full target guide + ground
        truth + prediction regardless of what's currently toggled visible
        on screen, since the point is a complete record to draw
        conclusions from later.
        """
        if not STATE.performance_sessions:
            messagebox.showwarning("No data", "No performance sessions recorded yet.")
            return

        out_dir = filedialog.askdirectory(title="Choose folder to export session PNGs")
        if not out_dir:
            return

        used_names = set()
        n_saved = 0
        for summary, trace in zip(STATE.performance_sessions, STATE.performance_session_traces):
            base = f"{summary['user_id']}_{summary['session_id']}"
            base = "".join(c if (c.isalnum() or c in "_-") else "_" for c in base)
            name = base
            i = 2
            while name in used_names:
                name = f"{base}_{i}"; i += 1
            used_names.add(name)
            try:
                self._perf_render_session_png(summary, trace,
                                               os.path.join(out_dir, name + ".png"))
                n_saved += 1
            except Exception as e:
                self._log(f"PNG export failed for {base}: {e}", "err")

        self._log(f"Exported {n_saved} performance session PNG(s) → {out_dir}", "ok")
        messagebox.showinfo("Export complete",
                             f"Saved {n_saved} session PNG(s) to:\n{out_dir}")

    def _perf_render_session_png(self, summary, trace, path):
        """Render one session's full graph (target guide + ground truth +
        prediction) as a standalone PNG — independent of the live canvas,
        so it's unaffected by the PREDICTION/GRIP READING toggle state."""
        ts     = trace["ts"]
        yt     = trace["yt"]
        yp_raw = trace["yp_raw"]
        yp_f   = trace["yp_f"]
        yp     = yp_f if summary["used_output"] == "filtered" else yp_raw
        # Guide shape as captured AT RECORDING TIME for this session — see
        # the matching comment where performance_session_traces is built.
        guide_t = trace.get("guide_t", [])
        guide_y = trace.get("guide_y", [])
        guide_total_t = trace.get("guide_total_t", 0.0)

        mvc_n  = force_mvc_reference or 1.0
        yt_pct = yt / mvc_n * 100.0
        yp_pct = yp / mvc_n * 100.0

        fig = Figure(figsize=(11, 5.5), facecolor=PANEL)
        fig.subplots_adjust(top=0.86, bottom=0.14, left=0.08, right=0.97)
        ax = fig.add_subplot(111)
        ax.set_facecolor(PANEL2)
        for sp in ax.spines.values():
            sp.set_color(BORDER); sp.set_linewidth(0.8)
        ax.tick_params(colors=MUTED, labelsize=11)
        ax.set_xlabel("Time (s)", color=MUTED, fontsize=12, labelpad=6)
        ax.set_ylabel("% MVC", color=MUTED, fontsize=12, labelpad=8)

        if len(guide_t):
            ax.plot(guide_t, guide_y, color=RED, lw=3.0,
                    zorder=3, label="Target", solid_capstyle="round")
            for pct in [25, 50, 75, 100]:
                ax.axhline(pct, color=MUTED, lw=0.5, linestyle=":", zorder=1, alpha=0.5)

        ax.plot(ts, yt_pct, color=GREEN, lw=2.2, zorder=4,
                label="Ground truth", alpha=0.9, solid_capstyle="round")
        ax.plot(ts, yp_pct, color=PURPLE, lw=2.2, zorder=5,
                label="Predicted", alpha=0.9, solid_capstyle="round")

        x_max = max(float(ts.max()) if len(ts) else 1.0, guide_total_t)
        y_max = max(float(yt_pct.max()) if len(yt_pct) else 0.0,
                    float(yp_pct.max()) if len(yp_pct) else 0.0, 100.0)
        ax.set_xlim(0, x_max * 1.02)
        ax.set_ylim(0, y_max * 1.15)

        lag_txt = "—" if summary["lag_ms"] is None else f"{summary['lag_ms']:.0f} ms"
        ax.set_title(
            f"{summary['user_id']} / {summary['session_id']}   "
            f"R²={summary['r2']:.3f}  %RMSE={summary['pct_rmse']:.1f}%  "
            f"r={summary['pearson_r']:.3f}  lag={lag_txt}  "
            f"({summary['n_windows']} windows, {summary['used_output']} output)",
            color=TEXT, fontsize=12, loc="left", pad=12, fontweight="bold")

        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.10),
                  ncol=3, fontsize=11, labelcolor=TEXT,
                  facecolor=PANEL2, edgecolor=BORDER, framealpha=0.85,
                  borderpad=0.8, handlelength=2.0, columnspacing=2.0)

        fig.savefig(path, dpi=150, facecolor=PANEL)

    # ════════════════════════════════════════
    #  EXOSKELETON TAB  (06)
    # ════════════════════════════════════════


    def _build_exo_tab(self, parent):
        """Tab 05 — integrated exoskeleton + grip force monitor.
        Sends only CURRENT COMMANDS to the OpenRB-150; the Arduino handles
        micro-open release logic internally.
        """

        # ── Header ────────────────────────────────────────────────────────
        hdr = tk.Frame(parent, bg=PANEL,
                       highlightbackground=BORDER, highlightthickness=1)
        hdr.pack(fill=tk.X, padx=16, pady=(12, 4))
        tk.Label(hdr, text="EXOSKELETON  —  Predicted Force → Current Control",
                 font=("Arial", 10, "bold"), fg=MUTED, bg=PANEL
                 ).pack(anchor=tk.W, padx=10, pady=(8, 2))
        tk.Label(
            hdr,
            text="Uses the trained EMG model to predict grip force, maps that prediction "
                 "to a current command, and sends only the current over serial. "
                 "The Arduino/OpenRB sketch handles the micro-open release behavior internally. "
                 "Live predicted force, GDX grip force, commanded current, and measured current are "
                 "shown together below.",
            font=FONT_MONO_SM, fg=MUTED, bg=PANEL,
            wraplength=920, justify=tk.LEFT
        ).pack(anchor=tk.W, padx=10, pady=(0, 8))

        # Internal rolling history
        self._exo_hist_len = 400
        self._exo_t_hist = collections.deque(maxlen=self._exo_hist_len)
        self._exo_pred_hist = collections.deque(maxlen=self._exo_hist_len)
        self._exo_true_hist = collections.deque(maxlen=self._exo_hist_len)
        self._exo_cmd_hist = collections.deque(maxlen=self._exo_hist_len)
        self._exo_meas_hist = collections.deque(maxlen=self._exo_hist_len)
        self._exo_goalpos_hist = collections.deque(maxlen=self._exo_hist_len)
        self._exo_presentpos_hist = collections.deque(maxlen=self._exo_hist_len)

        # Latest values
        self._exo_latest_goal_ma = 0.0
        self._exo_latest_present_ma = 0.0
        self._exo_latest_goal_pos = 0
        self._exo_latest_present_pos = 0
        self._latest_pred_force_raw = 0.0
        self._latest_pred_force_filt = 0.0
        self._latest_true_force = 0.0

        # Reader thread state
        self._exo_rx_running = False
        self._exo_rx_thread = None
        self._exo_plot_after = None
        self._exo_stream_t0 = None
        self._exo_write_lock = threading.Lock()

        # ── Connection row ────────────────────────────────────────────────
        conn_f = tk.Frame(parent, bg=PANEL,
                          highlightbackground=BORDER, highlightthickness=1)
        conn_f.pack(fill=tk.X, padx=16, pady=(4, 4))
        tk.Label(conn_f, text="CONNECTIONS",
                 font=("Arial", 9, "bold"), fg=MUTED, bg=PANEL
                 ).pack(anchor=tk.W, padx=10, pady=(6, 4))

        row = tk.Frame(conn_f, bg=PANEL)
        row.pack(fill=tk.X, padx=10, pady=(0, 8))

        self._exo_port_var = tk.StringVar(value="-- scan first --")
        self._exo_baud_var = tk.StringVar(value="115200")

        self._exo_port_menu = tk.OptionMenu(row, self._exo_port_var, "-- scan first --")
        self._exo_port_menu.configure(bg=PANEL2, fg=TEXT, relief=tk.FLAT,
                                      font=FONT_MONO_SM, width=24,
                                      highlightbackground=BORDER,
                                      activebackground=PANEL2)
        self._exo_port_menu["menu"].configure(bg=PANEL2, fg=TEXT, font=FONT_MONO_SM)
        self._exo_port_menu.pack(side=tk.LEFT, padx=(0, 8))

        tk.Button(row, text="⟳  SCAN", font=("Arial", 9, "bold"),
                  bg=PANEL2, fg=YELLOW, relief=tk.FLAT, padx=10, pady=5,
                  command=self._exo_scan_ports).pack(side=tk.LEFT, padx=(0, 12))

        tk.Label(row, text="BAUD", font=FONT_MONO_SM, fg=MUTED, bg=PANEL
                 ).pack(side=tk.LEFT, padx=(0, 4))
        baud_menu = tk.OptionMenu(row, self._exo_baud_var, "57600", "115200", "1000000")
        baud_menu.configure(bg=PANEL2, fg=TEXT, relief=tk.FLAT,
                            font=FONT_MONO_SM, width=8,
                            highlightbackground=BORDER,
                            activebackground=PANEL2)
        baud_menu["menu"].configure(bg=PANEL2, fg=TEXT, font=FONT_MONO_SM)
        baud_menu.pack(side=tk.LEFT, padx=(0, 12))

        self._exo_connect_btn = tk.Button(
            row, text="⬡  CONNECT EXO",
            font=("Arial", 10, "bold"), bg=PANEL2, fg=ACCENT,
            relief=tk.FLAT, padx=12, pady=6,
            command=self._exo_toggle_connect)
        self._exo_connect_btn.pack(side=tk.LEFT, padx=(0, 8))

        self._exo_gdx_btn = tk.Button(
            row, text="⬡  CONNECT GDX",
            font=("Arial", 10, "bold"), bg=PANEL2, fg=GREEN,
            relief=tk.FLAT, padx=12, pady=6,
            command=self._exo_connect_gdx)
        self._exo_gdx_btn.pack(side=tk.LEFT)

        # ── Status row ────────────────────────────────────────────────────
        status_f = tk.Frame(parent, bg=BG)
        status_f.pack(fill=tk.X, padx=16, pady=(2, 4))
        self._exo_status_lbl = tk.Label(
            status_f, text="●  DISCONNECTED",
            font=("Arial", 11, "bold"), fg=RED, bg=BG)
        self._exo_status_lbl.pack(side=tk.LEFT)
        self._exo_tx_lbl = tk.Label(
            status_f, text="",
            font=FONT_MONO_SM, fg=MUTED, bg=BG)
        self._exo_tx_lbl.pack(side=tk.LEFT, padx=(16, 0))

        # ── Mapping / stream controls ─────────────────────────────────────
        ctrl_f = tk.Frame(parent, bg=PANEL,
                          highlightbackground=BORDER, highlightthickness=1)
        ctrl_f.pack(fill=tk.X, padx=16, pady=(4, 4))
        tk.Label(ctrl_f, text="FORCE → CURRENT MAPPING",
                 font=("Arial", 9, "bold"), fg=MUTED, bg=PANEL
                 ).pack(anchor=tk.W, padx=10, pady=(6, 4))

        row1 = tk.Frame(ctrl_f, bg=PANEL)
        row1.pack(fill=tk.X, padx=10, pady=(0, 4))

        tk.Label(row1, text="MVC force (N)",
                 font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack(side=tk.LEFT)
        self._exo_mvc_force_var = tk.StringVar(value="200.0")
        tk.Entry(row1, textvariable=self._exo_mvc_force_var, width=8,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER,
                 highlightthickness=1).pack(side=tk.LEFT, padx=(6, 12))

        tk.Label(row1, text="Max current (mA)",
                 font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack(side=tk.LEFT)
        self._exo_max_current_var = tk.StringVar(value="1000")
        tk.Entry(row1, textvariable=self._exo_max_current_var, width=6,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER,
                 highlightthickness=1).pack(side=tk.LEFT, padx=(6, 12))

        tk.Label(row1, text="Rate (Hz)", font=FONT_MONO_SM, fg=MUTED, bg=PANEL
                 ).pack(side=tk.LEFT)
        self._exo_rate_var = tk.IntVar(value=20)
        tk.Scale(row1, variable=self._exo_rate_var,
                 from_=1, to=50, resolution=1,
                 orient=tk.HORIZONTAL, length=110,
                 bg=PANEL, fg=TEXT, troughcolor=PANEL2,
                 highlightthickness=0, bd=0, showvalue=0
                 ).pack(side=tk.LEFT, padx=(6, 4))
        self._exo_rate_lbl = tk.Label(row1, text="20 Hz",
                                      font=FONT_MONO_SM, fg=ACCENT, bg=PANEL, width=6)
        self._exo_rate_lbl.pack(side=tk.LEFT)
        self._exo_rate_var.trace_add("write", lambda *_: self._exo_rate_lbl.configure(
            text=f"{self._exo_rate_var.get()} Hz"))

        row2 = tk.Frame(ctrl_f, bg=PANEL)
        row2.pack(fill=tk.X, padx=10, pady=(0, 8))

        self._exo_use_filtered_var = tk.BooleanVar(value=True)
        tk.Checkbutton(row2, text="Use filtered prediction",
                       variable=self._exo_use_filtered_var,
                       font=FONT_MONO_SM, bg=PANEL, fg=MUTED,
                       selectcolor=PANEL2, activebackground=PANEL,
                       activeforeground=TEXT).pack(side=tk.LEFT)

        self._exo_stream_btn = tk.Button(
            row2, text="▶  START STREAM",
            font=("Arial", 10, "bold"), bg=PANEL2, fg=ACCENT,
            relief=tk.FLAT, padx=14, pady=7, state=tk.DISABLED,
            command=self._exo_toggle_stream)
        self._exo_stream_btn.pack(side=tk.LEFT, padx=(16, 8))

        tk.Label(row2, text="Protocol: sends integer current only",
                 font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack(side=tk.LEFT)

        # ── Metric cards ──────────────────────────────────────────────────
        met_f = tk.Frame(parent, bg=PANEL,
                         highlightbackground=BORDER, highlightthickness=1)
        met_f.pack(fill=tk.X, padx=16, pady=(4, 4))
        tk.Label(met_f, text="LIVE EXOSKELETON",
                 font=("Arial", 9, "bold"), fg=MUTED, bg=PANEL
                 ).pack(anchor=tk.W, padx=10, pady=(6, 2))
        cards_row = tk.Frame(met_f, bg=PANEL)
        cards_row.pack(fill=tk.X, padx=10, pady=(0, 8))
        self._exo_pred_card = self._metric_card(cards_row, "Predicted (N)", "—", ACCENT)
        self._exo_true_card = self._metric_card(cards_row, "GDX Force (N)", "—", GREEN)
        self._exo_sent_card = self._metric_card(cards_row, "Cmd Current", "—", ACCENT2)
        self._exo_meas_card = self._metric_card(cards_row, "Measured mA", "—", YELLOW)
        self._exo_tx_count = self._metric_card(cards_row, "Packets Sent", "0", MUTED)
        self._exo_err_card = self._metric_card(cards_row, "TX Errors", "0", RED)

        # ── Combined graph ────────────────────────────────────────────────
        fig_wrap = tk.Frame(parent, bg=BG)
        fig_wrap.pack(fill=tk.BOTH, expand=True, padx=16, pady=(4, 4))

        self._exo_fig = Figure(figsize=(8.2, 3.8), facecolor=BG, tight_layout=True)
        self._exo_ax_force = self._exo_fig.add_subplot(111)
        self._exo_ax_current = self._exo_ax_force.twinx()

        self._exo_ax_force.set_facecolor(PANEL2)
        self._exo_ax_current.set_facecolor("none")
        for sp in self._exo_ax_force.spines.values():
            sp.set_color(BORDER)
        for sp in self._exo_ax_current.spines.values():
            sp.set_color(BORDER)

        self._exo_ax_force.tick_params(colors=MUTED, labelsize=7)
        self._exo_ax_current.tick_params(colors=MUTED, labelsize=7)
        self._exo_ax_force.set_xlabel("Time (s)", color=MUTED, fontsize=8)
        self._exo_ax_force.set_ylabel("% MVC", color=GREEN, fontsize=8)
        self._exo_ax_current.set_ylabel("Current (mA)", color=ACCENT2, fontsize=8)
        self._exo_ax_force.set_title("Predicted force, measured grip force, and motor current",
                                     color=MUTED, fontsize=8)
        self._exo_ax_force.grid(True, color=BORDER, alpha=0.5)

        self._exo_force_pred_line, = self._exo_ax_force.plot([], [], color=ACCENT, lw=1.6, label="Predicted (% MVC)")
        self._exo_force_true_line, = self._exo_ax_force.plot([], [], color=GREEN, lw=1.6, label="GDX (% MVC)")
        self._exo_cmd_line, = self._exo_ax_current.plot([], [], color=ACCENT2, lw=1.6, label="Cmd mA")
        self._exo_meas_line, = self._exo_ax_current.plot([], [], color=YELLOW, lw=1.2, label="Measured mA")

        lines = [self._exo_force_pred_line, self._exo_force_true_line, self._exo_cmd_line, self._exo_meas_line]
        labels = [ln.get_label() for ln in lines]
        self._exo_ax_force.legend(lines, labels, loc="upper right",
                                  fontsize=7, labelcolor=TEXT,
                                  facecolor=PANEL2, edgecolor=BORDER)

        self._exo_canvas = FigureCanvasTkAgg(self._exo_fig, master=fig_wrap)
        self._exo_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # ── Serial console ────────────────────────────────────────────────
        log_f = tk.Frame(parent, bg=BG)
        log_f.pack(fill=tk.BOTH, expand=False, padx=16, pady=(4, 8))
        tk.Label(log_f, text="TX / RX LOG",
                 font=("Arial", 9, "bold"), fg=MUTED, bg=BG
                 ).pack(anchor=tk.W)
        self._exo_log = tk.Text(log_f, height=7, bg=PANEL2, fg=MUTED,
                                font=("Courier New", 8), state=tk.DISABLED,
                                relief=tk.FLAT, wrap=tk.WORD,
                                highlightbackground=BORDER, highlightthickness=1)
        self._exo_log.pack(fill=tk.BOTH, expand=True)

        # Existing exo state
        self._exo_serial = None
        self._exo_streaming = False
        self._exo_tx_total = 0
        self._exo_err_total = 0
        self._exo_last_force = 0.0
        self._exo_stream_after = None

    def _exo_log_line(self, msg, color=None):
        import time as _time
        ts = _time.strftime("%H:%M:%S")
        line = f"[{ts}]  {msg}\n"
        try:
            self._exo_log.configure(state=tk.NORMAL)
            tag = ts + str(id(msg))
            self._exo_log.insert(tk.END, line, tag)
            if color:
                self._exo_log.tag_configure(tag, foreground=color)
            lines = int(self._exo_log.index("end-1c").split(".")[0])
            if lines > 200:
                self._exo_log.delete("1.0", f"{lines-200}.0")
            self._exo_log.see(tk.END)
            self._exo_log.configure(state=tk.DISABLED)
        except Exception:
            pass

    def _exo_scan_ports(self):
        try:
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
        except Exception as e:
            self._exo_log_line(f"Scan error: {e}", RED)
            return

        menu = self._exo_port_menu["menu"]
        menu.delete(0, "end")
        found = []
        for p in ports:
            desc = (p.description or "").lower()
            hwid = (p.hwid or "").lower()
            is_openrb = ("openrb" in desc or "2f5d" in hwid or
                         "robotis" in desc or "openrb-150" in desc)
            label = p.device
            if is_openrb:
                label += "  ← OpenRB-150"
            menu.add_command(label=label,
                             command=lambda v=p.device: self._exo_port_var.set(v))
            found.append((p.device, is_openrb))

        if not found:
            self._exo_port_var.set("-- no ports found --")
            self._exo_log_line("No serial ports found.", YELLOW)
            return

        openrb_ports = [dev for dev, flag in found if flag]
        if openrb_ports:
            self._exo_port_var.set(openrb_ports[0])
            self._exo_log_line(f"Detected OpenRB-150: {openrb_ports[0]}", GREEN)
        else:
            self._exo_port_var.set(found[0][0])
            self._exo_log_line(f"No OpenRB-150 signature found. Defaulting to {found[0][0]}", YELLOW)

        for dev, flag in found:
            tag = "  [OpenRB-150]" if flag else ""
            self._exo_log_line(f"  {dev}{tag}", GREEN if flag else None)

    def _exo_toggle_connect(self):
        if self._exo_serial and self._exo_serial.is_open:
            self._exo_disconnect()
        else:
            self._exo_connect()

    def _exo_connect_gdx(self):
        if STATE.connected_gdx:
            self._exo_log_line("GDX already connected.", GREEN)
            self._exo_gdx_btn.configure(text="✔  GDX CONNECTED", fg=GREEN)
            return

        self._exo_gdx_btn.configure(text="Connecting...", state=tk.DISABLED)
        self.update()

        def worker():
            ok, msg = setup_gdx()
            def finish():
                if ok:
                    STATE.connected_gdx = True
                    self._set_dot(self._gdx_dot, GREEN)
                    self._gdx_lbl.configure(text="GDX: LIVE")
                    self._exo_gdx_btn.configure(text="✔  GDX CONNECTED", fg=GREEN, state=tk.NORMAL)
                    self._exo_log_line("GDX connected.", GREEN)
                    # setup_gdx() opens both scales if both are plugged in;
                    # refresh the robot tab's status display to match.
                    self._robot_refresh_gdx_status()
                else:
                    self._exo_gdx_btn.configure(text="⬡  CONNECT GDX", fg=GREEN, state=tk.NORMAL)
                    self._exo_log_line(f"GDX failed: {msg}", RED)
            self.after(0, finish)

        threading.Thread(target=worker, daemon=True).start()

    def _exo_open_serial(self, port, baud):
        import serial as _serial
        ser = _serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 0.05
        ser.write_timeout = 1.0
        ser.dsrdtr = False
        ser.rtscts = False
        ser.open()
        time.sleep(2.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        deadline = time.time() + 5.0
        ready_seen = False
        consecutive_empties = 0
        while time.time() < deadline:
            line = ser.readline().decode(errors="ignore").strip()
            if line == "READY":
                ready_seen = True
                break
            if line == "":
                consecutive_empties += 1
                if consecutive_empties >= 2:
                    break
            else:
                consecutive_empties = 0

        ser.reset_input_buffer()
        return ser, ready_seen

    def _exo_parse_rx_line(self, line):
        s = line.strip()
        if not s:
            return None
        parts = s.split(",")
        # Supports:
        # DATA,goal,present
        # DATA,goal,present,GOALPOS,x,PRESENTPOS,y
        if len(parts) >= 3 and parts[0] == "DATA":
            try:
                goal = float(parts[1])
                present = float(parts[2])
                goalpos = None
                presentpos = None
                if len(parts) >= 7:
                    for i in range(3, len(parts)-1, 2):
                        key = parts[i]
                        val = parts[i+1]
                        if key == "GOALPOS":
                            goalpos = int(float(val))
                        elif key == "PRESENTPOS":
                            presentpos = int(float(val))
                return {"goal": goal, "present": present, "goalpos": goalpos, "presentpos": presentpos}
            except Exception:
                return None
        return None

    def _exo_rx_loop(self):
        while self._exo_rx_running and self._exo_serial and self._exo_serial.is_open:
            try:
                raw = self._exo_serial.readline().decode(errors="ignore").strip()
            except Exception:
                break
            if not raw:
                continue

            parsed = self._exo_parse_rx_line(raw)
            if parsed is None:
                # Show occasional non-DATA lines like READY / STOPPED / debug
                if raw not in ("READY",):
                    self.after(0, lambda msg=raw: self._exo_log_line(f"RX  {msg}", MUTED))
                continue

            t = 0.0
            if self._exo_stream_t0 is not None:
                t = time.time() - self._exo_stream_t0

            self._exo_latest_goal_ma = parsed["goal"]
            self._exo_latest_present_ma = parsed["present"]
            if parsed["goalpos"] is not None:
                self._exo_latest_goal_pos = parsed["goalpos"]
            if parsed["presentpos"] is not None:
                self._exo_latest_present_pos = parsed["presentpos"]

            self._exo_meas_hist.append(parsed["present"])
            if len(self._exo_t_hist) < len(self._exo_meas_hist):
                # keep lengths aligned when RX comes in before TX tick
                self._exo_t_hist.append(t)
                self._exo_pred_hist.append(self._latest_pred_force_filt if self._exo_use_filtered_var.get()
                                           else self._latest_pred_force_raw)
                self._exo_true_hist.append(_force_to_pct_mvc(_gdx_last_force[0]))
                self._exo_cmd_hist.append(self._exo_last_cmd_ma if hasattr(self, "_exo_last_cmd_ma") else 0.0)

            if parsed["goalpos"] is not None:
                self._exo_goalpos_hist.append(parsed["goalpos"])
            if parsed["presentpos"] is not None:
                self._exo_presentpos_hist.append(parsed["presentpos"])

    def _exo_connect(self):
        port = self._exo_port_var.get()
        if not port or "scan" in port.lower() or "no ports" in port.lower():
            self._exo_log_line("Select a port first (use ⟳ SCAN).", RED)
            return
        port = port.split("  ←")[0].strip()
        try:
            baud = int(self._exo_baud_var.get())
        except ValueError:
            baud = 115200

        try:
            ser, ready_seen = self._exo_open_serial(port, baud)
            self._exo_serial = ser
            self._exo_log_line(f"Connected to {port} @ {baud} baud" +
                               ("  [READY]" if ready_seen else ""), GREEN)
            self._exo_status_lbl.configure(
                text=f"●  CONNECTED  {port} @ {baud}", fg=GREEN)
            self._exo_connect_btn.configure(text="◈  DISCONNECT EXO", fg=RED)
            self._exo_stream_btn.configure(state=tk.NORMAL)
            self._exo_rx_running = True
            self._exo_rx_thread = threading.Thread(target=self._exo_rx_loop, daemon=True)
            self._exo_rx_thread.start()
            self._exo_plot_loop()
        except Exception as e:
            self._exo_log_line(f"Connection failed: {e}", RED)
            self._exo_status_lbl.configure(text="●  CONNECTION FAILED", fg=RED)

    def _exo_disconnect(self):
        if self._exo_streaming:
            self._exo_stop_stream()

        self._exo_rx_running = False
        try:
            if self._exo_serial and self._exo_serial.is_open:
                self._exo_serial.close()
        except Exception:
            pass
        self._exo_serial = None
        self._exo_status_lbl.configure(text="●  DISCONNECTED", fg=RED)
        self._exo_connect_btn.configure(text="⬡  CONNECT EXO", fg=ACCENT)
        self._exo_stream_btn.configure(state=tk.DISABLED, text="▶  START STREAM", fg=ACCENT)
        self._exo_log_line("Disconnected.", YELLOW)

    def _exo_toggle_stream(self):
        if self._exo_streaming:
            self._exo_stop_stream()
        else:
            self._exo_start_stream()

    def _exo_force_to_current(self, force_n):
        try:
            mvc_force = float(self._exo_mvc_force_var.get())
        except Exception:
            mvc_force = 200.0
        try:
            max_current = int(float(self._exo_max_current_var.get()))
        except Exception:
            max_current = 1000

        force_n = max(0.0, float(force_n))
        mvc_force = max(1e-6, mvc_force)
        max_current = max(1, max_current)

        current = int(round((force_n / mvc_force) * max_current))
        current = max(0, min(max_current, current))
        return current

    def _exo_start_stream(self):
        if not STATE.model_trained:
            self._exo_log_line("Train a model first (02 TRAIN).", RED)
            return
        if not (self._exo_serial and self._exo_serial.is_open):
            self._exo_log_line("Connect to OpenRB-150 first.", RED)
            return
        if not STATE.connected_gdx:
            self._exo_log_line("Connect GDX first so you can see real grip force.", YELLOW)

        # Make sure live inference is running, otherwise the Exo tab
        # will just keep showing the last cached prediction.
        if not STATE.inferring:
            STATE.inferring = True
            STATE._ema_last = None
            self._exo_log_line(
                "Inference auto-started so predicted force streams live.",
                ACCENT
            )

        self._exo_streaming = True
        self._exo_tx_total = 0
        self._exo_err_total = 0
        self._exo_last_cmd_ma = 0
        self._exo_stream_t0 = time.time()

        self._exo_t_hist.clear()
        self._exo_pred_hist.clear()
        self._exo_true_hist.clear()
        self._exo_cmd_hist.clear()
        self._exo_meas_hist.clear()
        self._exo_goalpos_hist.clear()
        self._exo_presentpos_hist.clear()

        self._exo_stream_btn.configure(text="■  STOP STREAM", fg=RED)
        self._exo_status_lbl.configure(
            text=f"●  STREAMING  @ {self._exo_rate_var.get()} Hz", fg=GREEN)
        self._exo_log_line(
            f"Stream started @ {self._exo_rate_var.get()} Hz  |  "
            f"{self._exo_mvc_force_var.get()} N → {self._exo_max_current_var.get()} mA",
            GREEN)
        self._exo_stream_tick()

    def _exo_stop_stream(self):
        self._exo_streaming = False
        if self._exo_stream_after:
            try:
                self.after_cancel(self._exo_stream_after)
            except Exception:
                pass
            self._exo_stream_after = None

        try:
            if self._exo_serial and self._exo_serial.is_open:
                with self._exo_write_lock:
                    self._exo_serial.write(b"STOP\n")
        except Exception:
            pass

        self._exo_stream_btn.configure(text="▶  START STREAM", fg=ACCENT)
        port = self._exo_port_var.get().split("  ←")[0].strip()
        self._exo_status_lbl.configure(
            text=f"●  CONNECTED  {port}  (stream stopped)", fg=YELLOW)
        self._exo_log_line(
            f"Stream stopped. Total sent: {self._exo_tx_total}  Errors: {self._exo_err_total}",
            YELLOW)

    def _exo_stream_tick(self):
        if not self._exo_streaming:
            return

        force_raw = float(getattr(self, "_latest_pred_force_raw", 0.0))
        force_filt = float(getattr(self, "_latest_pred_force_filt", force_raw))
        force_used = force_filt if self._exo_use_filtered_var.get() else force_raw
        force_used = max(0.0, force_used)

        current_cmd = self._exo_force_to_current(force_used)
        self._exo_last_force = force_used
        self._exo_last_cmd_ma = current_cmd

        # UI cards — show % MVC
        self._exo_pred_card.configure(text=f"{_force_to_pct_mvc(force_used):.1f}%")
        self._exo_true_card.configure(text=f"{_force_to_pct_mvc(_gdx_last_force[0]):.1f}%")
        self._exo_sent_card.configure(text=f"{current_cmd:d}")
        self._exo_meas_card.configure(text=f"{self._exo_latest_present_ma:.0f}")
        self._exo_tx_count.configure(text=str(self._exo_tx_total))
        self._exo_err_card.configure(text=str(self._exo_err_total))

        # Record history — force in % MVC for the chart
        t = time.time() - self._exo_stream_t0 if self._exo_stream_t0 is not None else 0.0
        self._exo_t_hist.append(t)
        self._exo_pred_hist.append(_force_to_pct_mvc(force_used))
        self._exo_true_hist.append(_force_to_pct_mvc(_gdx_last_force[0]))
        self._exo_cmd_hist.append(current_cmd)

        packet = f"{current_cmd}\n"
        try:
            if self._exo_serial and self._exo_serial.is_open:
                with self._exo_write_lock:
                    try:
                        self._exo_serial.reset_output_buffer()
                    except Exception:
                        pass
                    self._exo_serial.write(packet.encode("ascii"))
                self._exo_tx_total += 1
                self._exo_tx_count.configure(text=str(self._exo_tx_total))
                if self._exo_tx_total % 50 == 0:
                    self._exo_log_line(f"TX #{self._exo_tx_total}  {packet.strip()} mA", GREEN)
                self._exo_tx_lbl.configure(text=f"last TX: {packet.strip()} mA", fg=MUTED)
        except Exception as e:
            self._exo_err_total += 1
            self._exo_err_card.configure(text=str(self._exo_err_total))
            err_short = str(e).split("\n")[0][:60]
            self._exo_log_line(f"TX error: {err_short}", RED)
            self._exo_tx_lbl.configure(text=f"TX error: {err_short}", fg=RED)

        interval_ms = max(20, int(1000 / self._exo_rate_var.get()))
        self._exo_stream_after = self.after(interval_ms, self._exo_stream_tick)

    def _exo_plot_loop(self):
        # Always keep plot alive after first connect
        try:
            t = list(self._exo_t_hist)
            pred = list(self._exo_pred_hist)
            true = list(self._exo_true_hist)
            cmd = list(self._exo_cmd_hist)
            meas = list(self._exo_meas_hist)

            self._exo_force_pred_line.set_data(t[:len(pred)], pred)
            self._exo_force_true_line.set_data(t[:len(true)], true)
            self._exo_cmd_line.set_data(t[:len(cmd)], cmd)

            if meas:
                t_meas = t[-len(meas):] if len(t) >= len(meas) else list(range(len(meas)))
                self._exo_meas_line.set_data(t_meas, meas)

            if t:
                self._exo_ax_force.set_xlim(max(0.0, t[-1] - 20.0), max(20.0, t[-1] + 0.5))
            all_force = pred + true
            if all_force:
                self._exo_ax_force.set_ylim(0, max(120.0, max(all_force) * 1.1))
            all_current = cmd + meas
            if all_current:
                self._exo_ax_current.set_ylim(0, max(200.0, max(all_current) * 1.2))

            self._exo_canvas.draw_idle()
        except Exception:
            pass

        if getattr(self, "_exo_serial", None) is not None:
            self._exo_plot_after = self.after(120, self._exo_plot_loop)


    # ════════════════════════════════════════
    #  ROBOT INTEGRATION TAB
    # ════════════════════════════════════════
    def _build_robot_tab(self, parent):
        """Tab 07 — real-time robot control via EMG grip prediction.

        Layout:
          Connections: dual-GDX button + robot Arduino serial port
          Stream controls: rate, filtered toggle, start/stop
          Live metric cards: human grip, prediction, robot grip, TX count
          Scrolling plot: green=Scale 1 (human), cyan=prediction, orange=Scale 2 (robot)
          TX/RX log
        """
        # ── Rolling history buffers ───────────────────────────────────────
        self._robot_hist_len  = 400
        self._robot_t_hist    = collections.deque(maxlen=self._robot_hist_len)
        self._robot_sc1_hist  = collections.deque(maxlen=self._robot_hist_len)
        self._robot_sc2_hist  = collections.deque(maxlen=self._robot_hist_len)
        self._robot_pred_hist = collections.deque(maxlen=self._robot_hist_len)

        # Serial / stream state
        self._robot_serial           = None
        self._robot_streaming        = False
        self._robot_tx_total         = 0
        self._robot_err_total        = 0
        self._robot_t0               = None
        self._robot_stream_after     = None
        self._robot_plot_after       = None
        self._robot_write_lock       = threading.Lock()
        self._robot_handshake_running = False  # True while boot/calibration reader is active

        # MicroOpen protocol state
        self._robot_cmd_ma_hist   = collections.deque(maxlen=self._robot_hist_len)
        self._robot_enc_hist      = collections.deque(maxlen=self._robot_hist_len)
        self._robot_meas_ma_hist  = collections.deque(maxlen=self._robot_hist_len)
        self._robot_last_cmd_ma   = 0
        self._robot_latest_enc    = 0
        self._robot_latest_meas_ma = 0.0
        self._robot_rx_running    = False
        self._robot_rx_thread     = None
        # Calibration geometry learned from Arduino's READY line
        self._robot_span_abs      = 0    # |open_pos - close_pos| in encoder ticks
        self._robot_sign_open     = 1    # +1 if open > close, -1 if open < close

        # ── Header ───────────────────────────────────────────────────────
        hdr = tk.Frame(parent, bg=PANEL,
                       highlightbackground=BORDER, highlightthickness=1)
        hdr.pack(fill=tk.X, padx=16, pady=(6, 2))
        tk.Label(hdr,
                 text="ROBOT INTEGRATION  —  EMG Prediction → Serial Bridge → Robot Pincher",
                 font=("Arial", 10, "bold"), fg=MUTED, bg=PANEL
                 ).pack(anchor=tk.W, padx=10, pady=(5, 5))

        # ── Connections ──────────────────────────────────────────────────
        conn_f = tk.Frame(parent, bg=PANEL,
                          highlightbackground=BORDER, highlightthickness=1)
        conn_f.pack(fill=tk.X, padx=16, pady=(2, 2))

        conn_row = tk.Frame(conn_f, bg=PANEL)
        conn_row.pack(fill=tk.X, padx=10, pady=6)

        # GDX status — read-only indicator. Both scales are connected from
        # the main "CONNECT GDX" button (sidebar / exo tab); there's no
        # separate connect action here, so this just reflects live state.
        self._robot_gdx_btn = tk.Label(
            conn_row, text="⬡  GDX: NOT CONNECTED",
            font=("Arial", 10, "bold"), bg=PANEL2, fg=MUTED,
            relief=tk.FLAT, padx=12, pady=6)
        self._robot_gdx_btn.pack(side=tk.LEFT, padx=(0, 16))

        # Robot serial port picker
        self._robot_port_var  = tk.StringVar(value="-- scan first --")
        self._robot_baud_var  = tk.StringVar(value="115200")

        self._robot_port_menu = tk.OptionMenu(conn_row, self._robot_port_var, "-- scan first --")
        self._robot_port_menu.configure(bg=PANEL2, fg=TEXT, relief=tk.FLAT,
                                        font=FONT_MONO_SM, width=22,
                                        highlightbackground=BORDER,
                                        activebackground=PANEL2)
        self._robot_port_menu["menu"].configure(bg=PANEL2, fg=TEXT, font=FONT_MONO_SM)
        self._robot_port_menu.pack(side=tk.LEFT, padx=(0, 4))

        tk.Button(conn_row, text="⟳", font=("Arial", 9, "bold"),
                  bg=PANEL2, fg=YELLOW, relief=tk.FLAT, padx=8, pady=5,
                  command=self._robot_scan_ports).pack(side=tk.LEFT, padx=(0, 10))

        tk.Label(conn_row, text="BAUD", font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT, padx=(0, 4))
        robot_baud_menu = tk.OptionMenu(
            conn_row, self._robot_baud_var, "9600", "57600", "115200", "1000000")
        robot_baud_menu.configure(bg=PANEL2, fg=TEXT, relief=tk.FLAT,
                                  font=FONT_MONO_SM, width=8,
                                  highlightbackground=BORDER,
                                  activebackground=PANEL2)
        robot_baud_menu["menu"].configure(bg=PANEL2, fg=TEXT, font=FONT_MONO_SM)
        robot_baud_menu.pack(side=tk.LEFT, padx=(0, 10))

        self._robot_serial_btn = tk.Button(
            conn_row, text="⬡  CONNECT ROBOT",
            font=("Arial", 10, "bold"), bg=PANEL2, fg=ACCENT,
            relief=tk.FLAT, padx=12, pady=6,
            command=self._robot_toggle_serial)
        self._robot_serial_btn.pack(side=tk.LEFT, padx=(0, 16))

        self._robot_status_lbl = tk.Label(
            conn_row, text="●  DISCONNECTED",
            font=("Arial", 10, "bold"), fg=RED, bg=PANEL)
        self._robot_status_lbl.pack(side=tk.LEFT)
        self._robot_gdx_status_lbl = tk.Label(
            conn_row, text="  |  GDX: —",
            font=FONT_MONO_SM, fg=MUTED, bg=PANEL)
        self._robot_gdx_status_lbl.pack(side=tk.LEFT, padx=(4, 0))

        # ── Motor calibration handshake ───────────────────────────────────
        # The Arduino firmware (GripReleaseController) blocks in setup() waiting
        # for the user to manually position the gripper at CLOSE and OPEN, pressing
        # ENTER after each. These buttons send \n to the Arduino at the right time.
        # The handshake reader enables each button when the matching prompt arrives.
        cal_f = tk.Frame(parent, bg=PANEL,
                         highlightbackground=BORDER, highlightthickness=1)
        cal_f.pack(fill=tk.X, padx=16, pady=(0, 2))
        cal_row = tk.Frame(cal_f, bg=PANEL)
        cal_row.pack(fill=tk.X, padx=10, pady=5)
        tk.Label(cal_row, text="MOTOR CAL:",
                 font=("Arial", 9, "bold"), fg=MUTED, bg=PANEL
                 ).pack(side=tk.LEFT, padx=(0, 8))
        self._robot_cal_close_btn = tk.Button(
            cal_row, text="📍  CONFIRM CLOSE POS",
            font=("Arial", 10, "bold"), bg=PANEL2, fg=MUTED,
            relief=tk.FLAT, padx=12, pady=6, state=tk.DISABLED,
            command=lambda: self._robot_send_enter("close"))
        self._robot_cal_close_btn.pack(side=tk.LEFT, padx=(0, 8))
        self._robot_cal_open_btn = tk.Button(
            cal_row, text="📍  CONFIRM OPEN POS",
            font=("Arial", 10, "bold"), bg=PANEL2, fg=MUTED,
            relief=tk.FLAT, padx=12, pady=6, state=tk.DISABLED,
            command=lambda: self._robot_send_enter("open"))
        self._robot_cal_open_btn.pack(side=tk.LEFT, padx=(0, 16))
        self._robot_cal_lbl = tk.Label(
            cal_row, text="—  connect robot serial to begin",
            font=FONT_MONO_SM, fg=MUTED, bg=PANEL)
        self._robot_cal_lbl.pack(side=tk.LEFT)

        # ── Stream controls ───────────────────────────────────────────────
        ctrl_f = tk.Frame(parent, bg=PANEL,
                          highlightbackground=BORDER, highlightthickness=1)
        ctrl_f.pack(fill=tk.X, padx=16, pady=(0, 2))
        ctrl_row = tk.Frame(ctrl_f, bg=PANEL)
        ctrl_row.pack(fill=tk.X, padx=10, pady=5)

        tk.Label(ctrl_row, text="Rate (Hz)", font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT)
        self._robot_rate_var = tk.IntVar(value=10)
        tk.Scale(ctrl_row, variable=self._robot_rate_var,
                 from_=1, to=50, resolution=1,
                 orient=tk.HORIZONTAL, length=110,
                 bg=PANEL, fg=TEXT, troughcolor=PANEL2,
                 highlightthickness=0, bd=0, showvalue=0
                 ).pack(side=tk.LEFT, padx=(6, 4))
        self._robot_rate_lbl = tk.Label(ctrl_row, text="10 Hz",
                                        font=FONT_MONO_SM, fg=ACCENT, bg=PANEL, width=6)
        self._robot_rate_lbl.pack(side=tk.LEFT, padx=(0, 16))
        self._robot_rate_var.trace_add("write", lambda *_: self._robot_rate_lbl.configure(
            text=f"{self._robot_rate_var.get()} Hz"))

        self._robot_use_filtered_var = tk.BooleanVar(value=True)
        tk.Checkbutton(ctrl_row, text="Use filtered prediction",
                       variable=self._robot_use_filtered_var,
                       font=FONT_MONO_SM, bg=PANEL, fg=MUTED,
                       selectcolor=PANEL2, activebackground=PANEL,
                       activeforeground=TEXT).pack(side=tk.LEFT, padx=(0, 16))

        self._robot_stream_btn = tk.Button(
            ctrl_row, text="▶  START STREAM",
            font=("Arial", 10, "bold"), bg=PANEL2, fg=ACCENT,
            relief=tk.FLAT, padx=14, pady=7, state=tk.DISABLED,
            command=self._robot_toggle_stream)
        self._robot_stream_btn.pack(side=tk.LEFT)

        # ── MicroOpen release parameters ──────────────────────────────────
        mo_f = tk.Frame(parent, bg=PANEL,
                        highlightbackground=BORDER, highlightthickness=1)
        mo_f.pack(fill=tk.X, padx=16, pady=(0, 2))
        mo_row = tk.Frame(mo_f, bg=PANEL)
        mo_row.pack(fill=tk.X, padx=10, pady=5)
        tk.Label(mo_row, text="PARAMS:", font=("Arial", 9, "bold"),
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT, padx=(0, 8))

        tk.Label(mo_row, text="MVC force (N)", font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT)
        self._robot_mvc_force_var = tk.StringVar(value="30.0")
        tk.Entry(mo_row, textvariable=self._robot_mvc_force_var, width=7,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER,
                 highlightthickness=1).pack(side=tk.LEFT, padx=(4, 14))

        tk.Label(mo_row, text="Max current (mA)", font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT)
        self._robot_max_ma_var = tk.StringVar(value="1000")
        tk.Entry(mo_row, textvariable=self._robot_max_ma_var, width=6,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER,
                 highlightthickness=1).pack(side=tk.LEFT, padx=(4, 14))

        # Span — auto-filled from Arduino READY message; can also be typed manually.
        # This is the |open_pos - close_pos| in encoder ticks from calibration.
        # 100% MVC → 0 ticks offset (fully closed). 0% MVC → span ticks (fully open).
        tk.Label(mo_row, text="Span (ticks)", font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT)
        self._robot_span_var = tk.StringVar(value="0")
        tk.Entry(mo_row, textvariable=self._robot_span_var, width=6,
                 bg=PANEL2, fg=PURPLE, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER,
                 highlightthickness=1).pack(side=tk.LEFT, padx=(4, 4))
        tk.Label(mo_row, text="(auto from calibration — or type manually)",
                 font=FONT_MONO_SM, fg=MUTED, bg=PANEL).pack(side=tk.LEFT, padx=(4, 0))
        # Keep _robot_pulse_var so nothing else breaks, but it is no longer used.
        self._robot_pulse_var = tk.IntVar(value=0)

        # ── Metric cards ─────────────────────────────────────────────────
        met_f = tk.Frame(parent, bg=PANEL,
                         highlightbackground=BORDER, highlightthickness=1)
        met_f.pack(fill=tk.X, padx=16, pady=(0, 2))
        cards_row = tk.Frame(met_f, bg=PANEL)
        cards_row.pack(fill=tk.X, padx=10, pady=4)
        self._robot_sc1_card   = self._metric_card(cards_row, "Human (% MVC)", "—", GREEN)
        self._robot_pred_card  = self._metric_card(cards_row, "Prediction (% MVC)", "—", ACCENT)
        self._robot_sc2_card   = self._metric_card(cards_row, "Robot (% RFO)", "—", ACCENT2)
        self._robot_cmd_card   = self._metric_card(cards_row, "Cmd mA", "—", YELLOW)
        self._robot_enc_card   = self._metric_card(cards_row, "Encoder", "—", PURPLE)
        self._robot_tx_card    = self._metric_card(cards_row, "Packets TX", "0", MUTED)
        self._robot_err_card   = self._metric_card(cards_row, "TX Errors", "0", RED)

        # RFO calibration row
        rfo_row = tk.Frame(met_f, bg=PANEL)
        rfo_row.pack(fill=tk.X, padx=10, pady=(0, 6))
        self._robot_rfo_cal_btn = tk.Button(
            rfo_row, text="⊙  CALIBRATE RFO  (ramp motor to set mA)",
            font=("Arial", 9, "bold"), bg=PANEL2, fg=ACCENT2,
            relief=tk.FLAT, padx=10, pady=4,
            command=self._robot_rfo_cal_start)
        self._robot_rfo_cal_btn.pack(side=tk.LEFT, padx=(0, 8))
        tk.Button(rfo_row, text="↺  RESET RFO",
                  font=("Arial", 9, "bold"), bg=PANEL2, fg=YELLOW,
                  relief=tk.FLAT, padx=8, pady=4,
                  command=self._robot_rfo_reset).pack(side=tk.LEFT, padx=(0, 12))
        self._robot_rfo_lbl = tk.Label(
            rfo_row, text=f"RFO = {_robot_rfo:.2f} N  (not calibrated)",
            font=FONT_MONO_SM, fg=MUTED, bg=PANEL)
        self._robot_rfo_lbl.pack(side=tk.LEFT)

        # ── Robot Response Metrics ────────────────────────────────────────
        rmet_f = tk.Frame(parent, bg=PANEL,
                          highlightbackground=BORDER, highlightthickness=1)
        rmet_f.pack(fill=tk.X, padx=16, pady=(0, 2))
        tk.Label(rmet_f, text="ROBOT RESPONSE METRICS",
                 font=("Arial", 9, "bold"), fg=MUTED, bg=PANEL
                 ).pack(anchor=tk.W, padx=10, pady=(4, 2))

        # Recording controls
        rec_row = tk.Frame(rmet_f, bg=PANEL)
        rec_row.pack(fill=tk.X, padx=10, pady=(0, 0))
        tk.Label(rec_row, text="USER", font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT, padx=(0, 2))
        self._robot_perf_user_var = tk.StringVar(value="SUB_001")
        tk.Entry(rec_row, textvariable=self._robot_perf_user_var, width=10,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER,
                 highlightthickness=1).pack(side=tk.LEFT, padx=(0, 6))
        tk.Label(rec_row, text="SESSION", font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT, padx=(0, 2))
        self._robot_perf_session_var = tk.StringVar(value="")
        tk.Entry(rec_row, textvariable=self._robot_perf_session_var, width=10,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER,
                 highlightthickness=1).pack(side=tk.LEFT, padx=(0, 6))
        tk.Label(rec_row, text="DURATION (s)", font=FONT_MONO_SM,
                 fg=MUTED, bg=PANEL).pack(side=tk.LEFT, padx=(0, 2))
        self._robot_perf_dur_var = tk.StringVar(value="45")
        tk.Entry(rec_row, textvariable=self._robot_perf_dur_var, width=4,
                 bg=PANEL2, fg=TEXT, relief=tk.FLAT, font=FONT_MONO_SM,
                 insertbackground=TEXT, highlightbackground=BORDER,
                 highlightthickness=1).pack(side=tk.LEFT, padx=(0, 8))
        self._robot_perf_btn = tk.Button(
            rec_row, text="▶  RECORD SESSION",
            font=("Arial", 10, "bold"), bg=PANEL2, fg=ACCENT,
            relief=tk.FLAT, padx=10, pady=4,
            command=self._robot_toggle_recording)
        self._robot_perf_btn.pack(side=tk.LEFT, padx=(0, 4))
        tk.Button(rec_row, text="⟳  CLEAR TABLE", font=("Arial", 10, "bold"),
                  bg=PANEL2, fg=YELLOW, relief=tk.FLAT, padx=10, pady=4,
                  command=self._robot_clear_sessions).pack(side=tk.LEFT, padx=(0, 4))

        # Export row — same trio as the Performance tab: summary stats,
        # full raw streaming data, and a PNG snapshot per session.
        export_row = tk.Frame(rmet_f, bg=PANEL)
        export_row.pack(fill=tk.X, padx=10, pady=(0, 2))
        tk.Button(export_row, text="⇩  EXPORT SUMMARY (MEAN ± STD)", font=("Arial", 10, "bold"),
                  bg=PANEL2, fg=GREEN, relief=tk.FLAT, padx=10, pady=4,
                  command=self._robot_export_csv).pack(side=tk.LEFT, padx=(0, 4))
        tk.Button(export_row, text="⇩  EXPORT ALL DATA", font=("Arial", 10, "bold"),
                  bg=PANEL2, fg=ACCENT, relief=tk.FLAT, padx=10, pady=4,
                  command=self._robot_export_all_data_csv).pack(side=tk.LEFT, padx=(0, 4))
        tk.Button(export_row, text="⇩  EXPORT PNG (ALL SESSIONS)", font=("Arial", 10, "bold"),
                  bg=PANEL2, fg=PURPLE, relief=tk.FLAT, padx=10, pady=4,
                  command=self._robot_export_png).pack(side=tk.LEFT, padx=(0, 4))

        # Progress bar
        rec_prog_row = tk.Frame(rmet_f, bg=PANEL)
        rec_prog_row.pack(fill=tk.X, padx=10, pady=(2, 4))
        self._robot_perf_prog_lbl = tk.Label(
            rec_prog_row, text="READY", font=FONT_MONO_SM,
            fg=MUTED, bg=PANEL, anchor=tk.W)
        self._robot_perf_prog_lbl.pack(side=tk.LEFT, padx=(0, 8))
        self._robot_perf_prog_track = tk.Canvas(
            rec_prog_row, height=8, bg=BORDER, highlightthickness=0)
        self._robot_perf_prog_track.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 8))
        self._robot_perf_prog_bar = self._robot_perf_prog_track.create_rectangle(
            0, 0, 0, 8, fill=ACCENT2, outline="")

        # Prediction → Robot Grip — session averages
        pr_avg_f = tk.Frame(parent, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        pr_avg_f.pack(fill=tk.X, padx=16, pady=(0, 2))
        tk.Label(pr_avg_f, text="PREDICTION → ROBOT GRIP — SESSION AVERAGES",
                 font=("Arial", 9, "bold"), fg=MUTED, bg=PANEL).pack(anchor=tk.W, padx=8, pady=(4, 2))
        pr_avg_row = tk.Frame(pr_avg_f, bg=PANEL)
        pr_avg_row.pack(fill=tk.X, padx=8, pady=(0, 4))
        self._robot_pr_avg_r2   = self._metric_card(pr_avg_row, "R²",        "—", ACCENT)
        self._robot_pr_avg_rmse = self._metric_card(pr_avg_row, "%RMSE",     "—", YELLOW)
        self._robot_pr_avg_r    = self._metric_card(pr_avg_row, "Pearson r", "—", GREEN)
        self._robot_pr_avg_lag  = self._metric_card(pr_avg_row, "Lag (ms)",  "—", PURPLE)

        # Human Grip → Robot Grip — session averages
        hr_avg_f = tk.Frame(parent, bg=PANEL, highlightbackground=BORDER, highlightthickness=1)
        hr_avg_f.pack(fill=tk.X, padx=16, pady=(0, 2))
        tk.Label(hr_avg_f, text="HUMAN GRIP → ROBOT GRIP — SESSION AVERAGES",
                 font=("Arial", 9, "bold"), fg=MUTED, bg=PANEL).pack(anchor=tk.W, padx=8, pady=(4, 2))
        hr_avg_row = tk.Frame(hr_avg_f, bg=PANEL)
        hr_avg_row.pack(fill=tk.X, padx=8, pady=(0, 4))
        self._robot_hr_avg_r2   = self._metric_card(hr_avg_row, "R²",        "—", ACCENT)
        self._robot_hr_avg_rmse = self._metric_card(hr_avg_row, "%RMSE",     "—", YELLOW)
        self._robot_hr_avg_r    = self._metric_card(hr_avg_row, "Pearson r", "—", GREEN)
        self._robot_hr_avg_lag  = self._metric_card(hr_avg_row, "Lag (ms)",  "—", PURPLE)

        # 3-trace overlay chart (recording-based, fixed x-axis)
        robot_overlay_f = tk.Frame(parent, bg=PANEL)
        robot_overlay_f.pack(fill=tk.X, padx=16, pady=(0, 2))
        self._build_robot_overlay_canvas(robot_overlay_f)

        # TX/RX log
        log_f = tk.Frame(parent, bg=BG)
        log_f.pack(fill=tk.X, padx=16, pady=(4, 8))
        tk.Label(log_f, text="TX / RX LOG",
                 font=("Arial", 9, "bold"), fg=MUTED, bg=BG
                 ).pack(anchor=tk.W)
        self._robot_log_txt = tk.Text(
            log_f, height=5, bg=PANEL2, fg=MUTED,
            font=("Courier New", 8), state=tk.DISABLED,
            relief=tk.FLAT, wrap=tk.WORD,
            highlightbackground=BORDER, highlightthickness=1)
        self._robot_log_txt.pack(fill=tk.X)

        # Prediction → Robot Grip session results table
        pr_table_wrap = tk.Frame(parent, bg=BG)
        pr_table_wrap.pack(fill=tk.BOTH, expand=True, padx=16, pady=(4, 4))
        tk.Label(pr_table_wrap, text="PREDICTION → ROBOT GRIP SESSION RESULTS",
                 font=("Arial", 9, "bold"), fg=MUTED, bg=BG).pack(anchor=tk.W)
        pr_cols = ("user", "session", "n", "r2", "rmse", "pearson", "lag")
        self._robot_pr_table = ttk.Treeview(
            pr_table_wrap, columns=pr_cols, show="headings", height=5)
        for col, title, width in [
            ("user", "User", 110), ("session", "Session", 130),
            ("n", "Samples", 70), ("r2", "R²", 70),
            ("rmse", "%RMSE", 80), ("pearson", "Pearson r", 90), ("lag", "Lag (ms)", 80)
        ]:
            self._robot_pr_table.heading(col, text=title)
            self._robot_pr_table.column(col, width=width, anchor=tk.CENTER)
        self._robot_pr_table.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        pr_sb = ttk.Scrollbar(pr_table_wrap, orient=tk.VERTICAL, command=self._robot_pr_table.yview)
        self._robot_pr_table.configure(yscrollcommand=pr_sb.set)
        pr_sb.pack(side=tk.RIGHT, fill=tk.Y)

        # Human Grip → Robot Grip session results table
        hr_table_wrap = tk.Frame(parent, bg=BG)
        hr_table_wrap.pack(fill=tk.BOTH, expand=True, padx=16, pady=(0, 8))
        tk.Label(hr_table_wrap, text="HUMAN GRIP → ROBOT GRIP SESSION RESULTS",
                 font=("Arial", 9, "bold"), fg=MUTED, bg=BG).pack(anchor=tk.W)
        hr_cols = ("user", "session", "n", "r2", "rmse", "pearson", "lag")
        self._robot_hr_table = ttk.Treeview(
            hr_table_wrap, columns=hr_cols, show="headings", height=5)
        for col, title, width in [
            ("user", "User", 110), ("session", "Session", 130),
            ("n", "Samples", 70), ("r2", "R²", 70),
            ("rmse", "%RMSE", 80), ("pearson", "Pearson r", 90), ("lag", "Lag (ms)", 80)
        ]:
            self._robot_hr_table.heading(col, text=title)
            self._robot_hr_table.column(col, width=width, anchor=tk.CENTER)
        self._robot_hr_table.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        hr_sb = ttk.Scrollbar(hr_table_wrap, orient=tk.VERTICAL, command=self._robot_hr_table.yview)
        self._robot_hr_table.configure(yscrollcommand=hr_sb.set)
        hr_sb.pack(side=tk.RIGHT, fill=tk.Y)

    # ── Robot tab helpers ─────────────────────────────────────────────────

    def _robot_log(self, msg, color=None):
        ts   = time.strftime("%H:%M:%S")
        line = f"[{ts}]  {msg}\n"
        try:
            self._robot_log_txt.configure(state=tk.NORMAL)
            tag = ts + str(id(msg))
            self._robot_log_txt.insert(tk.END, line, tag)
            if color:
                self._robot_log_txt.tag_configure(tag, foreground=color)
            n = int(self._robot_log_txt.index("end-1c").split(".")[0])
            if n > 200:
                self._robot_log_txt.delete("1.0", f"{n-200}.0")
            self._robot_log_txt.see(tk.END)
            self._robot_log_txt.configure(state=tk.DISABLED)
        except Exception:
            pass

    # ── RFO calibration ──────────────────────────────────────────────────

    def _robot_rfo_cal_start(self):
        """Ramp motor current 0 -> max_ma directly over ROBOT_RFO_CAL_DURATION
        (independent of the EMG stream / prediction), tracking Scale 2's peak
        force during the ramp and setting _robot_rfo to that peak.
        """
        if not STATE.connected_gdx:
            self._robot_log("GDX not connected — connect GDX before calibrating RFO.", RED)
            return
        if not (self._robot_serial and self._robot_serial.is_open):
            self._robot_log("Robot serial not connected — connect robot before calibrating RFO.", RED)
            return
        try:
            self._robot_rfo_cal_target_ma = max(1, int(float(self._robot_max_ma_var.get())))
        except Exception:
            self._robot_rfo_cal_target_ma = 1000
        self._robot_rfo_cal_peak    = 0.0
        self._robot_rfo_cal_start_t = time.time()
        self._robot_rfo_cal_end     = self._robot_rfo_cal_start_t + ROBOT_RFO_CAL_DURATION
        self._robot_rfo_cal_btn.configure(
            text=f"● RAMPING MOTOR…  0/{self._robot_rfo_cal_target_ma} mA", fg=RED, state=tk.DISABLED)
        self._robot_log(
            f"RFO calibration started — ramping motor to {self._robot_rfo_cal_target_ma} mA "
            f"over {ROBOT_RFO_CAL_DURATION:.0f}s.", ACCENT2)
        self._robot_rfo_cal_tick()

    def _robot_rfo_cal_tick(self):
        now       = time.time()
        remaining = self._robot_rfo_cal_end - now
        frac      = min(1.0, (now - self._robot_rfo_cal_start_t) / ROBOT_RFO_CAL_DURATION)
        cmd_ma    = int(round(frac * self._robot_rfo_cal_target_ma))

        try:
            if self._robot_serial and self._robot_serial.is_open:
                with self._robot_write_lock:
                    self._robot_serial.write(f"C,{cmd_ma},0\n".encode("ascii"))
        except Exception as e:
            self._robot_log(f"RFO cal TX error: {str(e).split(chr(10))[0][:60]}", RED)

        sc2 = _robot_gdx_scale2[0]
        if sc2 > self._robot_rfo_cal_peak:
            self._robot_rfo_cal_peak = sc2

        if remaining > 0:
            self._robot_rfo_cal_btn.configure(
                text=f"● RAMPING MOTOR…  {cmd_ma}/{self._robot_rfo_cal_target_ma} mA  {remaining:.1f}s")
            self.after(50, self._robot_rfo_cal_tick)
        else:
            # Ramp complete — release the grip now that we've recorded the peak.
            try:
                if self._robot_serial and self._robot_serial.is_open:
                    with self._robot_write_lock:
                        self._robot_serial.write(b"C,0,0\n")
            except Exception:
                pass
            global _robot_rfo
            peak = max(self._robot_rfo_cal_peak, 0.1)
            _robot_rfo = peak
            self._robot_rfo_lbl.configure(
                text=f"RFO = {peak:.2f} N  ✔ calibrated",
                fg=GREEN)
            self._robot_rfo_cal_btn.configure(
                text="⊙  CALIBRATE RFO  (ramp motor to set mA)",
                fg=ACCENT2, state=tk.NORMAL)
            self._robot_log(
                f"RFO calibrated: {peak:.2f} N at {self._robot_rfo_cal_target_ma} mA — "
                f"robot % RFO will now be stable from ramp 1.", GREEN)

    def _robot_rfo_reset(self):
        global _robot_rfo
        _robot_rfo = 0.1
        self._robot_rfo_lbl.configure(
            text=f"RFO = {_robot_rfo:.2f} N  (not calibrated)", fg=MUTED)
        self._robot_log("RFO reset to 0.1 N — will grow from next squeeze.", YELLOW)

    def _build_robot_overlay_canvas(self, parent):
        """Robot response chart — red guide + 3 data traces."""
        popout_bar = self._add_popout_button(parent, "robot")

        fig = Figure(figsize=(8.2, 3.2), facecolor=PANEL)
        fig.subplots_adjust(top=0.82, bottom=0.28, left=0.09, right=0.94)
        ax  = fig.add_subplot(111)
        self._robot_fig = fig
        self._robot_ax  = ax

        ax.set_facecolor(PANEL2)
        for sp in ax.spines.values():
            sp.set_color(BORDER); sp.set_linewidth(0.6)
        ax.tick_params(colors=MUTED, labelsize=11)
        ax.set_xlabel("Time (s)", color=MUTED, fontsize=11, labelpad=4)
        ax.set_ylabel("% Force", color=MUTED, fontsize=11, labelpad=6)
        ax.set_xlim(0, 1); ax.set_ylim(0, 135)

        # Red effort guide (same source as performance tab)
        self._robot_guide_line, = ax.plot(
            [], [], color=RED, lw=3.5, zorder=3,
            label="Target", solid_capstyle="round", solid_joinstyle="round")
        self._robot_h_guides  = []
        self._robot_seg_texts = []

        # zorder explicit and > the guide's 3 on all of these — without it
        # they fall back to Line2D's default zorder of 2 and render behind
        # the red guide instead of in front of it.
        self._robot_sc1_line, = ax.plot(
            [], [], color=GREEN, lw=2.2, zorder=4, label="Human Grip (% MVC)",
            solid_capstyle="round", alpha=0.9)
        self._robot_pred_line, = ax.plot(
            [], [], color=ACCENT, lw=2.2, zorder=5, label="Prediction (% MVC)",
            linestyle="--", solid_capstyle="round", alpha=0.9)
        self._robot_sc2_line, = ax.plot(
            [], [], color=ACCENT2, lw=2.2, zorder=6, label="Robot Grip (% RFO)",
            solid_capstyle="round", alpha=0.9)
        # Tracks the human grip line, so it stays visible whenever that does.
        self._robot_live_dot, = ax.plot(
            [], [], 'o', color=GREEN, ms=8, zorder=8, alpha=1.0)

        # Prediction and robot output are hidden by default so the
        # participant tracks the red guide by feel; human grip stays
        # visible always since it's just their own live effort. Toggling
        # afterward reveals the already-recorded curves for review.
        self._make_visibility_toggle(
            popout_bar, "PREDICTION", ACCENT, [self._robot_pred_line],
            lambda: self._robot_canvas.draw_idle(), "robot")
        self._make_visibility_toggle(
            popout_bar, "ROBOT GRIP", ACCENT2, [self._robot_sc2_line],
            lambda: self._robot_canvas.draw_idle(), "robot")
        self._robot_playhead = ax.axvline(
            x=-1, color=ACCENT, lw=2.0, linestyle="--", zorder=7, alpha=0.8)
        self._robot_now_span_lines = []

        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.28),
                  ncol=4, fontsize=10, labelcolor=TEXT,
                  facecolor=PANEL2, edgecolor=BORDER, framealpha=0.85,
                  borderpad=0.8, handlelength=2.0, columnspacing=1.5)
        ax.set_title("ROBOT RESPONSE  —  press RECORD SESSION to begin",
                     color=MUTED, fontsize=10, loc="left", pad=10)

        self._robot_canvas = FigureCanvasTkAgg(fig, master=parent)
        self._robot_canvas.get_tk_widget().pack(fill=tk.X)

    def _robot_overlay_draw_guide(self, dur):
        """Draw red effort guide + h-lines on the robot chart (mirrors perf tab)."""
        ax = self._robot_ax
        for ln in self._robot_h_guides:
            try: ln.remove()
            except Exception: pass
        self._robot_h_guides.clear()
        for tx in self._robot_seg_texts:
            try: tx.remove()
            except Exception: pass
        self._robot_seg_texts.clear()

        if not hasattr(self, "_overlay_t_path"):
            return
        gt = np.array(self._overlay_t_path)
        gf = np.array(self._overlay_y_path)
        if not len(gt):
            return
        self._robot_guide_line.set_data(gt, gf)

        for pct in [25, 50, 75, 100]:
            ln = ax.axhline(pct, color=MUTED, lw=0.5, linestyle=":", zorder=1, alpha=0.5)
            self._robot_h_guides.append(ln)

        tx = ax.text(0.01, 102, "100% MVC", color=MUTED, fontsize=8,
                     transform=ax.get_yaxis_transform(), va="bottom", ha="left")
        self._robot_seg_texts.append(tx)

        reps      = getattr(self, "_overlay_reps",      3)
        rest_s    = getattr(self, "_overlay_rest_s",    REST_SEC)
        ramp_half = getattr(self, "_overlay_ramp_half", RAMP_SEC / 2.0)

        t_c = 0.0
        for i in range(reps):
            t_c += rest_s
            ramp_peak = t_c + ramp_half
            tx = ax.text(ramp_peak, 104, f"Ramp {i+1}",
                         color=MUTED, fontsize=8, ha="center", va="bottom",
                         fontweight="bold", clip_on=False)
            self._robot_seg_texts.append(tx)
            t_c += ramp_half + RAMP_PEAK_HOLD_SEC + ramp_half + rest_s

        ax.set_xlim(0, dur)
        ax.set_ylim(0, 135)
        ax.set_title("ROBOT RESPONSE — follow the red guide  |  recording…",
                     color=YELLOW, fontsize=10, loc="left", pad=10, fontweight="bold")

    # ── Robot response session management ────────────────────────────────

    def _robot_toggle_recording(self):
        if STATE.robot_recording:
            self._robot_finish_session(stopped_early=True)
        else:
            self._robot_start_recording()

    def _robot_start_recording(self):
        if not self._robot_streaming:
            self._robot_log("Start STREAM first before recording a session.", YELLOW)
            return
        try:
            dur = float(self._robot_perf_dur_var.get())
        except ValueError:
            dur = 20.0
        dur = max(1.0, dur)
        STATE.robot_duration     = dur
        STATE.robot_session_pred = []
        STATE.robot_session_sc1  = []
        STATE.robot_session_sc2  = []
        STATE.robot_session_ts   = []
        STATE.robot_recording    = True
        self._robot_perf_t0 = time.time()
        user_id    = (self._robot_perf_user_var.get()    or "").strip() or "SUB_001"
        session_id = (self._robot_perf_session_var.get() or "").strip() or time.strftime("S_%H%M%S")
        self._robot_perf_user_var.set(user_id)
        self._robot_perf_session_var.set(session_id)
        self._robot_perf_btn.configure(text="■  STOP SESSION", fg=RED)
        self._robot_perf_prog_lbl.configure(
            text=f"RECORDING  {user_id} / {session_id}", fg=RED)
        self._robot_perf_prog_track.coords(self._robot_perf_prog_bar, 0, 0, 0, 8)

        # Reset data traces and draw guide
        self._robot_sc1_line.set_data([], [])
        self._robot_pred_line.set_data([], [])
        self._robot_sc2_line.set_data([], [])
        self._robot_live_dot.set_data([], [])
        self._robot_playhead.set_xdata([-1])
        for ln in self._robot_now_span_lines:
            try: ln.remove()
            except Exception: pass
        self._robot_now_span_lines.clear()
        self._robot_overlay_draw_guide(dur)  # sets xlim, ylim, title
        self._robot_canvas.draw_idle()
        self._popout_sync("robot")

        self._robot_log(f"Robot session started: {user_id}/{session_id}  {dur:.0f}s", ACCENT)
        self._robot_poll_id = self.after(100, self._robot_poll)

    def _robot_poll(self):
        if not STATE.robot_recording:
            return
        elapsed = time.time() - self._robot_perf_t0
        dur     = STATE.robot_duration
        pct     = min(elapsed / dur, 1.0)
        w = self._robot_perf_prog_track.winfo_width()
        self._robot_perf_prog_track.coords(self._robot_perf_prog_bar, 0, 0, int(pct * w), 8)
        self._robot_perf_prog_lbl.configure(text=f"RECORDING  {elapsed:.1f}s / {dur:.0f}s")

        ts   = list(STATE.robot_session_ts)
        sc1  = list(STATE.robot_session_sc1)
        pred = list(STATE.robot_session_pred)
        sc2  = list(STATE.robot_session_sc2)
        n    = min(len(ts), len(sc1), len(pred), len(sc2))
        if n > 0:
            t0    = ts[0]
            t_rel = [ts[i] - t0 for i in range(n)]
            self._robot_sc1_line.set_data(t_rel, sc1[:n])
            self._robot_pred_line.set_data(t_rel, pred[:n])
            self._robot_sc2_line.set_data(t_rel, sc2[:n])
            self._robot_live_dot.set_data([t_rel[-1]], [sc1[n - 1]])
            data_max = max(max(sc1[:n]), max(pred[:n]), max(sc2[:n]))
            if data_max * 1.1 > self._robot_ax.get_ylim()[1]:
                self._robot_ax.set_ylim(0, max(110, data_max * 1.15))

        self._robot_playhead.set_xdata([elapsed])
        for ln in self._robot_now_span_lines:
            try: ln.remove()
            except Exception: pass
        self._robot_now_span_lines.clear()
        try:
            band_w = dur * 0.008
            span = self._robot_ax.axvspan(elapsed - band_w, elapsed + band_w,
                                          color=ACCENT, alpha=0.08, zorder=2)
            self._robot_now_span_lines.append(span)
        except Exception:
            pass
        self._robot_canvas.draw_idle()
        self._popout_sync("robot")

        if elapsed >= dur:
            self._robot_finish_session(stopped_early=False)
            return
        self._robot_poll_id = self.after(100, self._robot_poll)

    def _robot_rampup_mask(self, ts_arr):
        """Return boolean mask of samples that fall within ramp-up segments of the guide path."""
        if not (hasattr(self, "_overlay_t_path") and len(self._overlay_t_path) >= 2):
            return np.ones(len(ts_arr), dtype=bool)
        gt = np.array(self._overlay_t_path)
        gy = np.array(self._overlay_y_path)
        ts_rel = ts_arr - ts_arr[0]
        mask = np.zeros(len(ts_rel), dtype=bool)
        for i in range(len(gt) - 1):
            if gy[i + 1] > gy[i] and gt[i + 1] > gt[i]:
                mask |= (ts_rel >= gt[i]) & (ts_rel <= gt[i + 1])
        return mask

    def _robot_finish_session(self, stopped_early=False):
        if hasattr(self, "_robot_poll_id"):
            try:
                self.after_cancel(self._robot_poll_id)
            except Exception:
                pass
        STATE.robot_recording = False
        self._robot_perf_btn.configure(text="▶  RECORD SESSION", fg=ACCENT)
        self._robot_guide_line.set_data([], [])
        self._robot_live_dot.set_data([], [])
        self._robot_playhead.set_xdata([-1])
        for ln in self._robot_now_span_lines:
            try: ln.remove()
            except Exception: pass
        self._robot_now_span_lines.clear()
        for ln in self._robot_h_guides:
            try: ln.remove()
            except Exception: pass
        self._robot_h_guides.clear()
        for tx in self._robot_seg_texts:
            try: tx.remove()
            except Exception: pass
        self._robot_seg_texts.clear()

        pred_arr = np.array(STATE.robot_session_pred, dtype=float)
        sc1_arr  = np.array(STATE.robot_session_sc1,  dtype=float)
        sc2_arr  = np.array(STATE.robot_session_sc2,  dtype=float)
        ts_arr   = np.array(STATE.robot_session_ts,   dtype=float)
        # Raw samples are timestamped relative to when the serial STREAM
        # started (self._robot_t0), not when this recording SESSION started
        # (self._robot_perf_t0) — those clocks only match if you hit record
        # the instant streaming begins. Rebase to session-relative time here,
        # once, so the guide overlay (which always starts at t=0), the PNG
        # export, and the CSV export all agree — otherwise a session started
        # a while after streaming began shows a big empty gap before the
        # data on any plot that isn't the live on-screen view (which already
        # worked around this by subtracting ts[0] itself, see _robot_poll).
        if len(ts_arr):
            ts_arr = ts_arr - ts_arr[0]

        if len(pred_arr) < 5 or not (len(pred_arr) == len(sc2_arr) == len(sc1_arr)):
            self._robot_perf_prog_lbl.configure(
                text="STOPPED — not enough data", fg=YELLOW)
            self._robot_log("Robot session discarded: not enough data.", "warn")
            self._robot_ax.set_title(
                "ROBOT RESPONSE  —  session discarded (not enough data)",
                color=YELLOW, fontsize=10, loc="left", pad=10)
            self._robot_canvas.draw_idle()
            self._popout_sync("robot")
            return

        try:
            rng_pr      = sc2_arr.max() - sc2_arr.min()
            r2_pr       = float(r2_score(pred_arr, sc2_arr))
            rmse_pr     = float(np.sqrt(mean_squared_error(pred_arr, sc2_arr)))
            nrmse_pr    = rmse_pr / (rng_pr + 1e-9) * 100
            pear_pr, _  = pearsonr(pred_arr, sc2_arr) if len(pred_arr) > 2 else (0, 0)
            pear_pr     = float(pear_pr)
            lag_s_pr    = compute_signal_lag_seconds(pred_arr, sc2_arr, ts_arr, max_lag_seconds=0.30)
            lag_ms_pr   = None if lag_s_pr is None else lag_s_pr * 1000.0

            rng_hr      = sc2_arr.max() - sc2_arr.min()
            r2_hr       = float(r2_score(sc1_arr, sc2_arr))
            rmse_hr     = float(np.sqrt(mean_squared_error(sc1_arr, sc2_arr)))
            nrmse_hr    = rmse_hr / (rng_hr + 1e-9) * 100
            pear_hr, _  = pearsonr(sc1_arr, sc2_arr) if len(sc1_arr) > 2 else (0, 0)
            pear_hr     = float(pear_hr)
            lag_s_hr    = compute_signal_lag_seconds(sc1_arr, sc2_arr, ts_arr, max_lag_seconds=0.30)
            lag_ms_hr   = None if lag_s_hr is None else lag_s_hr * 1000.0
        except Exception as e:
            self._robot_perf_prog_lbl.configure(text=f"ERROR — {e}", fg=RED)
            self._robot_log(f"Robot session metrics failed: {e}", "err")
            return

        user_id    = (self._robot_perf_user_var.get()    or "").strip() or "SUB_001"
        session_id = (self._robot_perf_session_var.get() or "").strip() or time.strftime("S_%H%M%S")
        summary = {
            "user_id":      user_id,
            "session_id":   session_id,
            "n":            int(len(pred_arr)),
            "pr_r2":        r2_pr,
            "pr_pct_rmse":  nrmse_pr,
            "pr_pearson_r": pear_pr,
            "pr_lag_ms":    lag_ms_pr,
            "hr_r2":        r2_hr,
            "hr_pct_rmse":  nrmse_hr,
            "hr_pearson_r": pear_hr,
            "hr_lag_ms":    lag_ms_hr,
        }
        STATE.robot_sessions.append(summary)
        STATE.robot_session_traces.append({
            "ts": ts_arr.copy(), "pred": pred_arr.copy(),
            "sc1": sc1_arr.copy(), "sc2": sc2_arr.copy(),
            # Snapshot the guide shape as it was AT RECORDING TIME — the live
            # _overlay_t_path/_overlay_y_path can change later (ramp/hold
            # settings edited, effort graph updated) before the user exports
            # PNGs for all sessions in one batch, which would otherwise draw
            # every old session against today's current guide shape instead
            # of the one it was actually recorded against.
            "guide_t": list(getattr(self, "_overlay_t_path", [])),
            "guide_y": list(getattr(self, "_overlay_y_path", [])),
            "guide_total_t": getattr(self, "_overlay_total_t", 0.0),
        })
        self._robot_append_session_row(summary)
        self._robot_update_avg_metrics()
        self._robot_save_session_csv(summary, ts_arr, pred_arr, sc1_arr, sc2_arr)

        done_text  = "STOPPED EARLY" if stopped_early else "COMPLETE"
        lag_pr_txt = "—" if lag_ms_pr is None else f"{lag_ms_pr:.0f} ms"
        lag_hr_txt = "—" if lag_ms_hr is None else f"{lag_ms_hr:.0f} ms"
        result_color = GREEN if not stopped_early else YELLOW
        self._robot_perf_prog_lbl.configure(
            text=(f"{done_text}  {user_id}/{session_id}  |  "
                  f"Pred→Robot R²={r2_pr:.3f}  Human→Robot R²={r2_hr:.3f}"),
            fg=result_color)
        self._robot_ax.set_title(
            (f"{done_text}  Pred→Robot R²={r2_pr:.3f} r={pear_pr:.3f}  |  "
             f"Human→Robot R²={r2_hr:.3f} r={pear_hr:.3f}"),
            color=result_color, fontsize=9, loc="left", pad=10)
        self._robot_canvas.draw_idle()
        self._popout_sync("robot")
        self._robot_log(
            f"ALL DATA ({len(pred_arr)} samples)  "
            f"Pred→Robot: R²={r2_pr:.3f} %RMSE={nrmse_pr:.1f}% r={pear_pr:.3f} lag={lag_pr_txt}  "
            f"Human→Robot: R²={r2_hr:.3f} %RMSE={nrmse_hr:.1f}% r={pear_hr:.3f} lag={lag_hr_txt}", GREEN)

        # ── Ramp-up-only comparison ───────────────────────────────────────
        ru_mask = self._robot_rampup_mask(ts_arr)
        n_ru = int(ru_mask.sum())
        if n_ru >= 10:
            try:
                pred_ru = pred_arr[ru_mask]
                sc1_ru  = sc1_arr[ru_mask]
                sc2_ru  = sc2_arr[ru_mask]
                ts_ru   = ts_arr[ru_mask]

                rng_ru          = sc2_ru.max() - sc2_ru.min()
                r2_ru_pr        = float(r2_score(pred_ru, sc2_ru))
                rmse_ru_pr      = float(np.sqrt(mean_squared_error(pred_ru, sc2_ru)))
                nrmse_ru_pr     = rmse_ru_pr / (rng_ru + 1e-9) * 100
                pear_ru_pr, _   = pearsonr(pred_ru, sc2_ru) if n_ru > 2 else (0, 0)
                lag_s_ru_pr     = compute_signal_lag_seconds(pred_ru, sc2_ru, ts_ru, max_lag_seconds=0.30)
                lag_ms_ru_pr    = None if lag_s_ru_pr is None else lag_s_ru_pr * 1000.0

                r2_ru_hr        = float(r2_score(sc1_ru, sc2_ru))
                rmse_ru_hr      = float(np.sqrt(mean_squared_error(sc1_ru, sc2_ru)))
                nrmse_ru_hr     = rmse_ru_hr / (rng_ru + 1e-9) * 100
                pear_ru_hr, _   = pearsonr(sc1_ru, sc2_ru) if n_ru > 2 else (0, 0)
                lag_s_ru_hr     = compute_signal_lag_seconds(sc1_ru, sc2_ru, ts_ru, max_lag_seconds=0.30)
                lag_ms_ru_hr    = None if lag_s_ru_hr is None else lag_s_ru_hr * 1000.0

                lag_ru_pr_txt = "—" if lag_ms_ru_pr is None else f"{lag_ms_ru_pr:.0f} ms"
                lag_ru_hr_txt = "—" if lag_ms_ru_hr is None else f"{lag_ms_ru_hr:.0f} ms"
                self._robot_log(
                    f"RAMP-UP ONLY ({n_ru} samples)  "
                    f"Pred→Robot: R²={r2_ru_pr:.3f} %RMSE={nrmse_ru_pr:.1f}% r={pear_ru_pr:.3f} lag={lag_ru_pr_txt}  "
                    f"Human→Robot: R²={r2_ru_hr:.3f} %RMSE={nrmse_ru_hr:.1f}% r={pear_ru_hr:.3f} lag={lag_ru_hr_txt}", ACCENT)

                # Δ comparison
                def _fmt_delta(full, ru):
                    d = ru - full
                    return f"+{d:.3f}" if d >= 0 else f"{d:.3f}"
                self._robot_log(
                    f"  Δ vs all data  "
                    f"Pred→Robot: ΔR²={_fmt_delta(r2_pr, r2_ru_pr)}  Δ%RMSE={_fmt_delta(nrmse_pr, nrmse_ru_pr):.1f}%  "
                    f"Human→Robot: ΔR²={_fmt_delta(r2_hr, r2_ru_hr)}  Δ%RMSE={_fmt_delta(nrmse_hr, nrmse_ru_hr):.1f}%", MUTED)
            except Exception as e:
                self._robot_log(f"Ramp-up analysis failed: {e}", YELLOW)
        else:
            self._robot_log(
                "Ramp-up analysis skipped — guide path not available or too few ramp-up samples.", MUTED)

    def _robot_append_session_row(self, summary):
        lag_pr = "—" if summary["pr_lag_ms"] is None else f"{summary['pr_lag_ms']:.0f}"
        self._robot_pr_table.insert("", tk.END, values=(
            summary["user_id"], summary["session_id"], summary["n"],
            f"{summary['pr_r2']:.3f}", f"{summary['pr_pct_rmse']:.1f}%",
            f"{summary['pr_pearson_r']:.3f}", lag_pr
        ))
        lag_hr = "—" if summary["hr_lag_ms"] is None else f"{summary['hr_lag_ms']:.0f}"
        self._robot_hr_table.insert("", tk.END, values=(
            summary["user_id"], summary["session_id"], summary["n"],
            f"{summary['hr_r2']:.3f}", f"{summary['hr_pct_rmse']:.1f}%",
            f"{summary['hr_pearson_r']:.3f}", lag_hr
        ))

    def _robot_update_avg_metrics(self):
        if not STATE.robot_sessions:
            for w in [self._robot_pr_avg_r2, self._robot_pr_avg_rmse,
                      self._robot_pr_avg_r,  self._robot_pr_avg_lag,
                      self._robot_hr_avg_r2, self._robot_hr_avg_rmse,
                      self._robot_hr_avg_r,  self._robot_hr_avg_lag]:
                w.configure(text="—")
            return
        pr_r2   = np.array([s["pr_r2"]        for s in STATE.robot_sessions], dtype=float)
        pr_rmse = np.array([s["pr_pct_rmse"]  for s in STATE.robot_sessions], dtype=float)
        pr_r    = np.array([s["pr_pearson_r"] for s in STATE.robot_sessions], dtype=float)
        pr_lag  = np.array([s["pr_lag_ms"]    for s in STATE.robot_sessions
                            if s["pr_lag_ms"] is not None], dtype=float)
        hr_r2   = np.array([s["hr_r2"]        for s in STATE.robot_sessions], dtype=float)
        hr_rmse = np.array([s["hr_pct_rmse"]  for s in STATE.robot_sessions], dtype=float)
        hr_r    = np.array([s["hr_pearson_r"] for s in STATE.robot_sessions], dtype=float)
        hr_lag  = np.array([s["hr_lag_ms"]    for s in STATE.robot_sessions
                            if s["hr_lag_ms"] is not None], dtype=float)
        self._robot_pr_avg_r2.configure(  text=f"{pr_r2.mean():.3f}")
        self._robot_pr_avg_rmse.configure(text=f"{pr_rmse.mean():.1f}%")
        self._robot_pr_avg_r.configure(   text=f"{pr_r.mean():.3f}")
        self._robot_pr_avg_lag.configure( text="—" if len(pr_lag) == 0 else f"{pr_lag.mean():.0f}")
        self._robot_hr_avg_r2.configure(  text=f"{hr_r2.mean():.3f}")
        self._robot_hr_avg_rmse.configure(text=f"{hr_rmse.mean():.1f}%")
        self._robot_hr_avg_r.configure(   text=f"{hr_r.mean():.3f}")
        self._robot_hr_avg_lag.configure( text="—" if len(hr_lag) == 0 else f"{hr_lag.mean():.0f}")

    def _robot_clear_sessions(self):
        if not messagebox.askyesno("Clear", "Clear in-memory robot session table?"):
            return
        STATE.robot_sessions.clear()
        STATE.robot_detail_rows.clear()
        STATE.robot_session_traces.clear()
        for item in self._robot_pr_table.get_children():
            self._robot_pr_table.delete(item)
        for item in self._robot_hr_table.get_children():
            self._robot_hr_table.delete(item)
        self._robot_update_avg_metrics()
        self._robot_perf_prog_lbl.configure(text="READY", fg=MUTED)

    def _robot_save_session_csv(self, summary, ts_arr, pred_arr, sc1_arr, sc2_arr):
        folder = os.path.join(os.path.expanduser("~"), "Desktop", "PropControl_RobotResponse")
        os.makedirs(folder, exist_ok=True)
        detail_path  = os.path.join(folder, "robot_sessions.csv")
        summary_path = os.path.join(folder, "robot_summary.csv")

        detail_exists = os.path.exists(detail_path)
        with open(detail_path, "a", newline="") as f:
            w = csv.writer(f)
            if not detail_exists:
                w.writerow(["user_id", "session_id", "time_s",
                            "pred_pct_mvc", "human_pct_mvc", "robot_pct_rfo"])
            for t, p, h, s in zip(ts_arr, pred_arr, sc1_arr, sc2_arr):
                w.writerow([summary["user_id"], summary["session_id"],
                            f"{t:.4f}", f"{p:.4f}", f"{h:.4f}", f"{s:.4f}"])
                STATE.robot_detail_rows.append({
                    "user_id": summary["user_id"],
                    "session_id": summary["session_id"],
                    "time_s": float(t),
                    "pred_pct_mvc": float(p),
                    "human_pct_mvc": float(h),
                    "robot_pct_rfo": float(s),
                })

        summary_exists = os.path.exists(summary_path)
        with open(summary_path, "a", newline="") as f:
            w = csv.writer(f)
            if not summary_exists:
                w.writerow(["user_id", "session_id", "n",
                            "pr_r2", "pr_pct_rmse", "pr_pearson_r", "pr_lag_ms",
                            "hr_r2", "hr_pct_rmse", "hr_pearson_r", "hr_lag_ms"])
            w.writerow([summary["user_id"], summary["session_id"], summary["n"],
                        f"{summary['pr_r2']:.6f}", f"{summary['pr_pct_rmse']:.6f}",
                        f"{summary['pr_pearson_r']:.6f}",
                        "" if summary["pr_lag_ms"] is None else f"{summary['pr_lag_ms']:.3f}",
                        f"{summary['hr_r2']:.6f}", f"{summary['hr_pct_rmse']:.6f}",
                        f"{summary['hr_pearson_r']:.6f}",
                        "" if summary["hr_lag_ms"] is None else f"{summary['hr_lag_ms']:.3f}"])

    def _robot_export_csv(self):
        if not STATE.robot_sessions:
            messagebox.showwarning("No data", "No robot response sessions recorded yet.")
            return
        timestamp    = time.strftime("%Y%m%d_%H%M%S")
        default_name = f"robot_response_summary_{timestamp}.csv"
        path = filedialog.asksaveasfilename(
            title="Save Robot Response Summary",
            initialfile=default_name,
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        sessions = STATE.robot_sessions
        pr_r2   = np.array([s["pr_r2"]        for s in sessions], dtype=float)
        pr_rmse = np.array([s["pr_pct_rmse"]  for s in sessions], dtype=float)
        pr_r    = np.array([s["pr_pearson_r"] for s in sessions], dtype=float)
        pr_lag  = np.array([s["pr_lag_ms"]    for s in sessions
                            if s["pr_lag_ms"] is not None], dtype=float)
        hr_r2   = np.array([s["hr_r2"]        for s in sessions], dtype=float)
        hr_rmse = np.array([s["hr_pct_rmse"]  for s in sessions], dtype=float)
        hr_r    = np.array([s["hr_pearson_r"] for s in sessions], dtype=float)
        hr_lag  = np.array([s["hr_lag_ms"]    for s in sessions
                            if s["hr_lag_ms"] is not None], dtype=float)
        try:
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["user_id", "session_id", "n",
                            "pr_r2", "pr_pct_rmse", "pr_pearson_r", "pr_lag_ms",
                            "hr_r2", "hr_pct_rmse", "hr_pearson_r", "hr_lag_ms"])
                for s in sessions:
                    w.writerow([s["user_id"], s["session_id"], s["n"],
                                f"{s['pr_r2']:.6f}", f"{s['pr_pct_rmse']:.6f}",
                                f"{s['pr_pearson_r']:.6f}",
                                "" if s["pr_lag_ms"] is None else f"{s['pr_lag_ms']:.3f}",
                                f"{s['hr_r2']:.6f}", f"{s['hr_pct_rmse']:.6f}",
                                f"{s['hr_pearson_r']:.6f}",
                                "" if s["hr_lag_ms"] is None else f"{s['hr_lag_ms']:.3f}"])
                w.writerow([])
                w.writerow(["MEAN", "", len(sessions),
                            f"{pr_r2.mean():.6f}", f"{pr_rmse.mean():.6f}",
                            f"{pr_r.mean():.6f}",
                            "" if len(pr_lag) == 0 else f"{pr_lag.mean():.3f}",
                            f"{hr_r2.mean():.6f}", f"{hr_rmse.mean():.6f}",
                            f"{hr_r.mean():.6f}",
                            "" if len(hr_lag) == 0 else f"{hr_lag.mean():.3f}"])
                w.writerow(["STD", "", "",
                            f"{pr_r2.std():.6f}", f"{pr_rmse.std():.6f}",
                            f"{pr_r.std():.6f}",
                            "" if len(pr_lag) == 0 else f"{pr_lag.std():.3f}",
                            f"{hr_r2.std():.6f}", f"{hr_rmse.std():.6f}",
                            f"{hr_r.std():.6f}",
                            "" if len(hr_lag) == 0 else f"{hr_lag.std():.3f}"])
            self._robot_log(f"Exported robot summary → {path}", GREEN)
        except Exception as e:
            messagebox.showerror("Export failed", str(e))

    def _robot_export_all_data_csv(self):
        """Export every recorded robot-session sample to a CSV chosen by the user."""
        if not STATE.robot_detail_rows:
            messagebox.showwarning("No data", "No robot response trial samples recorded yet.")
            return

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        default_name = f"robot_all_data_{timestamp}.csv"
        path = filedialog.asksaveasfilename(
            title="Save All Robot Response Trial Data",
            initialfile=default_name,
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return

        try:
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["user_id", "session_id", "time_s",
                            "pred_pct_mvc", "human_pct_mvc", "robot_pct_rfo"])
                for row in STATE.robot_detail_rows:
                    w.writerow([
                        row["user_id"], row["session_id"], f"{row['time_s']:.4f}",
                        f"{row['pred_pct_mvc']:.4f}", f"{row['human_pct_mvc']:.4f}",
                        f"{row['robot_pct_rfo']:.4f}",
                    ])
            self._robot_log(f"Robot all-data export → {os.path.basename(path)}", GREEN)
            messagebox.showinfo("Saved", f"All robot response trial data saved to:\n{path}")
        except Exception as e:
            self._robot_log(f"Robot all-data export error: {e}", RED)
            messagebox.showerror("Export failed", str(e))

    def _robot_export_png(self):
        """Export a PNG of every recorded robot session in one go — same
        pattern as the Performance tab's PNG export. Each session's raw
        ts/pred/sc1/sc2 arrays are auto-captured into
        STATE.robot_session_traces the moment it finishes, so this just
        batch-renders all of them now, always showing all three traces
        regardless of the current PREDICTION/ROBOT GRIP toggle state."""
        if not STATE.robot_sessions:
            messagebox.showwarning("No data", "No robot response sessions recorded yet.")
            return

        out_dir = filedialog.askdirectory(title="Choose folder to export session PNGs")
        if not out_dir:
            return

        used_names = set()
        n_saved = 0
        for summary, trace in zip(STATE.robot_sessions, STATE.robot_session_traces):
            base = f"{summary['user_id']}_{summary['session_id']}"
            base = "".join(c if (c.isalnum() or c in "_-") else "_" for c in base)
            name = base
            i = 2
            while name in used_names:
                name = f"{base}_{i}"; i += 1
            used_names.add(name)
            try:
                self._robot_render_session_png(summary, trace,
                                                os.path.join(out_dir, name + ".png"))
                n_saved += 1
            except Exception as e:
                self._robot_log(f"PNG export failed for {base}: {e}", RED)

        self._robot_log(f"Exported {n_saved} robot session PNG(s) → {out_dir}", GREEN)
        messagebox.showinfo("Export complete",
                             f"Saved {n_saved} session PNG(s) to:\n{out_dir}")

    def _robot_render_session_png(self, summary, trace, path):
        """Render one robot session's full graph (target guide + human grip
        + prediction + robot grip) as a standalone PNG — independent of the
        live canvas, so it's unaffected by the PREDICTION/ROBOT GRIP toggle
        state."""
        ts   = trace["ts"]
        pred = trace["pred"]
        sc1  = trace["sc1"]
        sc2  = trace["sc2"]
        # Use the guide shape as it was captured AT RECORDING TIME for this
        # session, not whatever the live effort-graph settings are now —
        # otherwise batch-exporting older sessions after changing ramp/hold
        # settings draws them against today's guide instead of their own.
        guide_t = trace.get("guide_t", [])
        guide_y = trace.get("guide_y", [])
        guide_total_t = trace.get("guide_total_t", 0.0)

        fig = Figure(figsize=(11, 5.8), facecolor=PANEL)
        fig.subplots_adjust(top=0.80, bottom=0.14, left=0.08, right=0.97)
        ax = fig.add_subplot(111)
        ax.set_facecolor(PANEL2)
        for sp in ax.spines.values():
            sp.set_color(BORDER); sp.set_linewidth(0.8)
        ax.tick_params(colors=MUTED, labelsize=11)
        ax.set_xlabel("Time (s)", color=MUTED, fontsize=12, labelpad=6)
        ax.set_ylabel("% Force", color=MUTED, fontsize=12, labelpad=8)

        if len(guide_t):
            ax.plot(guide_t, guide_y, color=RED, lw=3.0,
                    zorder=3, label="Target", solid_capstyle="round")
            for pct in [25, 50, 75, 100]:
                ax.axhline(pct, color=MUTED, lw=0.5, linestyle=":", zorder=1, alpha=0.5)

        ax.plot(ts, sc1, color=GREEN, lw=2.2, zorder=4,
                label="Human Grip (% MVC)", alpha=0.9, solid_capstyle="round")
        ax.plot(ts, pred, color=ACCENT, lw=2.2, zorder=5, linestyle="--",
                label="Prediction (% MVC)", alpha=0.9, solid_capstyle="round")
        ax.plot(ts, sc2, color=ACCENT2, lw=2.2, zorder=6,
                label="Robot Grip (% RFO)", alpha=0.9, solid_capstyle="round")

        x_max = max(float(ts.max()) if len(ts) else 1.0, guide_total_t)
        y_max = max(float(sc1.max()) if len(sc1) else 0.0,
                    float(pred.max()) if len(pred) else 0.0,
                    float(sc2.max()) if len(sc2) else 0.0, 100.0)
        ax.set_xlim(0, x_max * 1.02)
        ax.set_ylim(0, y_max * 1.15)

        lag_pr_txt = "—" if summary["pr_lag_ms"] is None else f"{summary['pr_lag_ms']:.0f} ms"
        lag_hr_txt = "—" if summary["hr_lag_ms"] is None else f"{summary['hr_lag_ms']:.0f} ms"
        ax.set_title(
            f"{summary['user_id']} / {summary['session_id']}  ({summary['n']} windows)\n"
            f"Pred→Robot: R²={summary['pr_r2']:.3f} %RMSE={summary['pr_pct_rmse']:.1f}% "
            f"r={summary['pr_pearson_r']:.3f} lag={lag_pr_txt}   |   "
            f"Human→Robot: R²={summary['hr_r2']:.3f} %RMSE={summary['hr_pct_rmse']:.1f}% "
            f"r={summary['hr_pearson_r']:.3f} lag={lag_hr_txt}",
            color=TEXT, fontsize=10.5, loc="left", pad=12, fontweight="bold")

        ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.10),
                  ncol=4, fontsize=11, labelcolor=TEXT,
                  facecolor=PANEL2, edgecolor=BORDER, framealpha=0.85,
                  borderpad=0.8, handlelength=2.0, columnspacing=2.0)

        fig.savefig(path, dpi=150, facecolor=PANEL)

    def _robot_scan_ports(self):
        try:
            import serial.tools.list_ports as _lp
            ports = list(_lp.comports())
        except Exception as e:
            self._robot_log(f"Scan error: {e}", RED)
            return
        menu = self._robot_port_menu["menu"]
        menu.delete(0, "end")
        if not ports:
            self._robot_port_var.set("-- no ports found --")
            self._robot_log("No serial ports found.", YELLOW)
            return
        for p in ports:
            menu.add_command(label=p.device,
                             command=lambda v=p.device: self._robot_port_var.set(v))
        self._robot_port_var.set(ports[0].device)
        self._robot_log(f"Found {len(ports)} port(s).", MUTED)

    def _robot_refresh_gdx_status(self):
        """Reflect live GDX connection state on the robot tab. Both scales
        are opened together from the main "CONNECT GDX" button (sidebar or
        exo tab) — there's no separate connect action here, so this just
        mirrors STATE/_gdx_device_count whenever it changes."""
        try:
            if not STATE.connected_gdx or _gdx_device_count <= 0:
                self._robot_gdx_btn.configure(text="⬡  GDX: NOT CONNECTED", fg=MUTED)
                self._robot_gdx_status_lbl.configure(text="  |  GDX: —", fg=MUTED)
            elif _gdx_device_count >= 2:
                self._robot_gdx_btn.configure(text="✔  GDX: SCALE 1 + 2 LIVE", fg=GREEN)
                self._robot_gdx_status_lbl.configure(
                    text="  |  GDX: Scale 1 + 2 LIVE", fg=GREEN)
            else:
                self._robot_gdx_btn.configure(text="✔  GDX: SCALE 1 ONLY", fg=YELLOW)
                self._robot_gdx_status_lbl.configure(
                    text="  |  GDX: Scale 1 only (plug in scale 2 + reconnect)", fg=YELLOW)
        except Exception:
            pass

    def _robot_toggle_serial(self):
        if self._robot_serial and self._robot_serial.is_open:
            self._robot_disconnect_serial()
        else:
            self._robot_connect_serial()

    def _robot_connect_serial(self):
        port = self._robot_port_var.get()
        if not port or "scan" in port.lower() or "no ports" in port.lower():
            self._robot_log("Select a port first (use ⟳ scan).", RED)
            return
        port = port.split("  ←")[0].strip()
        try:
            baud = int(self._robot_baud_var.get())
        except ValueError:
            baud = 115200

        try:
            import serial as _ser
            s = _ser.Serial()
            s.port          = port
            s.baudrate      = baud
            s.timeout       = 0.15   # short readline timeout so handshake loop stays responsive
            s.write_timeout = 1.0
            s.dsrdtr        = False
            s.rtscts        = False
            s.open()
            # Do NOT reset_input_buffer here — the Arduino immediately sends its
            # boot and calibration prompts; clearing the buffer would eat them.
            self._robot_serial = s
            self._robot_serial_btn.configure(text="◈  DISCONNECT ROBOT", fg=RED)
            self._robot_status_lbl.configure(
                text="●  WAITING — calibration in progress…", fg=YELLOW)
            # Stream button stays DISABLED until Arduino sends "READY"
            self._robot_stream_btn.configure(state=tk.DISABLED)
            self._robot_cal_lbl.configure(
                text="Waiting for Arduino boot messages…", fg=YELLOW)
            self._robot_log(f"Robot serial opened: {port} @ {baud}  — awaiting calibration", GREEN)
            # Start handshake reader; it enables the stream button when "READY" arrives
            self._robot_handshake_running = True
            threading.Thread(target=self._robot_handshake_loop,
                             args=(port, baud), daemon=True).start()
        except Exception as e:
            self._robot_log(f"Serial connect failed: {e}", RED)
            self._robot_status_lbl.configure(text="●  CONNECTION FAILED", fg=RED)

    def _robot_disconnect_serial(self):
        if self._robot_streaming:
            self._robot_stop_stream()
        self._robot_rx_running        = False
        self._robot_handshake_running = False
        try:
            if self._robot_serial and self._robot_serial.is_open:
                self._robot_serial.close()
        except Exception:
            pass
        self._robot_serial = None
        self._robot_serial_btn.configure(text="⬡  CONNECT ROBOT", fg=ACCENT)
        self._robot_status_lbl.configure(text="●  DISCONNECTED", fg=RED)
        self._robot_stream_btn.configure(
            state=tk.DISABLED, text="▶  START STREAM", fg=ACCENT)
        self._robot_cal_close_btn.configure(state=tk.DISABLED, fg=MUTED)
        self._robot_cal_open_btn.configure(state=tk.DISABLED, fg=MUTED)
        self._robot_cal_lbl.configure(text="—  connect robot serial to begin", fg=MUTED)
        self._robot_log("Robot serial disconnected.", YELLOW)

    def _robot_toggle_stream(self):
        if self._robot_streaming:
            self._robot_stop_stream()
        else:
            self._robot_start_stream()

    def _robot_start_stream(self):
        if not STATE.model_trained:
            self._robot_log("Train a model first (02 TRAIN).", RED)
            return
        if not (self._robot_serial and self._robot_serial.is_open):
            self._robot_log("Connect to robot serial first.", RED)
            return
        if not STATE.connected_gdx or _gdx_device_count < 1:
            self._robot_log(
                "GDX not connected — GDX traces will show zeros.", YELLOW)
        elif _gdx_device_count < 2:
            self._robot_log(
                "Only scale 1 connected — robot pincher trace will show zeros.", YELLOW)

        if not STATE.inferring:
            STATE.inferring = True
            STATE._ema_last = None
            self._robot_log("Inference auto-started.", ACCENT)

        # Clear Arduino "stopped" state: after a STOP command the firmware
        # silently ignores all non-zero C, commands until it sees goal_mA=0.
        # Send C,0,0 first so any subsequent command is obeyed immediately.
        try:
            if self._robot_serial and self._robot_serial.is_open:
                self._robot_serial.write(b"C,0,0\n")
        except Exception:
            pass

        self._robot_streaming    = True
        self._robot_tx_total     = 0
        self._robot_err_total    = 0
        self._robot_t0           = time.time()
        self._robot_last_cmd_ma  = 0
        self._robot_latest_enc   = 0
        self._robot_latest_meas_ma = 0.0

        self._robot_t_hist.clear()
        self._robot_sc1_hist.clear()
        self._robot_sc2_hist.clear()
        self._robot_pred_hist.clear()
        self._robot_cmd_ma_hist.clear()
        self._robot_enc_hist.clear()
        self._robot_meas_ma_hist.clear()

        # Start background RX reader for S,<enc>,<mA> telemetry
        self._robot_rx_running = True
        self._robot_rx_thread  = threading.Thread(
            target=self._robot_rx_loop, daemon=True)
        self._robot_rx_thread.start()

        self._robot_stream_btn.configure(text="■  STOP STREAM", fg=RED)
        self._robot_status_lbl.configure(
            text=f"●  STREAMING  @ {self._robot_rate_var.get()} Hz", fg=GREEN)
        self._robot_log(
            f"Stream started @ {self._robot_rate_var.get()} Hz  "
            f"| filtered={'yes' if self._robot_use_filtered_var.get() else 'no'}", GREEN)
        self._robot_stream_tick()

    def _robot_stop_stream(self):
        self._robot_streaming  = False
        self._robot_rx_running = False
        if self._robot_stream_after:
            try:
                self.after_cancel(self._robot_stream_after)
            except Exception:
                pass
            self._robot_stream_after = None

        # If a session was recording, finish it early
        if STATE.robot_recording:
            self._robot_finish_session(stopped_early=True)

        # Zero the motor immediately
        try:
            if self._robot_serial and self._robot_serial.is_open:
                with self._robot_write_lock:
                    self._robot_serial.write(b"STOP\n")
        except Exception:
            pass

        self._robot_stream_btn.configure(text="▶  START STREAM", fg=ACCENT)
        port = self._robot_port_var.get().split("  ←")[0].strip()
        self._robot_status_lbl.configure(
            text=f"●  CONNECTED  {port}  (stopped)", fg=YELLOW)
        self._robot_log(
            f"Stream stopped. TX: {self._robot_tx_total}  "
            f"Errors: {self._robot_err_total}", YELLOW)

    def _robot_handshake_loop(self, port, baud):
        """Background thread: reads the Arduino's boot/calibration sequence.

        The Arduino (GripReleaseController) blocks in setup() twice:
          1. "Move to CLOSE position, press ENTER..."
          2. "Move to OPEN position, press ENTER..."
        Each requires us to send \\n (via _robot_send_enter).  After both,
        the Arduino writes "READY" and enters command mode.

        If neither calibration prompt arrives within ~5 s (Arduino already
        calibrated / no reset on connect), we assume it's ready immediately.
        """
        _CAL_DETECT_TIMEOUT = 5.0   # if no "Move to…" prompt by this time, skip cal
        _READY_TIMEOUT      = 120.0  # absolute cap — Arduino must say READY within 2 min

        t_start = time.time()
        cal_detected = False

        while self._robot_handshake_running:
            if not self._robot_serial or not self._robot_serial.is_open:
                break

            elapsed = time.time() - t_start

            # If no calibration prompt seen within the detection window, assume ready
            if not cal_detected and elapsed > _CAL_DETECT_TIMEOUT:
                self.after(0, lambda: self._robot_on_handshake_ready(port, baud,
                    note="(no calibration prompts — Arduino assumed already ready)"))
                return

            if elapsed > _READY_TIMEOUT:
                self.after(0, lambda: self._robot_log(
                    "Handshake timeout — try disconnecting and reconnecting.", RED))
                return

            try:
                line = self._robot_serial.readline().decode(errors="ignore").strip()
            except Exception:
                break

            if not line:
                continue

            ll = line.lower()

            if "close position" in ll:
                cal_detected = True
                self.after(0, lambda: self._robot_on_cal_prompt("close"))
            elif "open position" in ll:
                self.after(0, lambda: self._robot_on_cal_prompt("open"))
            elif line.upper().startswith("READY"):
                # Parse "READY span=150 sign=1" to get calibration geometry.
                import re as _re
                m_span = _re.search(r'span=(\d+)',  line, _re.IGNORECASE)
                m_sign = _re.search(r'sign=(-?\d+)', line, _re.IGNORECASE)
                span = int(m_span.group(1)) if m_span else 0
                sign = int(m_sign.group(1)) if m_sign else 1
                self.after(0, lambda s=span, sg=sign: self._robot_on_handshake_ready(
                    port, baud, span_abs=s, sign_open=sg))
                return
            elif line.startswith("S,"):
                # Arduino is already streaming (no reset happened)
                self.after(0, lambda: self._robot_on_handshake_ready(port, baud,
                    note="(Arduino already in command mode)"))
                return
            else:
                self.after(0, lambda l=line: self._robot_log(f"[Arduino] {l[:70]}", MUTED))

    def _robot_on_cal_prompt(self, which):
        """Called on the main thread when the Arduino asks for a cal position."""
        if which == "close":
            self._robot_cal_close_btn.configure(state=tk.NORMAL, fg=GREEN)
            self._robot_cal_open_btn.configure(state=tk.DISABLED, fg=MUTED)
            self._robot_cal_lbl.configure(
                text="Move gripper to CLOSED position, then click CONFIRM CLOSE POS", fg=YELLOW)
            self._robot_log(
                "Arduino asks: move gripper to CLOSE position, then click CONFIRM CLOSE POS",
                YELLOW)
        else:
            self._robot_cal_close_btn.configure(state=tk.DISABLED, fg=MUTED)
            self._robot_cal_open_btn.configure(state=tk.NORMAL, fg=GREEN)
            self._robot_cal_lbl.configure(
                text="Move gripper to OPEN position, then click CONFIRM OPEN POS", fg=YELLOW)
            self._robot_log(
                "Arduino asks: move gripper to OPEN position, then click CONFIRM OPEN POS",
                YELLOW)

    def _robot_on_handshake_ready(self, port, baud, span_abs=0, sign_open=1, note=""):
        """Called on the main thread when the Arduino signals READY."""
        self._robot_handshake_running = False
        self._robot_sign_open = sign_open
        # If Arduino reported a span, use it and populate the UI field.
        # If not (old firmware or missed READY), keep whatever the user typed.
        if span_abs > 0:
            self._robot_span_abs = span_abs
            self._robot_span_var.set(str(span_abs))
            span_txt = f"  span={span_abs} ticks"
        else:
            span_txt = "  (span unknown — enter manually in Span field)"
        self._robot_cal_close_btn.configure(state=tk.DISABLED, fg=MUTED)
        self._robot_cal_open_btn.configure(state=tk.DISABLED, fg=MUTED)
        self._robot_cal_lbl.configure(
            text=f"✔  Calibrated — ready to stream{span_txt}", fg=GREEN)
        self._robot_stream_btn.configure(state=tk.NORMAL)
        self._robot_status_lbl.configure(
            text=f"●  CONNECTED  {port} @ {baud}", fg=GREEN)
        msg = f"Arduino READY{span_txt}.  {note}".strip()
        self._robot_log(msg, GREEN)

    def _robot_send_enter(self, which):
        """Send \\n to the Arduino to confirm a calibration position."""
        try:
            if self._robot_serial and self._robot_serial.is_open:
                with self._robot_write_lock:
                    self._robot_serial.write(b"\n")
            if which == "close":
                self._robot_cal_close_btn.configure(state=tk.DISABLED, fg=MUTED)
                self._robot_cal_lbl.configure(
                    text="CLOSE recorded — waiting for OPEN prompt…", fg=YELLOW)
                self._robot_log("Sent CLOSE position to Arduino.", GREEN)
            else:
                self._robot_cal_open_btn.configure(state=tk.DISABLED, fg=MUTED)
                self._robot_cal_lbl.configure(
                    text="OPEN recorded — waiting for Arduino READY…", fg=YELLOW)
                self._robot_log("Sent OPEN position to Arduino.", GREEN)
        except Exception as e:
            self._robot_log(f"Calibration send error: {e}", RED)

    def _robot_force_to_ma(self, force_n):
        """Map predicted grip force (N) → motor current (mA).
        Uses force_mvc_reference (calibrated human MVC) so that 100% MVC
        prediction → max_ma.  Falls back to the UI field only when MVC has
        not yet been calibrated (force_mvc_reference is None / 0).
        """
        try:
            max_ma = int(float(self._robot_max_ma_var.get()))
        except Exception:
            max_ma = 1000
        max_ma = max(1, max_ma)
        # Prefer the session-calibrated MVC so the mapping is consistent with
        # every other % display in the app.  If MVC hasn't been run yet, fall
        # back to the manually entered UI field.
        mvc_n = force_mvc_reference
        if not mvc_n or mvc_n <= 0:
            try:
                mvc_n = max(1e-6, float(self._robot_mvc_force_var.get()))
            except Exception:
                mvc_n = 30.0
        current = int(round(min(1.0, max(0.0, float(force_n)) / mvc_n) * max_ma))
        return max(0, min(max_ma, current))

    def _robot_rx_loop(self):
        """Background thread: reads S,<encoder>,<present_mA> from Arduino."""
        _rx_unrecog = 0  # count unrecognised lines so log doesn't flood
        while self._robot_rx_running and self._robot_serial and self._robot_serial.is_open:
            try:
                raw = self._robot_serial.readline().decode(errors="ignore").strip()
            except Exception:
                break
            if not raw:
                continue
            parts = raw.split(",")
            if len(parts) == 3 and parts[0] == "S":
                try:
                    enc     = int(parts[1])
                    meas_ma = float(parts[2])
                    self._robot_latest_enc     = enc
                    self._robot_latest_meas_ma = meas_ma
                    self._robot_enc_hist.append(enc)
                    self._robot_meas_ma_hist.append(meas_ma)
                except Exception:
                    pass
            else:
                # Log every unrecognised line (first 20, then every 100th) so the
                # TX/RX log reveals wrong port, wrong baud, or mismatched firmware.
                _rx_unrecog += 1
                if _rx_unrecog <= 20 or _rx_unrecog % 100 == 0:
                    self.after(0, lambda r=raw, n=_rx_unrecog:
                        self._robot_log(f"[RX #{n}] {r[:80]}", YELLOW))

    def _robot_stream_tick(self):
        if not self._robot_streaming:
            return

        # ── Latest prediction → current (mA) ─────────────────────────────
        force_raw  = float(getattr(self, "_latest_pred_force_raw",  0.0))
        force_filt = float(getattr(self, "_latest_pred_force_filt", force_raw))
        pred_used  = max(0.0, force_filt if self._robot_use_filtered_var.get() else force_raw)
        cmd_ma     = self._robot_force_to_ma(pred_used)

        # ── Proportional position offset ──────────────────────────────────
        # pulse_ticks is now an ABSOLUTE encoder offset (0 = close_pos,
        # span_abs = open_pos) — see updated Arduino firmware.
        # High EMG prediction → offset near 0 (gripper closed, high torque).
        # Low  EMG prediction → offset near span_abs (gripper open, released).
        # pred_pct is computed below but we need it here, so compute it first.
        pred_pct_early = _force_to_pct_mvc(pred_used)
        # Read span from the UI field so manual entry takes effect immediately.
        try:
            span = max(0, int(float(self._robot_span_var.get())))
            self._robot_span_abs = span  # keep instance var in sync
        except Exception:
            span = self._robot_span_abs
        if span > 0:
            # Invert prediction: 100% MVC → 0 ticks, 0% MVC → span ticks
            pulse_ticks = int(round((1.0 - min(1.0, max(0.0, pred_pct_early / 100.0))) * span))
        else:
            # Span unknown (Arduino not yet calibrated or old firmware):
            # fall back to fully-closed so motor doesn't drift.
            pulse_ticks = 0

        # ── GDX readings — convert to normalised % for display ────────────
        global _robot_rfo
        sc1 = max(0.0, _robot_gdx_scale1[0])
        sc2 = max(0.0, _robot_gdx_scale2[0])
        if sc2 > _robot_rfo:
            _robot_rfo = sc2          # update running max (RFO grows, never shrinks)
        sc1_pct  = _force_to_pct_mvc(sc1)
        sc2_pct  = sc2 / _robot_rfo * 100.0 if _robot_rfo > 0 else 0.0
        pred_pct = pred_pct_early  # already computed above for pulse_ticks
        t        = time.time() - self._robot_t0

        # ── History (% values stored so the plot renders in % directly) ───
        self._robot_t_hist.append(t)
        self._robot_sc1_hist.append(sc1_pct)
        self._robot_sc2_hist.append(sc2_pct)
        self._robot_pred_hist.append(pred_pct)
        self._robot_cmd_ma_hist.append(cmd_ma)

        # ── Session recording sample collection ──────────────────────────
        if STATE.robot_recording:
            STATE.robot_session_pred.append(pred_pct)
            STATE.robot_session_sc1.append(sc1_pct)
            STATE.robot_session_sc2.append(sc2_pct)
            STATE.robot_session_ts.append(t)

        # ── Metric cards ──────────────────────────────────────────────────
        phase_str = f"RAMP-UP (0t)" if pulse_ticks == 0 else f"RELEASE ({pulse_ticks}t)"
        self._robot_sc1_card.configure(text=f"{sc1_pct:.1f}%")
        self._robot_pred_card.configure(text=f"{pred_pct:.1f}%")
        self._robot_sc2_card.configure(text=f"{sc2_pct:.1f}%  (RFO={_robot_rfo:.1f}N)")
        self._robot_rfo_lbl.configure(text=f"RFO = {_robot_rfo:.2f} N  ✔ calibrated", fg=GREEN)
        self._robot_cmd_card.configure(text=f"{cmd_ma} mA")
        self._robot_enc_card.configure(text=str(self._robot_latest_enc))
        self._robot_tx_card.configure(text=str(self._robot_tx_total))
        self._robot_err_card.configure(text=str(self._robot_err_total))

        # ── Serial TX — C,<mA>,<abs_offset> ──────────────────────────────
        packet = f"C,{cmd_ma},{pulse_ticks}\n"
        try:
            if self._robot_serial and self._robot_serial.is_open:
                with self._robot_write_lock:
                    # Do NOT call reset_output_buffer() here — it can silently
                    # drop the previous command if the OS TX buffer hasn't
                    # flushed yet.  MicroOpenGripControl.py never does this.
                    self._robot_serial.write(packet.encode("ascii"))
                self._robot_tx_total += 1
                self._robot_tx_card.configure(text=str(self._robot_tx_total))
                if self._robot_tx_total % 50 == 0:
                    self._robot_log(
                        f"TX #{self._robot_tx_total}  {cmd_ma} mA  {phase_str}  "
                        f"Human={sc1_pct:.1f}%MVC  Robot={sc2_pct:.1f}%RFO", GREEN)
        except Exception as e:
            self._robot_err_total += 1
            self._robot_err_card.configure(text=str(self._robot_err_total))
            self._robot_log(f"TX error: {str(e).split(chr(10))[0][:60]}", RED)

        interval_ms = max(20, int(1000 / self._robot_rate_var.get()))
        self._robot_stream_after = self.after(interval_ms, self._robot_stream_tick)

    # ════════════════════════════════════════
    #  CLEANUP
    # ════════════════════════════════════════
    def _on_close(self):
        self._stop_event.set()
        _gdx_bg_running.clear()
        if emg_serial and emg_serial.is_open:
            try: emg_serial.close()
            except: pass
        disconnect_mindrove()
        try:
            if getattr(self, "_exo_serial", None) and self._exo_serial.is_open:
                self._exo_streaming = False
                self._exo_serial.close()
        except Exception:
            pass
        try:
            _teardown_all_gdx()
        except Exception:
            pass
        try:
            if getattr(self, "_robot_serial", None) and self._robot_serial.is_open:
                self._robot_streaming = False
                self._robot_serial.close()
        except Exception:
            pass
        self.destroy()


# ─────────────────────────────────────────────
if __name__ == "__main__":
    app = App()
    app.mainloop()