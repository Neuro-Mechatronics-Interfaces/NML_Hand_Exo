"""Shared UI, LSL, and preprocessing utilities for the EMG applications."""

from __future__ import annotations

from collections import deque

import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal
from pylsl import StreamInlet, resolve_byprop

from nml_hand_exo.processing._filters import bandpass_filter, notch_filter
from nml_hand_exo.processing._features import rectify, window_rms, z_score_norm


DARK_STYLE = """
QWidget { background-color: #1a1a1a; color: #e0e0e0; font-family: "Segoe UI", "Helvetica Neue", Arial, sans-serif; }
QGroupBox { background-color: #222222; border: 1px solid #333333; border-radius: 6px; margin-top: 1.2em; padding-top: 1.0em; font-weight: bold; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; color: #c0392b; }
QPushButton { background-color: #2e2e2e; color: #e0e0e0; border: 1px solid #444444; border-radius: 4px; padding: 5px 14px; min-height: 1.4em; }
QPushButton:hover { background-color: #3a3a3a; border-color: #c0392b; }
QPushButton:pressed { background-color: #c0392b; color: #ffffff; }
QPushButton:disabled { background-color: #252525; color: #555555; border-color: #333333; }
QPushButton[accent="true"] { background-color: #8b1a1a; color: #ffffff; border-color: #c0392b; }
QPushButton[accent="true"]:hover { background-color: #a52222; }
QPushButton[accent="true"]:pressed { background-color: #c0392b; }
QLineEdit, QComboBox, QDoubleSpinBox, QSpinBox { background-color: #2a2a2a; color: #e0e0e0; border: 1px solid #444444; border-radius: 4px; padding: 4px 8px; }
QLineEdit:focus, QComboBox:focus, QDoubleSpinBox:focus, QSpinBox:focus { border-color: #c0392b; }
QComboBox::drop-down { border: none; background: #333333; width: 20px; }
QComboBox QAbstractItemView { background-color: #2a2a2a; color: #e0e0e0; selection-background-color: #c0392b; }
QTextEdit { background-color: #111111; color: #aaaaaa; border: 1px solid #333333; border-radius: 4px; font-family: "Cascadia Code", "Consolas", "Courier New", monospace; }
QScrollArea { border: none; background-color: #1a1a1a; }
QTabWidget::pane { border: 1px solid #252525; background-color: #1a1a1a; top: -1px; }
QTabBar { background-color: #0d0d0d; }
QTabBar::tab { background-color: #111111; color: #555555; border: 1px solid #1c1c1c; border-bottom: none; padding: 7px 20px; margin-right: 2px; border-top-left-radius: 4px; border-top-right-radius: 4px; font-weight: normal; min-width: 80px; }
QTabBar::tab:selected { background-color: #1a1a1a; color: #e0e0e0; border-color: #2e2e2e; border-bottom: 2px solid #c0392b; font-weight: bold; }
QTabBar::tab:hover:!selected { background-color: #161616; color: #999999; border-color: #2a2a2a; }
"""


def _parse_int_list(text: str) -> list[int]:
    if not text.strip() or text.strip().lower() == "all":
        return []
    return [int(part.strip()) for part in text.split(",") if part.strip()]


def _select_channels(data: np.ndarray, channel_indices: list[int]) -> np.ndarray:
    if not channel_indices:
        return data
    valid = [idx for idx in channel_indices if 0 <= idx < data.shape[0]]
    return data[valid, :] if valid else data


def _adaptive_bandpass_limits(fs: int) -> tuple[float, float]:
    highcut = min(450.0, 0.45 * float(fs))
    lowcut = min(20.0, max(5.0, highcut * 0.2))
    if highcut <= lowcut:
        highcut = min(max(lowcut + 1.0, 0.4 * float(fs)), 0.45 * float(fs))
        lowcut = min(lowcut, max(5.0, highcut * 0.2))
    return lowcut, highcut


def _preprocess_window(window: np.ndarray, fs: int, mode: str) -> np.ndarray:
    if mode == "Raw":
        return window
    lowcut, highcut = _adaptive_bandpass_limits(fs)
    filtered = bandpass_filter(window, lowcut=lowcut, highcut=highcut, fs=fs, order=4, axis=1)
    filtered = notch_filter(filtered, fs=fs, f0=60.0, Q=30.0, axis=1)
    if mode == "Bandpass + notch":
        return filtered
    if mode == "Rectified envelope":
        return rectify(filtered)
    if mode == "RMS envelope":
        return window_rms(filtered, window_size=max(8, int(0.05 * fs)))
    if mode == "Z-score normalized":
        return z_score_norm(filtered)
    return filtered


class EmgStreamWorker(QThread):
    status_changed = pyqtSignal(str, str)
    stream_ready = pyqtSignal(object)
    chunk_received = pyqtSignal(object, object)

    def __init__(self, stream_type: str, stream_name: str):
        super().__init__()
        self._stream_type = stream_type
        self._stream_name = stream_name
        self._stop_requested = False

    def stop(self):
        self._stop_requested = True

    def run(self):
        try:
            candidates = resolve_byprop("type", self._stream_type, timeout=2)
        except Exception as exc:
            self.status_changed.emit(f"LSL resolve failed: {exc}", "#c0392b")
            return
        if self._stream_name.strip():
            wanted = self._stream_name.strip()
            candidates = [stream for stream in candidates if stream.name() == wanted]
        if not candidates:
            label = self._stream_name.strip() or self._stream_type
            self.status_changed.emit(f"No LSL stream found for '{label}'", "#c0392b")
            return

        stream = candidates[0]
        inlet = StreamInlet(stream, max_buflen=30)
        info = inlet.info()
        fs = int(info.nominal_srate()) or 1000
        n_channels = int(info.channel_count())
        labels = []
        try:
            channel = info.desc().child("channels").child("channel")
            for index in range(n_channels):
                label = channel.child_value("label")
                labels.append(label if label else f"ch{index}")
                channel = channel.next_sibling()
        except Exception:
            labels = [f"ch{index}" for index in range(n_channels)]
        self.stream_ready.emit({"name": stream.name(), "type": stream.type(), "sample_rate": fs, "channel_count": n_channels, "labels": labels})
        self.status_changed.emit(f"Connected to {stream.name()} ({stream.type()})", "#27ae60")

        while not self._stop_requested:
            chunk, timestamps = inlet.pull_chunk(timeout=0.1, max_samples=128)
            if chunk:
                self.chunk_received.emit(np.asarray(chunk, dtype=np.float64).T, np.asarray(timestamps, dtype=np.float64))
        self.status_changed.emit("LSL stream stopped", "#888888")


class ChunkBuffer:
    def __init__(self, max_samples: int):
        self._max_samples = max(1, int(max_samples))
        self._chunks: deque[np.ndarray] = deque()
        self._count = 0

    def set_max_samples(self, max_samples: int):
        self._max_samples = max(1, int(max_samples))
        self._trim()

    def clear(self):
        self._chunks.clear()
        self._count = 0

    def append(self, chunk: np.ndarray):
        if chunk.ndim != 2 or chunk.shape[1] == 0:
            return
        self._chunks.append(np.array(chunk, copy=True))
        self._count += chunk.shape[1]
        self._trim()

    def snapshot(self) -> np.ndarray | None:
        return np.concatenate(list(self._chunks), axis=1) if self._chunks else None

    def _trim(self):
        while self._count > self._max_samples and self._chunks:
            removed = self._chunks.popleft()
            self._count -= removed.shape[1]
