import json
import os
import sys
import threading
import time
from typing import Any, Dict, List, Optional

from PyQt5.QtCore import Qt, QThread, pyqtSignal

from pylsl import StreamInlet

from nml_wtf_exo.controller.KeyboardEventWorker import KeyboardEventWorker
from nml_wtf_exo.controller.FixedRateKeyParser import FixedRateKeyParser
from nml_wtf_exo.gui.ui.KeyboardOverlay import KeyboardOverlay
from nml_wtf_exo.utils.ReplacingQueue import ReplacingQueue

class LSLKeyboardHandler(QThread):
    status = pyqtSignal(str)
    error = pyqtSignal(str)
    connected = pyqtSignal()
    disconnected = pyqtSignal()
    newline = pyqtSignal()

    def __init__(self, stream_info, worker: KeyboardEventWorker,
                 parser: Optional[FixedRateKeyParser],
                 overlay: KeyboardOverlay):
        super().__init__()
        self.stream_info = stream_info
        self.worker = worker
        self.parser = parser
        self.overlay = overlay
        self._en = True  # Enable OS key input by default
        self._queue = ReplacingQueue(maxsize=1)
        self._stop = threading.Event()
        self._inlet: Optional[StreamInlet] = None

    def setEnable(self, enable: bool):
        self._queue.put(enable)

    def run(self):
        try:
            self._inlet = StreamInlet(self.stream_info)
            self.connected.emit()
            srate = float(self.stream_info.nominal_srate())
            json_mode = not (srate > 0.0 and (srate != float("inf")))
            name = self.stream_info.name()
            self.status.emit(f"Connected to {name} | mode={'JSON' if json_mode else 'Fixed'}")
            if json_mode:
                self._read_json_stream()
            else:
                self._read_fixed_stream()
        except Exception as e:
            self.error.emit(f"Reader error: {e}")
        finally:
            if self.parser is not None:
                for msg in self.parser.release_all():
                    self._submit_and_visualize(msg)
            self.disconnected.emit()

    def stop(self):
        self._stop.set()

    def _read_json_stream(self):
        while not self._stop.is_set():
            if self._queue.full():
                self._en = self._queue.get()
            sample, ts = self._inlet.pull_sample(timeout=0.2)
            if sample is None:
                continue
            payload = sample[0] if len(sample) == 1 else " ".join(str(s) for s in sample)
            # Try to parse—if it’s valid JSON with key events, mirror visually.
            obj = json.loads(payload)
            self._handle_message_object(obj)

    def _read_fixed_stream(self):
        while not self._stop.is_set():
            samples, _ = self._inlet.pull_chunk(timeout=0.2)
            if not samples:
                continue
            for s in samples:
                events = self.parser.process(s) if self.parser else []
                for ev in events:
                    self._submit_and_visualize(ev)

    # ----- helpers -----
    def _handle_message_object(self, msg: Dict[str, Any]):
        mtype = msg.get("type")
        if mtype == "key":
            self._submit_and_visualize(msg)
        elif mtype == "hotkey":
            # Visual flash of the chord; worker handles true press/release timing
            keys = [str(k) for k in msg.get("keys", [])]
            for k in keys:
                self.overlay.tap(k, duration_ms=90)
            if self._en:
                self.worker.submit(msg)  # still forward
        elif mtype == "sequence":
            for step in msg.get("steps", []):
                self._handle_message_object(step)
            if self._en:
                self.worker.submit(msg)
        else:
            if self._en:
                self.worker.submit(msg)

    def _submit_and_visualize(self, ev: Dict[str, Any]):
        if ev.get("type") == "key":
            k = str(ev.get("key", "")).lower()
            act = str(ev.get("action", "tap")).lower()
            if k in ("enter", "return") and act in ("tap", "press"):
                self.newline.emit()
            if act == "press":
                self.overlay.press(k)
            elif act == "release":
                self.overlay.release(k)
            elif act == "tap":
                self.overlay.tap(k, duration=0.08)
        if self._en:
            self.worker.submit(ev)
    