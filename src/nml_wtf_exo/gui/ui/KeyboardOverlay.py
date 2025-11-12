# nml/gui/ui/KeyboardOverlay.py
from __future__ import annotations
import json
import queue
import time
from typing import Dict, Set, Tuple
from PyQt5.QtCore import Qt, QRect, QTimer, QSize
from PyQt5.QtGui import QPixmap, QPainter, QColor
from PyQt5.QtWidgets import QWidget
from nml_wtf_exo.utils.paths import PATHS

class KeyboardOverlay(QWidget):
    """
    Displays a background keyboard image and paints semi-transparent
    rectangles on any keys currently 'pressed' (held) or 'tapped' (timed).
    """

    def __init__(self, layout_path: str, tick_ms: int = 30, parent=None):
        super().__init__(parent)
        # Geometry + assets
        self._layout: Dict[str, QRect] = {}
        self._pixmap = QPixmap()
        self._base_w = 1
        self._base_h = 1

        # Visual state
        self.overlay_color = QColor(40, 120, 255, 110)  # semi-transparent blue
        self._held: Set[str] = set()       # keys currently pressed
        self._active: Dict[str, float] = {}  # key -> expiry time for taps

        # Thread-safe event queue (consumed by GUI timer)
        self._queue: "queue.Queue[dict]" = queue.Queue()

        self.load_layout(layout_path)

        # Single periodic timer in GUI thread
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._process_queue)
        self._timer.start(tick_ms)

    # ---------- public API (thread-safe) ----------
    def tap(self, key: str, duration: float = 0.08):
        """Queue a tap (press+release after duration seconds). Safe from any thread."""
        self._queue.put({
            "action": "tap",
            "key": str(key).lower(),
            "timestamp": time.time(),
            "duration": float(duration),
        })

    def press(self, key: str):
        """Queue a press (held until an explicit release). Safe from any thread."""
        self._queue.put({"action": "press", "key": str(key).lower()})

    def release(self, key: str):
        """Queue a release. Safe from any thread."""
        self._queue.put({"action": "release", "key": str(key).lower()})

    def release_all(self):
        """Clear all highlights (call from GUI thread on disconnect/close)."""
        self._held.clear()
        self._active.clear()
        self.update()

    # ---------- layout / loading ----------
    def load_layout(self, layout_path: str):
        with open(layout_path, "r", encoding="utf-8") as f:
            layout = json.load(f)

        img_path = layout.get("image_path") or PATHS["virtual_keyboard_png"]
        self._pixmap = QPixmap(img_path)
        self._base_w = self._pixmap.width() or 1
        self._base_h = self._pixmap.height() or 1

        # Build rect map
        self._layout.clear()
        for k, rect in (layout.get("keys") or {}).items():
            if isinstance(rect, list) and len(rect) == 4:
                x, y, w, h = map(int, rect)
                self._layout[k.lower()] = QRect(x, y, w, h)

        # Match widget to image pixel dimensions exactly
        if not self._pixmap.isNull():
            self.setFixedSize(self._pixmap.size())

        self.update()

    def sizeHint(self) -> QSize:
        return self._pixmap.size()

    def key_rect(self, key: str):
        return self._layout.get(key.lower())

    # ---------- timer/queue processing (GUI thread) ----------
    def _process_queue(self):
        """Drain queued events and update visual state."""
        now = time.time()
        while True:
            try:
                ev = self._queue.get_nowait()
            except queue.Empty:
                break

            action = ev.get("action")
            k = str(ev.get("key", "")).lower()
            if not k:
                continue

            if action == "tap":
                ts = float(ev.get("timestamp", now))
                dur = float(ev.get("duration", 0.08))
                self._active[k] = ts + max(0.0, dur)
            elif action == "press":
                self._held.add(k)
                # if it was in taps, clear it (held dominates)
                self._active.pop(k, None)
            elif action == "release":
                self._held.discard(k)
                self._active.pop(k, None)

        # Clear expired taps
        expired = [k for k, texp in self._active.items() if now >= texp]
        for k in expired:
            self._active.pop(k, None)

        self.update()

    # ---------- painting ----------
    def paintEvent(self, event):
        p = QPainter(self)
        if not self._pixmap.isNull():
            p.drawPixmap(0, 0, self._pixmap)

        p.setPen(Qt.NoPen)
        p.setBrush(self.overlay_color)

        # Draw held keys
        for k in self._held:
            rect = self.key_rect(k)
            if rect:
                p.drawRect(rect)

        # Draw unexpired taps
        now = time.time()
        for k, expiry in self._active.items():
            if now < expiry:
                rect = self.key_rect(k)
                if rect:
                    p.drawRect(rect)
