import json
import threading
import time
from queue import Queue, Empty
from typing import Any, Dict, List, Union, Optional
class FixedRateKeyParser:
    """
    Example parser that maps continuous inputs to discrete key press/release
    events with hysteresis. Customize this for your application.

    Config schema idea (JSON):
    {
      "axes": [
        {"index": 0, "neg_key": "a", "pos_key": "d", "lo": 0.2, "hi": 0.4},
        {"index": 1, "neg_key": "w", "pos_key": "s", "lo": 0.2, "hi": 0.4}
      ],
      "buttons": [
        {"index": 2, "key": "space", "threshold": 0.5}
      ]
    }
    """
    def __init__(self):
        self.config: Dict[str, Any] = {"axes": [], "buttons": []}
        # Track which keys are currently held so we only emit press/release once.
        self._held_keys: set[str] = set()

    def configure(self, config_path: Optional[str]) -> None:
        if not config_path:
            return
        with open(config_path, "r", encoding="utf-8") as f:
            self.config = json.load(f)
        if "axes" not in self.config:
            self.config["axes"] = []
        if "buttons" not in self.config:
            self.config["buttons"] = []

    def process(self, sample: List[float]) -> List[Dict[str, Any]]:
        """
        Convert one fixed-rate numeric sample into a list of keyboard messages
        (dicts compatible with KeyboardEventWorker).
        """
        out: List[Dict[str, Any]] = []

        # Axes -> 2 keys (neg / pos) with hysteresis (lo/hi)
        for ax in self.config.get("axes", []):
            idx = ax.get("index", 0)
            neg_key = ax.get("neg_key")
            pos_key = ax.get("pos_key")
            lo = float(ax.get("lo", 0.2))
            hi = float(ax.get("hi", 0.4))
            val = float(sample[idx]) if idx < len(sample) else 0.0

            # Decide target held set for this axis
            want_neg = val <= -hi
            want_pos = val >= hi
            neutral = abs(val) <= lo

            # Release when neutral; otherwise hold only the desired direction.
            if neutral:
                for k in (neg_key, pos_key):
                    if k and k in self._held_keys:
                        out.append({"type": "key", "key": k, "action": "release"})
                        self._held_keys.discard(k)
            else:
                # Handle negative
                if want_neg:
                    if neg_key and neg_key not in self._held_keys:
                        out.append({"type": "key", "key": neg_key, "action": "press"})
                        self._held_keys.add(neg_key)
                    if pos_key and pos_key in self._held_keys:
                        out.append({"type": "key", "key": pos_key, "action": "release"})
                        self._held_keys.discard(pos_key)
                # Handle positive
                if want_pos:
                    if pos_key and pos_key not in self._held_keys:
                        out.append({"type": "key", "key": pos_key, "action": "press"})
                        self._held_keys.add(pos_key)
                    if neg_key and neg_key in self._held_keys:
                        out.append({"type": "key", "key": neg_key, "action": "release"})
                        self._held_keys.discard(neg_key)

        # Buttons -> single key threshold
        for btn in self.config.get("buttons", []):
            idx = btn.get("index", 0)
            key = btn.get("key")
            thr = float(btn.get("threshold", 0.5))
            val = float(sample[idx]) if idx < len(sample) else 0.0
            if key:
                if val >= thr and key not in self._held_keys:
                    out.append({"type": "key", "key": key, "action": "press"})
                    self._held_keys.add(key)
                elif val < thr and key in self._held_keys:
                    out.append({"type": "key", "key": key, "action": "release"})
                    self._held_keys.discard(key)

        return out

    def release_all(self) -> List[Dict[str, Any]]:
        """Release any keys we think are held when disconnecting."""
        out = [{"type": "key", "key": k, "action": "release"} for k in list(self._held_keys)]
        self._held_keys.clear()
        return out
