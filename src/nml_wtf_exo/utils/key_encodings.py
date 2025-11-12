# nml/utils/key_encodings.py
from __future__ import annotations
from typing import Dict, List

# Map characters that require SHIFT on a US keyboard to their base key.
_SHIFTED: Dict[str, str] = {
    "!": "1", "@": "2", "#": "3", "$": "4", "%": "5", "^": "6", "&": "7", "*": "8", "(": "9", ")": "0",
    "_": "-", "+": "=", "{": "[", "}": "]", "|": "\\",
    ":": ";", '"': "'", "<": ",", ">": ".", "?": "/",
}

_SIMPLE_TAPS = set(
    "abcdefghijklmnopqrstuvwxyz0123456789"
    "`-=[]\\;',./"  # unshifted punctuation on US layout
)

def char_to_events(ch: str) -> List[dict]:
    """
    Convert a single character to a list of keyboard JSON events
    understood by KeyboardEventWorker & your overlay.
    Uses US keyboard layout heuristics.
    """
    events: List[dict] = []

    if ch == " ":
        return [{"type": "key", "key": "space", "action": "tap"}]
    if ch == "\n":
        return [{"type": "key", "key": "enter", "action": "tap"}]
    if ch == "\t":
        return [{"type": "key", "key": "tab", "action": "tap"}]

    # Uppercase letters -> Shift+letter
    if "A" <= ch <= "Z":
        base = ch.lower()
        return [
            {"type": "key", "key": "shift", "action": "press"},
            {"type": "key", "key": base,   "action": "tap"},
            {"type": "key", "key": "shift","action": "release"},
        ]

    # Shifted punctuation (US)
    if ch in _SHIFTED:
        base = _SHIFTED[ch]
        return [
            {"type": "key", "key": "shift", "action": "press"},
            {"type": "key", "key": base,    "action": "tap"},
            {"type": "key", "key": "shift", "action": "release"},
        ]

    # Simple 1:1 taps
    if ch in _SIMPLE_TAPS:
        return [{"type": "key", "key": ch, "action": "tap"}]

    # Fallback: send as text (won't show on overlay, but avoids crashing)
    return [{"type": "text", "text": ch}]
