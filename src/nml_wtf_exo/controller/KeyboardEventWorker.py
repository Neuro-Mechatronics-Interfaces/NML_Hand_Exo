import json
import threading
import time
from queue import Queue, Empty
from typing import Any, Dict, List, Union, Optional
from pynput.keyboard import Controller, Key, KeyCode


class KeyboardEventWorker:
    """
    Background worker that consumes JSON messages from a queue
    and emulates keyboard events.

    Supported message formats (JSON strings or dicts):
      1) Tap/press/release a single key:
         {"type": "key", "key": "a", "action": "tap"}         # or "press" / "release"
         {"type": "key", "key": "space", "action": "tap"}
         {"type": "key", "key": "enter", "action": "tap"}

      2) Type text:
         {"type": "text", "text": "Hello, world!"}

      3) Chord/hotkey (press all, then release all):
         {"type": "hotkey", "keys": ["ctrl", "alt", "f"]}

      4) Delay (sleep inside worker):
         {"type": "delay", "seconds": 0.25}

      5) Sequence (batch multiple messages atomically):
         {"type": "sequence", "steps": [ <any of the above> ]}

    Notes:
      - Special keys: "ctrl", "alt", "shift", "cmd"/"win", "enter",
        "space", "tab", "esc", "backspace", "delete", "home", "end",
        "pageup", "pagedown", "left", "right", "up", "down", "f1".."f24".
      - For normal characters, just use the key itself (e.g., "a", "A", "1", ";").
      - All actions are executed on the worker thread.
    """

    def __init__(self, poll_interval: float = 0.05):
        self.queue: "Queue[Union[str, Dict[str, Any]]]" = Queue()
        self._poll_interval = poll_interval
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._kb = Controller()

    # -------------------- Public API --------------------

    def start(self, daemon: bool = True) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="KeyboardEventWorker", daemon=daemon)
        self._thread.start()

    def stop(self, wait: bool = True) -> None:
        self._stop.set()
        if wait and self._thread:
            self._thread.join(timeout=5)

    def submit(self, msg: Union[str, Dict[str, Any]]) -> None:
        """
        Enqueue a message (JSON string or Python dict).
        """
        self.queue.put(msg)

    # Convenience helpers:
    def type_text(self, text: str) -> None:
        self.submit({"type": "text", "text": text})

    def tap(self, key: str) -> None:
        self.submit({"type": "key", "key": key, "action": "tap"})

    def hotkey(self, keys: List[str]) -> None:
        self.submit({"type": "hotkey", "keys": keys})

    # -------------------- Worker internals --------------------

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self.queue.get(timeout=self._poll_interval)
            except Empty:
                continue

            try:
                self._handle_message(msg)
            except Exception as e:
                # Fail-safe: don't kill the worker on malformed messages.
                print(f"[KeyboardEventWorker] Error handling message: {e}")

            finally:
                self.queue.task_done()

    def _handle_message(self, msg: Union[str, Dict[str, Any]]) -> None:
        if isinstance(msg, str):
            msg = json.loads(msg)

        mtype = msg.get("type")
        if mtype == "text":
            self._do_text(msg)
        elif mtype == "key":
            self._do_key(msg)
        elif mtype == "hotkey":
            self._do_hotkey(msg)
        elif mtype == "delay":
            self._do_delay(msg)
        elif mtype == "sequence":
            self._do_sequence(msg)
        else:
            raise ValueError(f"Unknown message type: {mtype}")

    # -------------------- Actions --------------------

    def _do_text(self, msg: Dict[str, Any]) -> None:
        text = msg.get("text", "")
        if not isinstance(text, str):
            raise ValueError("text must be a string")
        self._kb.type(text)

    def _do_key(self, msg: Dict[str, Any]) -> None:
        key_name = msg.get("key")
        action = (msg.get("action") or "tap").lower()
        key = self._parse_key(key_name)

        if action == "tap":
            self._kb.press(key)
            self._kb.release(key)
        elif action == "press":
            self._kb.press(key)
        elif action == "release":
            self._kb.release(key)
        else:
            raise ValueError(f"Unknown key action: {action}")

    def _do_hotkey(self, msg: Dict[str, Any]) -> None:
        keys = msg.get("keys")
        if not isinstance(keys, list) or not keys:
            raise ValueError("hotkey 'keys' must be a non-empty list")

        parsed = [self._parse_key(k) for k in keys]
        # Press all in order, release in reverse (common hotkey behavior)
        for k in parsed:
            self._kb.press(k)
        # Small pause helps some targets register the chord
        time.sleep(0.02)
        for k in reversed(parsed):
            self._kb.release(k)

    def _do_delay(self, msg: Dict[str, Any]) -> None:
        seconds = float(msg.get("seconds", 0.0))
        if seconds > 0:
            time.sleep(seconds)

    def _do_sequence(self, msg: Dict[str, Any]) -> None:
        steps = msg.get("steps", [])
        if not isinstance(steps, list):
            raise ValueError("sequence 'steps' must be a list")
        for step in steps:
            self._handle_message(step)

    # -------------------- Key parsing --------------------

    def _parse_key(self, name: Union[str, int]) -> Union[Key, KeyCode]:
        """
        Map common string names to pynput Key/KeyCode.
        Accepts single characters or special names (case-insensitive).
        """
        if isinstance(name, int):
            # Optional: support by virtual keycode if desired
            return KeyCode.from_vk(name)

        if not isinstance(name, str) or not name:
            raise ValueError(f"Invalid key: {name!r}")

        n = name.strip().lower()

        special = {
            "ctrl": Key.ctrl, "control": Key.ctrl,
            "alt": Key.alt,
            "shift": Key.shift,
            "cmd": Key.cmd, "win": Key.cmd, "meta": Key.cmd,
            "enter": Key.enter, "return": Key.enter,
            "space": Key.space,
            "tab": Key.tab,
            "esc": Key.esc, "escape": Key.esc,
            "backspace": Key.backspace,
            "delete": Key.delete,
            "home": Key.home,
            "end": Key.end,
            "pageup": Key.page_up,
            "pagedown": Key.page_down,
            "left": Key.left, "right": Key.right, "up": Key.up, "down": Key.down,
        }
        if n in special:
            return special[n]

        # Function keys f1..f24
        if n.startswith("f") and n[1:].isdigit():
            idx = int(n[1:])
            fkeys = {
                1: Key.f1, 2: Key.f2, 3: Key.f3, 4: Key.f4, 5: Key.f5, 6: Key.f6,
                7: Key.f7, 8: Key.f8, 9: Key.f9, 10: Key.f10, 11: Key.f11, 12: Key.f12,
                13: Key.f13, 14: Key.f14, 15: Key.f15, 16: Key.f16, 17: Key.f17, 18: Key.f18,
                19: Key.f19, 20: Key.f20, 21: Key.f21, 22: Key.f22, 23: Key.f23, 24: Key.f24,
            }
            if idx in fkeys:
                return fkeys[idx]

        # Single character -> KeyCode
        if len(n) == 1:
            return KeyCode.from_char(n)

        # If it's more than one character and not special, try first char
        # (Alternatively, raise for strictness.)
        return KeyCode.from_char(n[0])


# -------------------- Example usage --------------------
if __name__ == "__main__":
    worker = KeyboardEventWorker(poll_interval=0.02)
    worker.start()

    # Type some text
    worker.submit({"type": "text", "text": "Hello from the worker!\n"})

    # Tap ENTER
    worker.submit('{"type":"key","key":"enter","action":"tap"}')

    # Hotkey: Ctrl+Shift+N
    worker.submit({"type": "hotkey", "keys": ["ctrl", "shift", "n"]})

    # A small scripted sequence
    worker.submit({
        "type": "sequence",
        "steps": [
            {"type": "text", "text": "Opening dev console in 3..."},
            {"type": "delay", "seconds": 0.5},
            {"type": "text", "text": " 2..."},
            {"type": "delay", "seconds": 0.5},
            {"type": "text", "text": " 1...\n"},
            {"type": "hotkey", "keys": ["ctrl", "shift", "i"]},
        ]
    })

    # Let the worker drain for a moment
    time.sleep(2)
    worker.stop()

