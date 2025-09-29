# intan/interface/lsl_subscribers.py
import time
from __future__ import annotations
import threading
from collections import deque
from typing import Optional, List, Tuple, Callable, Any
import numpy as np
from pylsl import StreamInlet, resolve_byprop, cf_string


class LSLMarkerSubscriber:
    """
    Subscribe to a string-valued marker stream (e.g., type="Markers").

    Usage
    -----
        def on_marker(text: str, ts: float, exo):
            exo.set_gesture(text)

        with LSLMarkerSubscriber(stream_type="Markers", verbose=True) as sub:
            sub.set_callback(on_marker, exo, only_on_change=True)
            # ... do other work; callbacks run in the background thread
    """

    def __init__(
        self,
        stream_name: Optional[str] = None,
        stream_type: Optional[str] = "Markers",
        timeout: float = 5.0,
        buf_events: int = 2000,   # keep last N events
        recover: bool = True,
        verbose: bool = False,
    ):
        if not stream_name and not stream_type:
            raise ValueError("Provide stream_name or stream_type.")
        self.stream_name = stream_name
        self.stream_type = stream_type
        self.timeout = float(timeout)
        self.buf_events = int(buf_events)
        self.recover = bool(recover)
        self.verbose = verbose

        # resolve & connect
        qkey = "name" if stream_name else "type"
        qval = stream_name or stream_type
        streams = resolve_byprop(qkey, qval, timeout=self.timeout)
        if not streams:
            raise RuntimeError(f"No LSL stream with {qkey}='{qval}' found within {self.timeout}s.")

        self.inlet = StreamInlet(streams[0], recover=self.recover)
        info = self.inlet.info()
        self.name = info.name()
        self.type = info.type()
        self.n_channels = info.channel_count()
        self.fs = float(info.nominal_srate())  # usually 0 for markers

        if self.verbose:
            print(f"[LSLMarkerSubscriber] connected to '{self.name}' (type={self.type})")

        # state
        self._lock = threading.Lock()
        self._stop = False
        self._thread: Optional[threading.Thread] = None
        self._markers: deque[Tuple[float, str]] = deque(maxlen=self.buf_events)

        # callback state
        self._cb: Optional[Callable[..., Any]] = None
        self._cb_args: tuple[Any, ...] = ()
        self._cb_kwargs: dict[str, Any] = {}
        self._only_on_change: bool = False
        self._last_text: Optional[str] = None

    # --- context manager ---
    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()

    # --- lifecycle ---
    def start(self):
        if self._thread and self._thread.is_alive():
            return self
        self._stop = False
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop = True
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            self.inlet.close_stream()
        except Exception:
            pass

    # --- callback API ---
    def set_callback(self, func: Optional[Callable[..., Any]], *args, only_on_change: bool = False, **kwargs) -> None:
        """
        Register a callback invoked as: func(text: str, ts: float, *args, **kwargs)

        Examples:
            sub.set_callback(on_marker)  # func(text, ts)
            sub.set_callback(on_marker, exo, only_on_change=True)  # func(text, ts, exo=...)
            sub.set_callback(on_marker, exo=exo)  # func(text, ts, exo=exo)

        Pass None to clear the callback.
        """
        with self._lock:
            self._cb = func
            self._cb_args = args
            self._cb_kwargs = kwargs
            self._only_on_change = bool(only_on_change)
            # Do NOT reset _last_text; change detection should persist

    # --- worker ---
    def _worker(self):
        while not self._stop:
            sample, ts = self.inlet.pull_sample(timeout=0.1)
            if sample is None:
                continue

            # Concatenate multi-field markers (rare) into one string
            text = sample[0] if len(sample) == 1 else "|".join(map(str, sample))
            text = str(text)

            # store & compute if we should fire
            with self._lock:
                self._markers.append((ts, text))
                cb = self._cb
                args = self._cb_args
                kwargs = self._cb_kwargs
                only_change = self._only_on_change
                last = self._last_text

                should_fire = True
                if only_change and (text == last):
                    should_fire = False
                self._last_text = text

            # invoke callback outside the lock
            if cb and should_fire:
                try:
                    cb(text, float(ts), *args, **kwargs)
                except Exception as e:
                    if self.verbose:
                        print(f"[LSLMarkerSubscriber] callback error: {e}")

    # --- polling helpers ---
    def get_recent_markers(self, k: int = 1) -> List[Tuple[float, str]]:
        """Return the last k markers as (timestamp, text)."""
        with self._lock:
            return list(self._markers)[-k:]

    def get_markers_since(self, since_ts: float) -> List[Tuple[float, str]]:
        with self._lock:
            return [(ts, s) for (ts, s) in self._markers if ts >= since_ts]

    def poll_latest(self) -> Optional[Tuple[float, str]]:
        """Non-blocking: return the most recent (ts, text) or None."""
        with self._lock:
            if not self._markers:
                return None
            return self._markers[-1]

    def pull(self, timeout: float = 0.0) -> Optional[Tuple[float, str]]:
        """
        Wait up to `timeout` seconds for a new marker (compares against the last one).
        Returns (ts, text) or None if nothing new appears.
        """
        deadline = time.monotonic() + max(0.0, float(timeout))
        before = self.poll_latest()
        while time.monotonic() < deadline:
            time.sleep(0.01)
            after = self.poll_latest()
            if after and after != before:
                return after
        return None

    # --- metadata ---
    def metadata(self) -> dict:
        return {
            "name": self.name,
            "type": self.type,
            "fs": self.fs,
            "n_channels": self.n_channels,
        }

# ---------- NUMERIC (continuous) STREAMS ----------

class LSLNumericSubscriber:
    """
    Subscribe to a continuous numeric stream (e.g., EMG/EEG).
    Typical usage:
        with LSLNumericSubscriber(stream_name="MyEMG").start() as sub:
            X = sub.get_latest_window(200)  # last 200 ms -> (C, N) float32
    """
    def __init__(
        self,
        stream_name: Optional[str] = None,
        stream_type: Optional[str] = None,
        timeout: float = 5.0,
        buf_seconds: float = 10.0,
        max_chunklen: int = 1024,
        recover: bool = True,
        verbose: bool = False,
    ):
        if not stream_name and not stream_type:
            raise ValueError("Provide stream_name or stream_type.")
        self.stream_name = stream_name
        self.stream_type = stream_type
        self.timeout = float(timeout)
        self.buf_seconds = float(buf_seconds)
        self.max_chunklen = int(max_chunklen)
        self.recover = bool(recover)
        self.verbose = verbose

        # resolve & connect
        qkey = "name" if stream_name else "type"
        qval = stream_name or stream_type
        streams = resolve_byprop(qkey, qval, timeout=self.timeout)
        if not streams:
            raise RuntimeError(f"No LSL stream with {qkey}='{qval}' found within {self.timeout}s.")
        self.inlet = StreamInlet(streams[0], recover=self.recover)
        info = self.inlet.info()
        self.name = info.name()
        self.type = info.type()
        self.n_channels = info.channel_count()
        self.fs = float(info.nominal_srate())
        if self.fs <= 0:
            raise RuntimeError(f"Stream '{self.name}' nominal_srate={self.fs}; expected continuous numeric.")

        # channel labels (best effort)
        try:
            ch = info.desc().child("channels").child("channel")
            labels = []
            for i in range(self.n_channels):
                labels.append(ch.child_value("label") or f"Ch{i}")
                ch = ch.next_sibling()
            self.channel_labels = labels
        except Exception:
            self.channel_labels = [f"Ch{i}" for i in range(self.n_channels)]

        if self.verbose:
            print(f"[LSLNumericSubscriber] connected to '{self.name}' (type={self.type}) "
                  f"fs={self.fs}, C={self.n_channels}")

        # state
        self._lock = threading.Lock()
        self._stop = False
        self._thread: Optional[threading.Thread] = None
        maxlen = int(max(1, self.fs * self.buf_seconds))
        self._bufs = [deque(maxlen=maxlen) for _ in range(self.n_channels)]

    # context manager
    def __enter__(self):
        self.start()
        return self
    def __exit__(self, *exc):
        self.stop()

    def start(self):
        if self._thread and self._thread.is_alive():
            return self
        self._stop = False
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop = True
        if self._thread:
            self._thread.join(timeout=1.0)
        try:
            self.inlet.close_stream()
        except Exception:
            pass

    def _worker(self):
        # use pull_chunk() to reduce Python overhead
        while not self._stop:
            samples, _timestamps = self.inlet.pull_chunk(timeout=0.1, max_samples=self.max_chunklen)
            if not samples:
                continue
            arr = np.asarray(samples, dtype=np.float32)  # (n, C)
            with self._lock:
                for c in range(min(arr.shape[1], self.n_channels)):
                    self._bufs[c].extend(arr[:, c])

    # API
    def get_latest_window(self, window_ms: int) -> np.ndarray:
        """Return last window_ms of data as (C, N) float32 (zero-padded if not enough)."""
        n = int(round(self.fs * window_ms / 1000.0))
        if n <= 0:
            return np.zeros((self.n_channels, 0), dtype=np.float32)
        out = np.zeros((self.n_channels, n), dtype=np.float32)
        with self._lock:
            for c in range(self.n_channels):
                buf = self._bufs[c]
                if len(buf) >= n:
                    out[c] = np.fromiter(list(buf)[-n:], dtype=np.float32, count=n)
                else:
                    pad = n - len(buf)
                    if pad > 0:
                        out[c, :pad] = 0.0
                        out[c, pad:] = np.fromiter(list(buf), dtype=np.float32)
        return out

    def metadata(self) -> dict:
        return {
            "name": self.name,
            "type": self.type,
            "fs": self.fs,
            "n_channels": self.n_channels,
            "channel_labels": self.channel_labels,
        }
