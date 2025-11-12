# ~/nml/utils/LogReader.py
# CSV reader for logs with columns: Time (float seconds), Sample (stringified list of floats).
import ast
import os
from typing import List, Tuple, Optional
import json
import pandas as pd
import numpy as np


class LogReader:
    def __init__(self, path: str):
        self.path = os.path.expanduser(path)
        if not os.path.exists(self.path):
            raise FileNotFoundError(self.path)
        self.meta = {}
        base, _ = os.path.splitext(self.path)
        meta_path = base + ".meta.json"
        if os.path.exists(meta_path):
            with open(meta_path, "r", encoding="utf-8") as f:
                self.meta = json.load(f)
        self.channel_labels = [c.get("label", "") for c in self.meta.get("channels", [])]
        self.y_origin = self.meta.get("y_axis_origin", "top_left_image")

        self._df = pd.read_csv(self.path)
        if "Time" not in self._df.columns or "Sample" not in self._df.columns:
            raise ValueError("CSV must have 'Time' and 'Sample' columns")
        self.times = self._df["Time"].to_numpy(dtype=float)
        first = self._parse_sample(self._df["Sample"].iloc[0])
        self.sample_len = len(first)
        if self.sample_len % 3 == 0:
            self.dims = 3
        elif self.sample_len % 2 == 0:
            self.dims = 2
        else:
            raise ValueError(f"Cannot infer dims per landmark from sample length={self.sample_len}")
        self.n_landmarks = self.sample_len // self.dims
        self.samples = np.empty((len(self._df), self.sample_len), dtype=np.float32)
        self.samples[0] = np.asarray(first, dtype=np.float32)
        for i in range(1, len(self._df)):
            self.samples[i] = np.asarray(self._parse_sample(self._df["Sample"].iloc[i]), dtype=np.float32)
        self.times = self.times - self.times[0]
        self._i = 0

    def _parse_sample(self, s: str):
        s = str(s)
        try:
            arr = json.loads(s)
        except Exception:
            arr = ast.literal_eval(s)
        if not isinstance(arr, (list, tuple)):
            raise ValueError("Sample cell must contain a list/tuple of floats")
        return list(arr)
    
    def recommended_flip_y(self) -> bool:
        return str(self.y_origin).lower().startswith("top_left")

    def labels(self):
        return list(self.channel_labels)

    def reset(self) -> None:
        self._i = 0

    def is_done(self) -> bool:
        return self._i >= len(self.times)

    def step(self) -> Optional[Tuple[float, np.ndarray, np.ndarray]]:
        if self.is_done():
            return None
        t = float(self.times[self._i])
        s = self.samples[self._i]
        if self.dims == 3:
            xs = s[0::3]
            ys = s[1::3]
        else:
            xs = s[0::2]
            ys = s[1::2]
        self._i += 1
        return t, xs, ys

    def peek_delta_to_next_ms(self, speed: float = 1.0) -> int:
        i = self._i
        if i >= len(self.times) - 1:
            return 0
        dt = float(self.times[i + 1] - self.times[i])
        dt = max(0.0, dt)
        if speed <= 0:
            speed = 1.0
        return int(dt * 1000.0 / speed)

    def frame_count(self) -> int:
        return len(self.times)

    def landmark_count(self) -> int:
        return self.n_landmarks

    def dims_per_landmark(self) -> int:
        return self.dims
