import json
import threading
import time
import os
from pylsl import StreamInlet
from nml_wtf_exo.lsl.utils import resolve_stream
import pandas as pd
from datetime import datetime

def _extract_channel_labels(info):
    # Pull channel labels from StreamInfo XML
    desc = info.desc()
    chans = desc.child("channels")
    labels = []
    ch = chans.child("channel")
    while ch.name():
        lab = ch.child_value("label") or f"chan_{len(labels)}"
        labels.append(lab)
        ch = ch.next_sibling()
    return labels

def _extract_extra(info):
    # Any extra useful bits (units, types) if present
    desc = info.desc()
    chans = desc.child("channels")
    extras = []
    ch = chans.child("channel")
    while ch.name():
        extras.append({
            "label": ch.child_value("label"),
            "unit": ch.child_value("unit"),
            "type": ch.child_value("type"),
        })
        ch = ch.next_sibling()
    return extras

class StreamLogger:
    def __init__(self, name, log_dir="landmarks", suffix="logs"):
        now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.base_filename = f"logger_{now_str}_{suffix}"
        os.makedirs(log_dir, exist_ok=True)
        self.log_dir = log_dir

        print(f"Looking for {name} stream...")
        streams = resolve_stream('name', name)
        self.inlet = StreamInlet(streams[0])
        info = self.inlet.info()
        print(f"Connected to {name}")

        self.name = name
        self.num_entries = 0
        self.max_entries = 256
        self.logs = []

        # ---- Write sidecar metadata once ----
        labels = _extract_channel_labels(info)
        meta = {
            "stream_name": info.name(),
            "type": info.type(),
            "source_id": info.source_id(),
            "channel_count": info.channel_count(),
            "nominal_srate": info.nominal_srate(),
            "created_at": now_str,
            # Optional: full per-channel descriptors (label/unit/type)
            "channels": _extract_extra(info),
            # Hints for the viewer:
            "dims_per_landmark": 3 if len(labels) % 3 == 0 else (2 if len(labels) % 2 == 0 else None),
            "y_axis_origin": "top_left_image",    # MediaPipe convention
            # If your Unity outlet wrote indices into XML somewhere, add them here too.
        }
        meta_path = os.path.join(self.log_dir, f"{self.base_filename}_{self.name}.meta.json")
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)

        # Threading
        self.running = False
        self.thread = threading.Thread(target=self.listen_loop, daemon=True)

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()
        self.flush()

    def listen_loop(self):
        while self.running:
            sample, timestamp = self.inlet.pull_sample(timeout=0.1)
            if sample:
                try:
                    self.handle_message(sample, timestamp)
                except Exception as e:
                    print(f"[ERROR] {e}")

    def handle_message(self, sample, ts):
        entry = {'Time': ts, 'Sample': json.dumps(sample)}
        self.logs.append(entry)
        self.num_entries += 1
        if self.num_entries == self.max_entries:
            self.flush()
            self.max_entries = 0

    def flush(self):
        if not self.logs:
            return
        df = pd.DataFrame(self.logs)
        csv_path = os.path.join(self.log_dir, f"{self.base_filename}_{self.name}.csv")
        df.to_csv(csv_path, mode='a', header=not os.path.exists(csv_path), index=False)
        self.logs = []

    def get_full_log(self):
        return pd.DataFrame(self.logs)
