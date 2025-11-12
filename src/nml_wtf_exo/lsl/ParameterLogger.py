import json
import threading
import time
import os
from pylsl import StreamInlet
from nml_wtf_exo.lsl.utils import resolve_stream
import pandas as pd
from datetime import datetime


class ParameterLogger:
    def __init__(self, log_dir="logs", suffix="logs"):
        # Timestamped filename prefix
        now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.base_filename = f"logger_{now_str}_{suffix}"

        # Create output directory
        os.makedirs(log_dir, exist_ok=True)
        self.log_dir = log_dir

        # Resolve stream
        print("Looking for MatlabTMSiState stream...")
        streams = resolve_stream('name', 'MatlabTMSiState')
        self.inlet = StreamInlet(streams[0])
        print("Connected to MatlabTMSiState")

        # Buffers
        self.log_all = []
        self.logs_by_type = {'state': [], 'filename': [], 'parameter': []}
        self.trial_log = []

        # Trial tracking
        self.in_trial = False
        self.current_filename = None
        self.current_trial = []

        # Threading
        self.running = False
        self.thread = threading.Thread(target=self.listen_loop, daemon=True)

    def start(self):
        self.running = True
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()
        self.flush_all_logs()

    def listen_loop(self):
        while self.running:
            sample, timestamp = self.inlet.pull_sample(timeout=0.1)
            if sample:
                try:
                    message = json.loads(sample[0])
                    self.handle_message(message, timestamp)
                except json.JSONDecodeError:
                    print(f"[JSON ERROR] Bad message: {sample[0]}")
                except Exception as e:
                    print(f"[ERROR] {e}")

    def handle_message(self, msg, lsl_ts):
        name = msg.get('name')
        value = msg.get('value')
        loop_ts = msg.get('loop_ts')

        # Build full entry
        entry = {
            'LSL_Timestamp': lsl_ts,
            'Loop_Timestamp': loop_ts,
            'Name': name,
            'Value': value
        }

        # Log to unified table
        self.log_all.append(entry)

        # Log by type
        if name in self.logs_by_type:
            self.logs_by_type[name].append(entry)
            self.flush_log_type(name)

        # Trial tracking
        if name == 'filename':
            self.current_filename = value

        elif name == 'state':
            if value == 'rec' and not self.in_trial:
                self.current_trial = [{
                    'Timestamp': lsl_ts,
                    'File': self.current_filename,
                    'Event': 'Recording Start'
                }]
                self.in_trial = True
            elif self.in_trial and value != 'rec':
                self.current_trial.append({
                    'Timestamp': lsl_ts,
                    'File': self.current_filename,
                    'Event': 'Recording End'
                })
                self.trial_log.extend(self.current_trial)
                self.flush_trial_log()
                self.current_trial = []
                self.in_trial = False

        elif name == 'parameter' and self.in_trial:
            self.current_trial.append({
                'Timestamp': lsl_ts,
                'File': self.current_filename,
                'Event': json.dumps(msg)
            })

    def flush_log_type(self, log_type):
        if log_type not in self.logs_by_type:
            return
        records = self.logs_by_type[log_type]
        if not records:
            return

        df = pd.DataFrame(records)
        csv_path = os.path.join(self.log_dir, f"{self.base_filename}_{log_type}.csv")
        df.to_csv(csv_path, mode='a', header=not os.path.exists(csv_path), index=False)
        self.logs_by_type[log_type] = []  # clear buffer

    def flush_trial_log(self):
        if not self.trial_log:
            return
        df = pd.DataFrame(self.trial_log)
        csv_path = os.path.join(self.log_dir, f"{self.base_filename}_trials.csv")
        df.to_csv(csv_path, mode='a', header=not os.path.exists(csv_path), index=False)
        self.trial_log = []

    def flush_all_logs(self):
        for log_type in self.logs_by_type:
            self.flush_log_type(log_type)
        self.flush_trial_log()

    def get_full_log(self):
        return pd.DataFrame(self.log_all)

    def get_trial_log(self):
        return pd.read_csv(os.path.join(self.log_dir, f"{self.base_filename}_trials.csv"))
