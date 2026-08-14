import time
import numpy as np
from nml_hand_exo.processing import compute_rms, RealtimeEMGFilter


class StateTriggerRMS:
    """
    A simple controller that monitors real-time EMG RMS signals and sends gesture commands
    to an exoskeleton based on channel group thresholds.

    Attributes:
        exo: Instance of HandExo.
        client: LSLClient or compatible data stream reader.
        threshold_percent: Threshold as a percentage above rest mean RMS.
        window_ms: Duration of EMG window for RMS calculation.
        rest_duration_sec: Time to collect baseline (rest) RMS signal.
    """

    def __init__(self, exo, client, threshold_percent=0.1, window_ms=200, rest_duration_sec=5, verbose=False):
        self.exo = exo
        self.client = client
        self.sampling_rate = client.sampling_rate if hasattr(client, 'sampling_rate') else 1000
        self.threshold_percent = threshold_percent
        self.window_ms = window_ms
        self.rest_duration_sec = rest_duration_sec
        self.verbose = verbose

        self.channel_groups = {}  # {gesture_name: [channel indices]}
        self.baseline_rms = {}  # Baseline RMS values for each channel
        self.last_state = None
        self.n_channels = client.n_channels if hasattr(client, 'n_channels') else len(client.get_channel_names())

        # Initialize filter
        self.emg_filter = RealtimeEMGFilter(
            sampling_rate=self.sampling_rate,
            n_channels=self.n_channels,
            enable_car=True,  # Common Average Reference
            enable_bandpass=True,  # Bandpass filter
            enable_notch=True,  # Notch filter
        )

    def set_trigger_state(self, gesture, channels):
        """Assigns a list of channels that will activate a specific gesture."""
        self.channel_groups[gesture] = channels
        if self.verbose:
            print(f"[Trigger] Set gesture '{gesture}' for channels {channels}")

    def calibrate_rest_state(self, save_path=None):
        """Collects baseline RMS for each gesture group and computes thresholds."""
        print("[Calibration] Collecting rest-state RMS for threshold estimation...")
        all_rms = []

        start_time = time.monotonic()
        while time.monotonic() - start_time < self.rest_duration_sec:
            window = self.client.get_latest_window(self.window_ms)
            if window is None or len(window) == 0:
                print("[Calibration] No data received. Ensure the client is streaming data.")
                time.sleep(self.window_ms / 1000.0)
                continue

            rms = compute_rms(window)
            all_rms.append(rms)
            time.sleep(self.window_ms / 1000.0)

        # Compute the average RMS over time for each channel, set as baseline
        all_ch_rms = np.stack(all_rms, axis=0)  # shape: (n_windows, n_channels)
        for ch in range(all_ch_rms.shape[1]):
            self.baseline_rms[ch] = np.mean(all_ch_rms[:, ch])
            if self.verbose:
                print(f"[Calibration] Channel {ch}: Baseline RMS = {self.baseline_rms[ch]:.2f}")
        if self.verbose:
            print("[Calibration] Collected RMS data for rest state.")

    def load_baseline(self, path):
        """Loads a saved baseline RMS dictionary from file."""
        self.baseline_rms = np.load(path, allow_pickle=True).item()

    def save_baseline(self, path):
        """Saves current baseline RMS values to file."""
        if not self.baseline_rms:
            raise RuntimeError("No rest means available to save. Please calibrate first.")
        if not path.endswith('.npy'):
            raise ValueError("Path must end with '.npy' extension.")

        # Save the rest_means dictionary to a .npy file
        np.save(path, self.baseline_rms)
        print(f"[Baseline] Saved to {path}")

    def start(self):
        """Begins monitoring the EMG stream and sends gesture commands when thresholds are crossed."""
        if not self.baseline_rms:
            raise RuntimeError("No baseline RMS values available. Please calibrate or load baseline first.")
        if not self.channel_groups:
            raise RuntimeError("No gesture channels set. Use set_trigger_state() to define at least one gesture.")

        print("[GestureTrigger] Starting gesture monitoring loop.")
        thresholds = {}
        for gesture, channels in self.channel_groups.items():
            rms_values = [self.baseline_rms[ch] for ch in channels]
            thresholds[gesture] = np.mean(rms_values) * (1 + self.threshold_percent)
            if self.verbose:
                print(f"[Trigger] Gesture '{gesture}' threshold set at {thresholds[gesture]:.2f} RMS")

        print(f"Thresholds for gestures: {thresholds}")

        if not self.client.streaming:
            self.client.start_streaming()

        cooldown_ms = 500  # 500ms cooldown
        last_trigger_time = 0
        try:
            while True:
                time.sleep(self.window_ms / 1000.0)
                now = time.monotonic() * 1000  # current time in ms

                window = self.client.get_latest_window(self.window_ms)
                if window is None or window.shape[1] == 0:
                    continue

                window = self.emg_filter.update(window)
                rms = compute_rms(window)

                detected_gesture = None
                max_rms_over_threshold = -np.inf
                for gesture, channels in self.channel_groups.items():
                    avg_rms = np.mean([rms[ch] for ch in channels])
                    if avg_rms > thresholds[gesture] and avg_rms > max_rms_over_threshold:
                        detected_gesture = gesture
                        max_rms_over_threshold = avg_rms

                if detected_gesture is not None and self.last_state != detected_gesture:
                    if now - last_trigger_time >= cooldown_ms:
                        print(f"[Trigger] {detected_gesture.capitalize()} RMS triggered ({avg_rms:.2f} > {thresholds[gesture]:.2f})")
                        self.exo.set_gesture_state(detected_gesture)
                        self.last_state = detected_gesture
                        last_trigger_time = now

                elif detected_gesture is None and self.last_state != "rest":
                    if now - last_trigger_time >= cooldown_ms:
                        # If no gesture detected, return to rest state
                        if self.verbose:
                            print("[Trigger] Returning to REST state.")
                        self.last_state = "rest"
                        last_trigger_time = now

        except KeyboardInterrupt:
            print("\n[GestureTrigger] Stopped by user.")

        finally:
            self.client.stop_streaming()
            self.exo.close()


class EMGClassifierTrigger:
    """
    A controller that uses a pre-trained EMG classifier to trigger gestures based on real-time EMG data.

    Attributes:
        exo: Instance of HandExo.
        client: LSLClient or compatible data stream reader.
        model: Pre-trained classifier model.
        scaler: Scaler used for normalizing EMG features.
        threshold_percent: Threshold as a percentage above rest mean RMS.
        window_ms: Duration of EMG window for RMS calculation.
    """

    def __init__(self, exo, client, model, scaler, threshold_percent=0.1, window_ms=200, verbose=False):
        self.exo = exo
        self.client = client
        self.model = model
        self.scaler = scaler
        self.threshold_percent = threshold_percent
        self.window_ms = window_ms
        self.verbose = verbose

        self.last_state = None

    def start(self):
        """Begins monitoring the EMG stream and sends gesture commands based on classifier predictions."""
        if not self.model or not self.scaler:
            raise RuntimeError("Model and scaler must be provided.")

        print("[EMGClassifierTrigger] Starting gesture monitoring loop.")

        if not self.client.streaming:
            self.client.start_streaming()

        try:
            while True:
                time.sleep(self.window_ms / 1000.0)
                window = self.client.get_latest_window(self.window_ms)
                if window is None or window.shape[1] == 0:
                    continue

                # Use one RMS feature per channel. The caller supplies a fitted
                # scaler and any estimator implementing scikit-learn's
                # ``predict`` protocol; no undeclared deep-learning runtime is
                # required by this compatibility controller.
                features = compute_rms(window).reshape(1, -1)
                scaled = self.scaler.transform(features)
                predicted_gesture = str(self.model.predict(scaled)[0])

                if predicted_gesture != self.last_state:
                    print(f"[ClassifierTrigger] Detected gesture: {predicted_gesture}")
                    self.exo.set_gesture_state(predicted_gesture)
                    self.last_state = predicted_gesture

        except KeyboardInterrupt:
            print("\n[EMGClassifierTrigger] Stopped by user.")

        finally:
            self.client.stop_streaming()
            self.exo.close()
            print("[EMGClassifierTrigger] Exoskeleton connection closed.")
