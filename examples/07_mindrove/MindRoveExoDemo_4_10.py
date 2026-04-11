import time
import threading
import queue
import logging

import numpy as np

from mindrove.board_shim import BoardShim, MindRoveInputParams, BoardIds
from mindrove.data_filter import DataFilter, FilterTypes, WindowOperations, DetrendOperations


# ============================================================
# SETTINGS
# ============================================================
BOARD_ID = BoardIds.MINDROVE_WIFI_BOARD

CAL_SEC = 5.0
WINDOW_SEC = 0.50       # 500 ms window for stable PSD
STEP_SEC = 0.10         # update every 100 ms

# EMG filtering
BANDPASS_LOW = 20.0
BANDPASS_HIGH = 150.0
NOTCH_FREQ = 60.0
NOTCH_BW = 4.0

# Frequency range used for muscle activity power
ACTIVITY_BAND_LOW = 35.0
ACTIVITY_BAND_HIGH = 120.0

# Smoothing
SMOOTH_ALPHA = 0.20

# Threshold tuning
THRESHOLD_BLEND = 0.35
REST_MARGIN = 1.10

# State stability
FRAMES_TO_SWITCH = 3

# IMU movement freeze
USE_IMU_FREEZE = True

# Main threshold you can tune for motion artifact rejection.
# Start around 80 to 200 if gyro is used.
# If freezing too often, increase it.
# If motion artifacts still trigger bad state changes, decrease it.
IMU_FREEZE_THRESHOLD = 120.0

# Optional smoothing for IMU movement score
IMU_SMOOTH_ALPHA = 0.25

PRINT_EVERY_SEC = 1.0


# ============================================================
# COMMAND LISTENER
# ============================================================
class CommandListener(threading.Thread):
    def __init__(self, cmd_queue):
        super().__init__(daemon=True)
        self.cmd_queue = cmd_queue

    def run(self):
        while True:
            try:
                cmd = input().strip().lower()
                self.cmd_queue.put(cmd)
                if cmd in ["quit", "q"]:
                    break
            except EOFError:
                break
            except Exception:
                pass


# ============================================================
# SIGNAL PROCESSING HELPERS
# ============================================================
def preprocess_channel(sig, fs):
    """
    Preprocess a single EMG channel.
    """
    x = np.array(sig, dtype=np.float64).copy()

    # Remove DC offset
    DataFilter.detrend(x, DetrendOperations.CONSTANT.value)

    # EMG bandpass
    DataFilter.perform_bandpass(
        x,
        fs,
        BANDPASS_LOW,
        BANDPASS_HIGH,
        4,
        FilterTypes.BUTTERWORTH.value,
        0
    )

    # 60 Hz notch
    DataFilter.perform_bandstop(
        x,
        fs,
        NOTCH_BW,
        NOTCH_FREQ,
        2,
        FilterTypes.BUTTERWORTH.value,
        0
    )

    return x


def compute_rms(x):
    return float(np.sqrt(np.mean(np.square(x)) + 1e-12))


def compute_bandpower(x, fs, f_low, f_high):
    """
    Compute Welch PSD bandpower in a target EMG frequency range.
    Uses the largest power of two <= signal length to avoid PSD errors.
    """
    data_len = len(x)

    # Largest power of 2 <= data length
    nfft = 1 << (data_len.bit_length() - 1)

    if nfft < 32:
        return 0.0

    psd = DataFilter.get_psd_welch(
        x,
        nfft,
        nfft // 2,
        fs,
        WindowOperations.BLACKMAN_HARRIS.value
    )

    return float(DataFilter.get_band_power(psd, f_low, f_high))


def extract_activity_score(data, exg_channels, fs):
    """
    Returns one scalar activity score for the full arm/hand.
    Uses both RMS and EMG-band spectral power.
    """
    rms_vals = []
    band_vals = []

    for ch in exg_channels:
        x = preprocess_channel(data[ch], fs)
        rms_vals.append(compute_rms(x))
        band_vals.append(compute_bandpower(x, fs, ACTIVITY_BAND_LOW, ACTIVITY_BAND_HIGH))

    rms_vals = np.array(rms_vals, dtype=np.float64)
    band_vals = np.array(band_vals, dtype=np.float64)

    raw_band_mean = float(np.mean(band_vals))
    raw_rms_mean = float(np.mean(rms_vals))

    # Final activity score
    # Mainly spectral activity, with some RMS support
    score = 0.75 * raw_band_mean + 0.25 * raw_rms_mean

    debug = {
        "rms_mean": raw_rms_mean,
        "band_mean": raw_band_mean,
    }

    return score, debug


def exp_smooth(prev, current, alpha):
    if prev is None:
        return current
    return alpha * current + (1.0 - alpha) * prev


# ============================================================
# IMU HELPERS
# ============================================================
def get_accel_channels_safe(board_id):
    try:
        return BoardShim.get_accel_channels(board_id)
    except Exception:
        return []


def get_gyro_channels_safe(board_id):
    try:
        return BoardShim.get_gyro_channels(board_id)
    except Exception:
        return []


def extract_movement_score(data, accel_channels, gyro_channels):
    """
    Returns a scalar movement score for freezing state updates during
    large arm motion.

    Priority:
    1. Gyro mean vector magnitude
    2. Accel derivative magnitude if gyro not available
    3. 0.0 if no IMU exists
    """
    # Use gyro if available
    if gyro_channels and len(gyro_channels) >= 3:
        gyro_signals = []
        for ch in gyro_channels[:3]:
            if ch < data.shape[0]:
                gyro_signals.append(np.array(data[ch], dtype=np.float64))

        if len(gyro_signals) == 3 and len(gyro_signals[0]) > 0:
            gyro_stack = np.vstack(gyro_signals)  # shape (3, N)
            gyro_mag = np.sqrt(np.sum(np.square(gyro_stack), axis=0))
            return float(np.mean(np.abs(gyro_mag)))

    # Fallback to accel derivative
    if accel_channels and len(accel_channels) >= 3:
        accel_signals = []
        for ch in accel_channels[:3]:
            if ch < data.shape[0]:
                accel_signals.append(np.array(data[ch], dtype=np.float64))

        if len(accel_signals) == 3 and len(accel_signals[0]) > 1:
            accel_stack = np.vstack(accel_signals)  # shape (3, N)
            diff_stack = np.diff(accel_stack, axis=1)
            diff_mag = np.sqrt(np.sum(np.square(diff_stack), axis=0))
            return float(np.mean(np.abs(diff_mag)))

    return 0.0


# ============================================================
# DATA COLLECTION HELPERS
# ============================================================
def get_latest_window(board_shim, num_points):
    data = board_shim.get_current_board_data(num_points)
    if data.shape[1] < num_points:
        return None
    return data


def collect_scores(board_shim, exg_channels, fs, duration_sec, window_points, step_sec):
    scores = []
    start = time.time()

    while time.time() - start < duration_sec:
        data = get_latest_window(board_shim, window_points)
        if data is None:
            time.sleep(step_sec)
            continue

        score, _ = extract_activity_score(data, exg_channels, fs)
        scores.append(score)
        time.sleep(step_sec)

    return np.array(scores, dtype=np.float64)


# ============================================================
# CLASSIFIER
# ============================================================
class GrabRestClassifier:
    def __init__(self):
        self.calibrated = False

        self.rest_scores = None
        self.grab_scores = None

        self.rest_max = None
        self.rest_mean = None
        self.grab_mean = None
        self.threshold = None

        self.smooth_score = None
        self.smooth_movement = None

        self.state = "REST"
        self.pending_state = "REST"
        self.pending_count = 0

    def calibrate(self, board_shim, exg_channels, fs):
        window_points = int(WINDOW_SEC * fs)

        print("\nStarting calibration...")
        print("Step 1/2: Relax completely for 5 seconds.")
        time.sleep(1.0)

        rest_scores = collect_scores(
            board_shim,
            exg_channels,
            fs,
            CAL_SEC,
            window_points,
            STEP_SEC
        )

        if len(rest_scores) < 5:
            print("Not enough rest data collected.")
            return False

        print("Rest recording complete.")
        print("Step 2/2: Squeeze/grab for 5 seconds as clearly as possible.")
        time.sleep(1.0)

        grab_scores = collect_scores(
            board_shim,
            exg_channels,
            fs,
            CAL_SEC,
            window_points,
            STEP_SEC
        )

        if len(grab_scores) < 5:
            print("Not enough grab data collected.")
            return False

        self.rest_scores = rest_scores
        self.grab_scores = grab_scores

        self.rest_max = float(np.max(rest_scores))
        self.rest_mean = float(np.mean(rest_scores))
        self.grab_mean = float(np.mean(grab_scores))

        # Threshold between rest and grab
        self.threshold = self.rest_max + THRESHOLD_BLEND * (self.grab_mean - self.rest_max)

        # Enforce slight margin above rest
        self.threshold = max(self.threshold, self.rest_max * REST_MARGIN)

        self.smooth_score = None
        self.smooth_movement = None
        self.state = "REST"
        self.pending_state = "REST"
        self.pending_count = 0
        self.calibrated = True

        print("\nCalibration complete.")
        print(f"Rest mean : {self.rest_mean:.4f}")
        print(f"Rest max  : {self.rest_max:.4f}")
        print(f"Grab mean : {self.grab_mean:.4f}")
        print(f"Threshold : {self.threshold:.4f}")
        print("\nLive state detection started.\n")

        return True

    def classify(self, emg_score, movement_score, freeze_threshold, use_imu_freeze=True):
        self.smooth_score = exp_smooth(self.smooth_score, emg_score, SMOOTH_ALPHA)
        self.smooth_movement = exp_smooth(self.smooth_movement, movement_score, IMU_SMOOTH_ALPHA)

        freeze_active = False
        if use_imu_freeze and self.smooth_movement is not None:
            freeze_active = self.smooth_movement >= freeze_threshold

        # If movement is large, hold last valid state
        if freeze_active:
            return self.state, self.smooth_score, self.smooth_movement, True

        # Otherwise classify using EMG
        if self.smooth_score >= self.threshold:
            proposed = "GRAB"
        else:
            proposed = "REST"

        # Debounce / hysteresis
        if proposed == self.state:
            self.pending_state = proposed
            self.pending_count = 0
        else:
            if proposed == self.pending_state:
                self.pending_count += 1
            else:
                self.pending_state = proposed
                self.pending_count = 1

            if self.pending_count >= FRAMES_TO_SWITCH:
                self.state = proposed
                self.pending_count = 0

        return self.state, self.smooth_score, self.smooth_movement, False


# ============================================================
# MAIN
# ============================================================
def main():
    BoardShim.enable_dev_board_logger()
    logging.basicConfig(level=logging.INFO)

    params = MindRoveInputParams()
    board_shim = None

    cmd_queue = queue.Queue()
    listener = CommandListener(cmd_queue)
    listener.start()

    classifier = GrabRestClassifier()

    try:
        print("Preparing MindRove session...")
        board_shim = BoardShim(BOARD_ID, params)
        board_shim.prepare_session()
        board_shim.start_stream()

        board_id = board_shim.get_board_id()
        exg_channels = BoardShim.get_exg_channels(board_id)
        accel_channels = get_accel_channels_safe(board_id)
        gyro_channels = get_gyro_channels_safe(board_id)
        fs = BoardShim.get_sampling_rate(board_id)

        window_points = int(WINDOW_SEC * fs)

        print("\nMindRove stream started.")
        print(f"Sampling rate : {fs} Hz")
        print(f"EXG channels  : {exg_channels}")
        print(f"Accel channels: {accel_channels}")
        print(f"Gyro channels : {gyro_channels}")
        print("\nCommands:")
        print("  calibrate or c   -> record new rest and grab calibration")
        print("  quit or q        -> exit\n")

        print(f"IMU freeze enabled: {USE_IMU_FREEZE}")
        print(f"IMU freeze threshold: {IMU_FREEZE_THRESHOLD}\n")

        last_state_printed = None
        last_periodic_print = 0.0
        last_freeze_printed = None

        while True:
            while not cmd_queue.empty():
                cmd = cmd_queue.get()

                if cmd in ["quit", "q"]:
                    print("Exiting...")
                    return

                elif cmd in ["calibrate", "c"]:
                    classifier.calibrate(board_shim, exg_channels, fs)

                elif cmd:
                    print(f"Unknown command: {cmd}")

            if classifier.calibrated:
                data = get_latest_window(board_shim, window_points)
                if data is not None:
                    emg_score, emg_debug = extract_activity_score(data, exg_channels, fs)
                    movement_score = extract_movement_score(data, accel_channels, gyro_channels)

                    state, smooth_score, smooth_movement, freeze_active = classifier.classify(
                        emg_score,
                        movement_score,
                        IMU_FREEZE_THRESHOLD,
                        USE_IMU_FREEZE
                    )

                    now = time.time()

                    state_changed = (state != last_state_printed)
                    freeze_changed = (freeze_active != last_freeze_printed)

                    if state_changed or freeze_changed:
                        freeze_text = "FREEZE" if freeze_active else "LIVE"
                        print(
                            f"MODE: {freeze_text} | "
                            f"STATE: {state} | "
                            f"emg_score={smooth_score:.4f} | "
                            f"imu_score={smooth_movement:.4f} | "
                            f"band_mean={emg_debug['band_mean']:.4f} | "
                            f"rms_mean={emg_debug['rms_mean']:.4f}"
                        )
                        last_state_printed = state
                        last_freeze_printed = freeze_active
                        last_periodic_print = now

                    elif now - last_periodic_print >= PRINT_EVERY_SEC:
                        freeze_text = "FREEZE" if freeze_active else "LIVE"
                        print(
                            f"MODE: {freeze_text} | "
                            f"STATE: {state} | "
                            f"emg_score={smooth_score:.4f} | "
                            f"imu_score={smooth_movement:.4f} | "
                            f"band_mean={emg_debug['band_mean']:.4f} | "
                            f"rms_mean={emg_debug['rms_mean']:.4f}"
                        )
                        last_periodic_print = now

            time.sleep(STEP_SEC)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except BaseException as e:
        logging.exception("Unhandled exception: %s", e)
    finally:
        if board_shim is not None and board_shim.is_prepared():
            print("Releasing MindRove session...")
            board_shim.release_session()


if __name__ == "__main__":
    main()