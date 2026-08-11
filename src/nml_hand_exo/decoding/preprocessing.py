from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.signal import butter, filtfilt, iirnotch, sosfiltfilt


@dataclass(frozen=True)
class PreprocessConfig:
    sample_rate_hz: float
    highpass_hz: float = 20.0
    lowpass_hz: float = 200.0
    notch_hz: float = 60.0
    notch_quality: float = 30.0


def preprocess_emg(emg: np.ndarray, config: PreprocessConfig) -> np.ndarray:
    """Band-pass and notch a channels-by-samples EMG window.

    Filters are omitted individually when the stream rate cannot represent
    their requested frequency. This keeps low-rate envelope streams usable
    while applying the full path to conventional raw EMG.
    """
    values = np.asarray(emg, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] < 2:
        raise ValueError("EMG window must have shape (channels, samples)")
    if not np.all(np.isfinite(values)):
        raise ValueError("EMG window contains non-finite values")
    sample_rate = float(config.sample_rate_hz)
    if not np.isfinite(sample_rate) or sample_rate <= 0:
        raise ValueError("Sample rate must be positive")
    nyquist = sample_rate / 2.0
    output = values - np.mean(values, axis=1, keepdims=True)
    low = float(config.highpass_hz)
    high = min(float(config.lowpass_hz), nyquist * 0.90)
    if 0.0 < low < high:
        sos = butter(4, [low, high], btype="bandpass", fs=sample_rate, output="sos")
        try:
            output = sosfiltfilt(sos, output, axis=1)
        except ValueError:
            # Very short windows still receive centering and feature extraction.
            pass
    notch = float(config.notch_hz)
    if 0.0 < notch < nyquist * 0.95:
        b, a = iirnotch(notch, float(config.notch_quality), fs=sample_rate)
        try:
            output = filtfilt(b, a, output, axis=1)
        except ValueError:
            pass
    return output
