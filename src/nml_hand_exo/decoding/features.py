from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FeatureConfig:
    common_mode: str = "median"
    log_compress: bool = True
    include_waveform_length: bool = False


@dataclass(frozen=True)
class SignalQuality:
    channel_rms: np.ndarray
    flat_channels: tuple[int, ...]
    saturated_channels: tuple[int, ...]
    noisy_channels: tuple[int, ...]

    @property
    def usable_fraction(self) -> float:
        count = int(self.channel_rms.size)
        bad = set(self.flat_channels) | set(self.saturated_channels) | set(self.noisy_channels)
        return 0.0 if count == 0 else (count - len(bad)) / count


def _channels_by_samples(emg: np.ndarray) -> np.ndarray:
    values = np.asarray(emg, dtype=np.float64)
    if values.ndim != 2:
        raise ValueError("EMG window must have shape (channels, samples)")
    if values.shape[0] < 1 or values.shape[1] < 2:
        raise ValueError("EMG window must contain at least one channel and two samples")
    if not np.all(np.isfinite(values)):
        raise ValueError("EMG window contains non-finite values")
    return values


def extract_emg_features(emg: np.ndarray, config: FeatureConfig | None = None) -> np.ndarray:
    """Return a compact feature vector for 8-channel or HD-EMG windows."""
    config = config or FeatureConfig()
    values = _channels_by_samples(emg)
    if config.common_mode == "median":
        values = values - np.median(values, axis=0, keepdims=True)
    elif config.common_mode == "mean":
        values = values - np.mean(values, axis=0, keepdims=True)
    elif config.common_mode != "none":
        raise ValueError(f"Unknown common-mode method: {config.common_mode}")
    rms = np.sqrt(np.mean(values * values, axis=1) + 1e-12)
    output = np.log1p(rms) if config.log_compress else rms
    if config.include_waveform_length:
        waveform_length = np.mean(np.abs(np.diff(values, axis=1)), axis=1)
        if config.log_compress:
            waveform_length = np.log1p(waveform_length)
        output = np.concatenate([output, waveform_length])
    return output.astype(np.float64, copy=False)


def assess_signal_quality(emg: np.ndarray) -> SignalQuality:
    values = _channels_by_samples(emg)
    rms = np.sqrt(np.mean(values * values, axis=1) + 1e-12)
    spread = np.std(values, axis=1)
    peak = np.max(np.abs(values), axis=1)
    median_rms = max(float(np.median(rms)), 1e-12)
    flat = tuple(np.flatnonzero(spread < max(1e-9, median_rms * 1e-4)).tolist())
    noisy = tuple(np.flatnonzero(rms > median_rms * 8.0).tolist())
    # Repeated extrema are a device-independent clipping proxy; an ADC-specific
    # threshold can be added by a stream adapter when its scale is known.
    saturated = []
    for channel in range(values.shape[0]):
        at_peak = np.isclose(np.abs(values[channel]), peak[channel], rtol=0.0, atol=1e-12)
        if peak[channel] > 0 and np.mean(at_peak) >= 0.02:
            saturated.append(channel)
    return SignalQuality(rms, flat, tuple(saturated), noisy)
