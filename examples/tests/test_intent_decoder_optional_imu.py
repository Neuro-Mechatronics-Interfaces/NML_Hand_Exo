import time
from types import SimpleNamespace

import numpy as np

from nml_hand_exo.applications.emg_intent_decoder_gui import (
    DEVICE_PRESETS,
    EmgIntentDecoderWindow,
    IMU_FRESHNESS_TIMEOUT_S,
)
from nml_hand_exo.decoding import DecoderDecision, IntentOutputStabilizer


class _UnexpectedBuffer:
    def snapshot(self):
        raise AssertionError("stale IMU samples must not be reused")


class _Label:
    def __init__(self):
        self.text = ""

    def setText(self, text):
        self.text = text


def test_runtime_pipeline_defaults_to_global_emg_baseline():
    pipeline = EmgIntentDecoderWindow._make_runtime_pipeline(
        "hand_open", "hand_close"
    )

    assert pipeline.open_label == "hand_open"
    assert pipeline.close_label == "hand_close"
    assert pipeline.require_orientation is False


def test_runtime_pipeline_can_explicitly_require_orientation():
    pipeline = EmgIntentDecoderWindow._make_runtime_pipeline(
        "hand_open", "hand_close", use_orientation=True
    )

    assert pipeline.require_orientation is True


def test_mindrove_presets_keep_combined_and_split_channel_indices_distinct():
    combined = DEVICE_PRESETS["MindRove 8 + IMU"]
    playback = DEVICE_PRESETS["MindRove XDF playback"]

    assert combined[:3] == ("1-8", "9-11", "12-14")
    assert playback[:3] == ("0-7", "0-2", "3-5")


def test_imu_freshness_requires_worker_metadata_and_recent_sample():
    now = time.monotonic()
    gui = SimpleNamespace(
        _imu_worker=object(),
        _imu_meta={"channel_count": 9},
        _last_imu_chunk_monotonic=now - 0.1,
    )

    assert EmgIntentDecoderWindow._imu_is_fresh(gui, now)

    gui._last_imu_chunk_monotonic = now - IMU_FRESHNESS_TIMEOUT_S - 0.01
    assert not EmgIntentDecoderWindow._imu_is_fresh(gui, now)

    gui._imu_worker = None
    assert not EmgIntentDecoderWindow._imu_is_fresh(gui, now)


def test_stale_imu_falls_back_without_reusing_buffer():
    gui = SimpleNamespace(
        _imu_worker=object(),
        _imu_meta={"channel_count": 9},
        _last_imu_chunk_monotonic=(
            time.monotonic() - IMU_FRESHNESS_TIMEOUT_S - 1.0
        ),
        _imu_buffer=_UnexpectedBuffer(),
    )

    orientation = EmgIntentDecoderWindow._latest_orientation(gui)

    assert not orientation.is_available


def test_decoder_tick_continues_when_imu_is_unavailable():
    decisions = []
    published_zeros = []
    orientation = SimpleNamespace(roll_deg=None)
    decision = DecoderDecision(
        state="hand_close",
        signed_intent=0.5,
        confidence=1.0,
        rejected=False,
    )
    gui = SimpleNamespace(
        _test_signal_active=False,
        _worker=object(),
        _last_chunk_monotonic=time.monotonic(),
        _latest_feature=lambda: (
            np.ones(8, dtype=np.float64),
            orientation,
            np.random.default_rng(0).normal(size=(8, 125)),
        ),
        quality_status=_Label(),
        _pipeline=SimpleNamespace(
            require_orientation=False,
            open_label="hand_open",
            close_label="hand_close",
            rest_label="rest",
            predict=lambda feature, sample_orientation: (
                decisions.append((feature, sample_orientation)) or decision
            )
        ),
        _output_stabilizer=IntentOutputStabilizer(),
        _show_decision=lambda value: decisions.append(value),
        _publish_zero=lambda: published_zeros.append(True),
        _outlet=None,
    )

    EmgIntentDecoderWindow._tick(gui)

    assert decisions[-1].signed_intent > 0.0
    assert published_zeros == []
    assert "orientation=global EMG baseline" in gui.quality_status.text
