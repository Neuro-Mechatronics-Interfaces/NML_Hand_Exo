import numpy as np
import pytest

from nml_hand_exo.applications.emg_intent_decoder_gui import EmgIntentDecoderWindow
from nml_hand_exo.decoding import (
    DecoderDecision,
    IntentDecoderPipeline,
    IntentOutputStabilizer,
)
from nml_hand_exo.decoding.selection import _balanced_recording_folds


def _decision(value: float, rejected: bool = False) -> DecoderDecision:
    return DecoderDecision(
        state="close" if value > 0 else "open" if value < 0 else "rest",
        signed_intent=value,
        confidence=1.0,
        rejected=rejected,
    )


def test_recording_folds_never_split_one_xdf_recording():
    labels = np.asarray(["rest", "open", "rest", "open", "rest", "open"])
    groups = np.asarray([
        "open_1:rest:1", "open_1:open:1",
        "open_2:rest:1", "open_2:open:1",
        "open_3:rest:1", "open_3:open:1",
    ])

    for train, test in _balanced_recording_folds(labels, groups, folds=3):
        train_recordings = {value.split(":", 1)[0] for value in groups[train]}
        test_recordings = {value.split(":", 1)[0] for value in groups[test]}
        assert train_recordings.isdisjoint(test_recordings)


def test_semantic_mapping_uses_label_words_not_rank_order():
    assert EmgIntentDecoderWindow._semantic_mapping_suggestion(
        "attempt_hand_close", "attempt_hand_open"
    ) == ("attempt_hand_open", "attempt_hand_close")
    assert EmgIntentDecoderWindow._semantic_mapping_suggestion(
        "attempt_wrist_flex", "attempt_index_flex"
    ) == (None, None)


def test_stabilizer_limits_steps_and_rejects_single_sample_reversal():
    stabilizer = IntentOutputStabilizer(
        ema_alpha=1.0, max_step=0.2, switch_samples=3
    )
    first = stabilizer.update(
        _decision(1.0), open_label="open", close_label="close"
    )
    second = stabilizer.update(
        _decision(1.0), open_label="open", close_label="close"
    )
    reversal = stabilizer.update(
        _decision(-1.0), open_label="open", close_label="close"
    )

    assert first.signed_intent == 0.2
    assert second.signed_intent == 0.4
    assert reversal.signed_intent >= 0.0


def test_stabilizer_allows_sustained_direction_switch():
    stabilizer = IntentOutputStabilizer(
        ema_alpha=1.0, max_step=1.0, switch_samples=3
    )
    stabilizer.update(_decision(1.0), open_label="open", close_label="close")
    outputs = [
        stabilizer.update(
            _decision(-1.0), open_label="open", close_label="close"
        ).signed_intent
        for _ in range(4)
    ]

    assert outputs[0] >= 0.0
    assert outputs[1] >= 0.0
    assert outputs[-1] < 0.0


def test_stabilizer_supports_asymmetric_open_and_close_deadbands():
    stabilizer = IntentOutputStabilizer(
        ema_alpha=1.0,
        max_step=1.0,
        open_enter_threshold=0.08,
        close_enter_threshold=0.20,
    )

    open_output = stabilizer.update(
        _decision(-0.10), open_label="open", close_label="close"
    )
    stabilizer.reset()
    close_inside_deadband = stabilizer.update(
        _decision(0.10), open_label="open", close_label="close"
    )
    close_output = stabilizer.update(
        _decision(0.25), open_label="open", close_label="close"
    )

    assert open_output.signed_intent < 0.0
    assert close_inside_deadband.signed_intent == 0.0
    assert close_output.signed_intent > 0.0


def test_stabilizer_shapes_unbounded_projection_then_clamps_command():
    stabilizer = IntentOutputStabilizer(
        ema_alpha=1.0,
        max_step=1.0,
        output_gain=0.5,
        response_exponent=2.0,
    )
    shaped = stabilizer.update(
        DecoderDecision(
            state="close",
            signed_intent=1.0,
            raw_signed_projection=0.5,
            confidence=1.0,
            rejected=False,
        ),
        open_label="open",
        close_label="close",
    )
    saturated = stabilizer.update(
        DecoderDecision(
            state="close",
            signed_intent=1.0,
            raw_signed_projection=3.0,
            confidence=1.0,
            rejected=False,
        ),
        open_label="open",
        close_label="close",
    )

    assert shaped.signed_intent == 0.125
    assert saturated.signed_intent == 1.0


def test_output_gain_does_not_move_projection_deadband():
    stabilizer = IntentOutputStabilizer(
        ema_alpha=1.0,
        max_step=1.0,
        open_enter_threshold=0.08,
        close_enter_threshold=0.08,
        output_gain=3.0,
    )
    inside = stabilizer.update(
        DecoderDecision(
            state="close",
            signed_intent=0.07,
            raw_signed_projection=0.07,
            confidence=1.0,
            rejected=False,
        ),
        open_label="open",
        close_label="close",
    )

    assert inside.signed_intent == 0.0


def test_active_reference_uses_recorded_ninetieth_percentile_not_mvc_median():
    rng = np.random.default_rng(4)
    rest = rng.normal([0.0, 0.0], 0.12, (120, 2))
    open_samples = rng.normal([-1.0, 0.1], 0.18, (120, 2))
    close_samples = rng.normal([1.0, 0.1], 0.18, (120, 2))
    features = np.vstack([rest, open_samples, close_samples])
    labels = np.asarray(["rest"] * 120 + ["open"] * 120 + ["close"] * 120)
    unavailable_orientation = np.full(len(labels), np.nan)

    pipeline = IntentDecoderPipeline(active_reference_quantile=0.90).fit(
        features,
        labels,
        unavailable_orientation,
        unavailable_orientation,
    )
    raw = pipeline.project_continuous(
        features, unavailable_orientation, unavailable_orientation
    )["raw_signed_projection"]

    assert np.quantile(-raw[labels == "open"], 0.90) == pytest.approx(1.0)
    assert np.quantile(raw[labels == "close"], 0.90) == pytest.approx(1.0)
    assert np.max(np.abs(raw)) > 1.0
