import numpy as np

from nml_hand_exo.applications.emg_intent_decoder_gui import EmgIntentDecoderWindow
from nml_hand_exo.decoding import DecoderDecision, IntentOutputStabilizer
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
