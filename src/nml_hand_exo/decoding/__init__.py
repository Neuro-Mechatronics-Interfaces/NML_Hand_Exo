"""Modular EMG intent-decoding components.

This package contains no exoskeleton hardware commands.  Decoders produce a
guarded intent decision that can be published through the existing LSL
contract and consumed by the hand-exo GUI.
"""

from .contracts import DecoderDecision, OrientationSample, PairEvaluation
from .features import FeatureConfig, SignalQuality, assess_signal_quality, extract_emg_features
from .models import ShrinkageLDAIntentModel
from .layout import StreamLayout, parse_channel_spec
from .orientation import ContinuousRestAdapter, orientation_from_accel
from .pipeline import IntentDecoderPipeline
from .preprocessing import PreprocessConfig, preprocess_emg
from .selection import rank_intent_pairs
from .session import IntentCaptureSession
from .xdf_session import canonical_intent_label, import_xdf_file, import_xdf_session

__all__ = [
    "ContinuousRestAdapter",
    "DecoderDecision",
    "FeatureConfig",
    "IntentCaptureSession",
    "IntentDecoderPipeline",
    "OrientationSample",
    "PairEvaluation",
    "PreprocessConfig",
    "ShrinkageLDAIntentModel",
    "StreamLayout",
    "SignalQuality",
    "assess_signal_quality",
    "canonical_intent_label",
    "extract_emg_features",
    "import_xdf_file",
    "import_xdf_session",
    "orientation_from_accel",
    "parse_channel_spec",
    "preprocess_emg",
    "rank_intent_pairs",
]
