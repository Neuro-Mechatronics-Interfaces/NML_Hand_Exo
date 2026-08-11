import tempfile
import unittest
from pathlib import Path

import numpy as np

from nml_hand_exo.decoding import (
    ContinuousRestAdapter,
    FeatureConfig,
    IntentCaptureSession,
    IntentDecoderPipeline,
    OrientationSample,
    PreprocessConfig,
    extract_emg_features,
    parse_channel_spec,
    preprocess_emg,
    rank_intent_pairs,
    canonical_intent_label,
)


class IntentDecodingTests(unittest.TestCase):
    def test_task_labels_are_canonicalized_for_decoder_sessions(self):
        self.assertEqual(canonical_intent_label("Hand Close"), "attempt_hand_close")
        self.assertEqual(canonical_intent_label("grasp:pinch"), "attempt_grasp_pinch")
        self.assertEqual(canonical_intent_label("attempt_custom"), "attempt_custom")

    def test_hd_emg_preserves_128_channels(self):
        rng = np.random.default_rng(3)
        raw = rng.normal(size=(128, 100))
        processed = preprocess_emg(raw, PreprocessConfig(sample_rate_hz=1000.0))
        feature = extract_emg_features(
            processed, FeatureConfig(common_mode="none")
        )
        self.assertEqual(processed.shape, raw.shape)
        self.assertEqual(feature.shape, (128,))

    def test_channel_ranges(self):
        self.assertEqual(parse_channel_spec("0-2,5,7-6", 8), (0, 1, 2, 5, 7, 6))
        self.assertEqual(len(parse_channel_spec("0-127", 128)), 128)
        with self.assertRaises(ValueError):
            parse_channel_spec("128", 128)

    def test_continuous_rest_adapter_removes_orientation_baseline(self):
        roll = np.linspace(-80.0, 80.0, 80)
        pitch = np.linspace(-30.0, 30.0, 80)
        radians = np.deg2rad(roll)
        features = np.column_stack([2.0 + np.sin(radians), 4.0 - 0.5 * np.cos(radians)])
        labels = np.full(80, "rest", dtype=object)
        adapter = ContinuousRestAdapter().fit(features, labels, roll, pitch)
        corrected = adapter.transform(features, roll, pitch)
        self.assertLess(float(np.max(np.abs(np.mean(corrected, axis=0)))), 0.05)

    def test_session_round_trip(self):
        session = IntentCaptureSession(participant_id="p01", device_name="8ch", channel_count=8)
        session.add(np.arange(8), "rest", "rest-01", 10.0, -5.0, np.ones((8, 10)))
        session.add(
            np.arange(8) + 1, "attempt_open", "attempt_open-01", 12.0, -4.0,
            np.full((8, 10), 2.0),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "session.npz"
            session.save(path)
            restored = IntentCaptureSession.load(path)
        self.assertEqual(restored.participant_id, "p01")
        self.assertEqual(restored.channel_count, 8)
        np.testing.assert_allclose(restored.arrays()[0], session.arrays()[0])
        np.testing.assert_allclose(restored.emg_windows, session.emg_windows)
        self.assertEqual(restored.labels, session.labels)

    def test_pair_ranking_and_guarded_pipeline(self):
        rng = np.random.default_rng(9)
        rows, labels, groups, rolls, pitches = [], [], [], [], []
        centers = {
            "rest": np.array([0.0, 0.0, 0.0]),
            "attempt_open": np.array([-4.0, 0.0, 0.0]),
            "attempt_close": np.array([4.0, 0.0, 0.0]),
            "weak": np.array([0.0, 0.15, 0.0]),
        }
        for label, center in centers.items():
            for repetition in range(4):
                roll = -60.0 + repetition * 40.0
                posture = np.array([0.0, 0.3 * np.sin(np.deg2rad(roll)), 0.0])
                for _ in range(12):
                    rows.append(center + posture + rng.normal(0.0, 0.18, 3))
                    labels.append(label)
                    groups.append(f"{label}-{repetition}")
                    rolls.append(roll)
                    pitches.append(0.0)
        X = np.asarray(rows)
        y = np.asarray(labels, dtype=object)
        group_values = np.asarray(groups, dtype=object)
        roll_values = np.asarray(rolls)
        pitch_values = np.asarray(pitches)
        ranking = rank_intent_pairs(X, y, group_values, roll_values, pitch_values, folds=4)
        self.assertEqual(
            {ranking[0].open_label, ranking[0].close_label},
            {"attempt_open", "attempt_close"},
        )
        keep = np.isin(y, ["rest", "attempt_open", "attempt_close"])
        pipeline = IntentDecoderPipeline(
            open_label="attempt_open",
            close_label="attempt_close",
            require_orientation=True,
        ).fit(X[keep], y[keep], roll_values[keep], pitch_values[keep])
        rejected = pipeline.predict(np.zeros(3), OrientationSample())
        self.assertTrue(rejected.rejected)
        self.assertEqual(rejected.signed_intent, 0.0)
        close = pipeline.predict(np.array([4.0, 0.0, 0.0]), OrientationSample(0.0, 0.0))
        self.assertGreater(close.signed_intent, 0.90)
        partial_close = pipeline.predict(
            np.array([2.0, 0.0, 0.0]), OrientationSample(0.0, 0.0)
        )
        partial_open = pipeline.predict(
            np.array([-2.0, 0.0, 0.0]), OrientationSample(0.0, 0.0)
        )
        self.assertGreater(partial_close.signed_intent, 0.25)
        self.assertLess(partial_close.signed_intent, 0.75)
        self.assertLess(partial_open.signed_intent, -0.25)
        self.assertGreater(partial_open.signed_intent, -0.75)
        self.assertGreater(close.close_activation, close.open_activation)
        batch_features = np.asarray([
            [0.0, 0.0, 0.0],
            [-2.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
        ])
        batch = pipeline.project_continuous(
            batch_features,
            np.zeros(3),
            np.zeros(3),
        )
        individual = np.asarray([
            pipeline.predict(row, OrientationSample(0.0, 0.0)).signed_intent
            for row in batch_features
        ])
        np.testing.assert_allclose(batch["signed_intent"], individual)


if __name__ == "__main__":
    unittest.main()
