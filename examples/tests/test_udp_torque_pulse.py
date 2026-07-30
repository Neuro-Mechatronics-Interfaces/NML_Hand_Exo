import unittest

from nml_hand_exo.interface._udp_torque_pulse import (
    TorquePulse,
    raised_cosine_amplitude,
    smoothstep,
)


class RaisedCosineAmplitudeTests(unittest.TestCase):
    def test_zero_at_both_ends(self):
        self.assertEqual(raised_cosine_amplitude(0, 1000), 0.0)
        self.assertEqual(raised_cosine_amplitude(1000, 1000), 0.0)
        # Outside the window the pulse applies nothing.
        self.assertEqual(raised_cosine_amplitude(-10, 1000), 0.0)
        self.assertEqual(raised_cosine_amplitude(1200, 1000), 0.0)

    def test_peaks_at_mid_duration(self):
        self.assertAlmostEqual(raised_cosine_amplitude(500, 1000), 1.0, places=9)

    def test_monotonic_rise_to_peak(self):
        samples = [raised_cosine_amplitude(t, 1000) for t in range(0, 501, 50)]
        for earlier, later in zip(samples, samples[1:]):
            self.assertLessEqual(earlier, later)
        self.assertTrue(all(0.0 <= v <= 1.0 for v in samples))

    def test_symmetric_about_midpoint(self):
        self.assertAlmostEqual(
            raised_cosine_amplitude(250, 1000),
            raised_cosine_amplitude(750, 1000),
            places=9,
        )

    def test_nonpositive_duration_is_zero(self):
        self.assertEqual(raised_cosine_amplitude(10, 0), 0.0)


class SmoothstepTests(unittest.TestCase):
    def test_clamped_and_monotonic(self):
        self.assertEqual(smoothstep(-1.0), 0.0)
        self.assertEqual(smoothstep(0.0), 0.0)
        self.assertEqual(smoothstep(1.0), 1.0)
        self.assertEqual(smoothstep(2.0), 1.0)
        self.assertAlmostEqual(smoothstep(0.5), 0.5, places=9)
        samples = [smoothstep(t / 10.0) for t in range(0, 11)]
        for earlier, later in zip(samples, samples[1:]):
            self.assertLessEqual(earlier, later)


class TorquePulseTests(unittest.TestCase):
    def test_samples_scale_peak_by_envelope(self):
        pulse = TorquePulse({2: 100.0, 3: -80.0}, duration_ms=1000, start_ms=0.0)
        currents, done = pulse.sample(500)
        self.assertFalse(done)
        self.assertAlmostEqual(currents[2], 100.0, places=6)
        self.assertAlmostEqual(currents[3], -80.0, places=6)

    def test_finished_pulse_reports_zero_and_done(self):
        pulse = TorquePulse({2: 100.0}, duration_ms=1000, start_ms=0.0)
        currents, done = pulse.sample(1000)
        self.assertTrue(done)
        self.assertEqual(currents, {2: 0.0})

    def test_start_offset_and_direction_preserved(self):
        pulse = TorquePulse({5: 60.0}, duration_ms=400, start_ms=1000.0)
        self.assertFalse(pulse.is_done(1200))
        currents, _ = pulse.sample(1200)  # midpoint -> peak
        self.assertAlmostEqual(currents[5], 60.0, places=6)

    def test_rejects_unknown_shape(self):
        with self.assertRaises(ValueError):
            TorquePulse({2: 10.0}, duration_ms=100, start_ms=0.0, shape="square")


if __name__ == "__main__":
    unittest.main()
