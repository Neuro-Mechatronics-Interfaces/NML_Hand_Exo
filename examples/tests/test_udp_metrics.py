import math
import unittest

from nml_hand_exo.interface._udp_metrics import TimeWeightedBacklogEMA


class TimeWeightedBacklogEMATests(unittest.TestCase):
    def test_tracks_current_depth_and_time_weighted_average(self):
        metric = TimeWeightedBacklogEMA(time_constant_s=2.0)
        metric.reset(now=0.0)
        metric.enqueue(now=0.0)

        current, average = metric.snapshot(now=1.0)

        self.assertEqual(current, 1)
        self.assertAlmostEqual(average, 1.0 - math.exp(-0.5))

        metric.complete(now=1.0)
        current, average = metric.snapshot(now=3.0)
        self.assertEqual(current, 0)
        self.assertAlmostEqual(
            average, (1.0 - math.exp(-0.5)) * math.exp(-1.0)
        )

    def test_depth_never_becomes_negative(self):
        metric = TimeWeightedBacklogEMA()
        metric.reset(now=0.0)
        metric.complete(now=1.0)

        self.assertEqual(metric.snapshot(now=1.0)[0], 0)


if __name__ == "__main__":
    unittest.main()
