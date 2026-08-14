from __future__ import annotations

import numpy as np

from nml_hand_exo.control import StateTriggerRMS


def test_baseline_round_trip_uses_one_attribute(tmp_path):
    trigger = StateTriggerRMS.__new__(StateTriggerRMS)
    trigger.baseline_rms = {0: 1.25, 1: 2.5}
    path = tmp_path / "baseline.npy"

    trigger.save_baseline(str(path))
    trigger.baseline_rms = {}
    trigger.load_baseline(str(path))

    assert trigger.baseline_rms == {0: 1.25, 1: 2.5}
