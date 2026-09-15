import math
import os
from types import SimpleNamespace
from unittest.mock import patch
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt5.QtWidgets import QApplication
import pyqtgraph as pg

from nml_hand_exo.applications._telemetry_plot import finite_curve_data
from nml_hand_exo.applications.hand_exo_gui import HandExoGUI


def test_finite_coordinates_preserve_gaps_and_source_boundaries():
    assert finite_curve_data(range(7), [0.1, float("nan"), 0.2, 0.3, float("inf"), 0.4, 0.5]) == (
        [0, 2, 3, 5, 6], [0.1, 0.2, 0.3, 0.4, 0.5], [False, True, False, True, False])


@pytest.mark.parametrize("bad", [None, float("nan"), float("inf"), -float("inf"), "nan"])
def test_invalid_telemetry_clears_average_and_first_valid_sample_recovers(bad):
    gui = SimpleNamespace(_telemetry_buffers={"torques": {}})
    HandExoGUI._buffer_telemetry_field(gui, "torques", {16: 0.1})
    HandExoGUI._buffer_telemetry_field(gui, "torques", {16: bad})
    assert HandExoGUI._averaged_telemetry_field(gui, "torques") == {}
    HandExoGUI._buffer_telemetry_field(gui, "torques", {16: 0.2})
    assert HandExoGUI._averaged_telemetry_field(gui, "torques") == {16: 0.2}


def test_real_plot_recovers_after_all_missing_and_disable_reenable_sources():
    app = QApplication.instance() or QApplication([])
    plot = pg.PlotWidget()
    gui = SimpleNamespace(_torque_plot=plot, motor_names=["index"])
    HandExoGUI._rebuild_torque_plot(gui)
    gui._torque_plot_t0 = 0
    try:
        for timestamp, value, source in [(1, 0.1, "measured"), (2, 0.2, "estimated"),
                                         (3, float("nan"), "measured"),
                                         (40, None, "unavailable"), (41, float("inf"), "estimated"),
                                         (42, 0.3, "measured"), (43, 0.4, "measured")]:
            with patch("nml_hand_exo.applications.hand_exo_gui.time.monotonic", return_value=timestamp):
                HandExoGUI._push_torque_plot(gui, {"index": value}, {"index": source})
            app.processEvents()
        x, y = gui._torque_curves["index"].getData()
        assert list(x) == [42, 43] and list(y) == [0.3, 0.4]
        assert all(math.isfinite(v) for axis in plot.viewRange() for v in axis)
        assert plot.viewRange()[0][1] >= 43
        assert not plot.grab().isNull()
    finally:
        plot.close()
