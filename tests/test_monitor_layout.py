"""Exercise the real nested tabs/scroll area without starting I/O workers."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt5.QtCore import QThread, QPoint
from PyQt5.QtWidgets import QApplication
from nml_hand_exo.applications.hand_exo_gui import HandExoGUI, DARK_STYLE


@pytest.fixture
def monitor(monkeypatch):
    app = QApplication.instance() or QApplication([])
    old_style = app.styleSheet()
    app.setStyleSheet(DARK_STYLE)
    monkeypatch.setattr(QThread, 'start', lambda *args: None)
    monkeypatch.setattr(HandExoGUI, '_load_stream_settings', lambda self: None)
    gui = HandExoGUI()
    gui.show()
    yield app, gui
    gui.close()
    app.setStyleSheet(old_style)


@pytest.mark.parametrize('width,height,rows', [(1600,960,9), (1280,900,18)])
def test_monitor_plot_fits_viewport_after_visiting_tall_pages(monitor, width, height, rows):
    app, gui = monitor
    gui.resize(width, height)
    gui._telem_table.setRowCount(rows)
    for index in (1, 4, 2):  # Setup, Integrations, then Monitor
        gui.main_tabs.setCurrentIndex(index)
        for _ in range(5):
            app.processEvents()
    viewport = gui._main_scroll.viewport()
    assert gui._main_scroll.verticalScrollBar().maximum() == 0
    if width >= 1600:
        assert gui._main_scroll.horizontalScrollBar().maximum() == 0
    splitter_bottom = gui._telem_splitter.mapTo(viewport, QPoint(0, gui._telem_splitter.height())).y()
    assert splitter_bottom <= viewport.height()
    assert 160 <= gui._telem_splitter.height() < viewport.height()
    if gui._torque_plot is not None:
        assert gui._torque_plot.height() == gui._telem_table.height()


def test_monitor_shrinks_on_resize_and_small_windows_can_still_scroll(monitor):
    app, gui = monitor
    gui.main_tabs.setCurrentIndex(2)
    heights = []
    for size in ((1600,1000), (1280,850), (800,500)):
        gui.resize(*size)
        for _ in range(5):
            app.processEvents()
        heights.append(gui._telem_splitter.height())
    assert heights[0] > heights[1] >= heights[2] >= 160
    assert gui._main_scroll.verticalScrollBar().maximum() > 0
