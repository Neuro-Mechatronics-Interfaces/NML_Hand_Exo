# -- Stylesheet ------------------------------------------------------------
# CMU core palette: Carnegie Red #C41230, Black #000000,
# Iron Gray #6D6E71, Steel Gray #E0E0E0, and White #FFFFFF.

DARK_STYLE = """
QWidget {
    background-color: #0b0b0b;
    color: #e0e0e0;
    font-family: "Open Sans", "Segoe UI", "Helvetica Neue", Arial, sans-serif;
}
QGroupBox {
    background-color: #171717;
    border: 1px solid #333333;
    border-radius: 6px;
    margin-top: 1.2em;
    padding-top: 1.0em;
    font-weight: bold;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
    color: #ef3a47;
}
QPushButton {
    background-color: #2e2e2e;
    color: #e0e0e0;
    border: 1px solid #444444;
    border-radius: 4px;
    padding: 5px 14px;
    min-height: 1.4em;
}
QPushButton:hover {
    background-color: #3a3a3a;
    border-color: #c41230;
}
QPushButton:pressed {
    background-color: #6d6e71;
    color: #ffffff;
}
QPushButton:disabled {
    background-color: #252525;
    color: #555555;
    border-color: #333333;
}
QPushButton[accent="true"] {
    background-color: #c41230;
    color: #ffffff;
    border-color: #ef3a47;
}
QPushButton[accent="true"]:hover {
    background-color: #d61a3c;
}
QPushButton[accent="true"]:pressed {
    background-color: #941120;
}
QPushButton[accent="true"]:disabled {
    background-color: #3d2026;
    color: #666666;
    border-color: #592630;
}
QPushButton[danger="true"] {
    background-color: #4a0712;
    color: #ffffff;
    border: 2px solid #ef3a47;
    font-weight: bold;
}
QPushButton[danger="true"]:hover {
    background-color: #700b1d;
    border-color: #ff6673;
}
QPushButton[danger="true"]:pressed {
    background-color: #941120;
}
QPushButton[danger="true"]:disabled {
    background-color: #2b1519;
    color: #666666;
    border-color: #592630;
}
QLineEdit, QComboBox {
    background-color: #2a2a2a;
    color: #e0e0e0;
    border: 1px solid #444444;
    border-radius: 4px;
    padding: 4px 8px;
}
QLineEdit:focus, QComboBox:focus {
    border-color: #ef3a47;
}
QComboBox::drop-down {
    border: none;
    background: #333333;
    width: 20px;
}
QComboBox QAbstractItemView {
    background-color: #2a2a2a;
    color: #e0e0e0;
    selection-background-color: #c41230;
}
QTextEdit {
    background-color: #111111;
    color: #aaaaaa;
    border: 1px solid #333333;
    border-radius: 4px;
    font-family: "Cascadia Code", "Consolas", "Courier New", monospace;
}
QScrollArea {
    border: none;
    background-color: #0b0b0b;
}
QScrollBar:vertical {
    background: #0b0b0b;
    width: 10px;
}
QScrollBar::handle:vertical {
    background: #444444;
    border-radius: 4px;
    min-height: 20px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}
QLabel#title {
    color: #ffffff;
    font-weight: bold;
}
QLabel#accent-line {
    background-color: #c41230;
    max-height: 2px;
    min-height: 2px;
}
QLabel#status-connected {
    color: #27ae60;
    font-weight: bold;
}
QLabel#status-disconnected {
    color: #c0392b;
    font-weight: bold;
}
QFrame#motor-row {
    background-color: #252525;
    border-radius: 4px;
    padding: 4px;
}
QDialog {
    background-color: #0b0b0b;
    color: #e0e0e0;
}
QTabWidget::pane {
    border: 1px solid #333333;
    background-color: #0b0b0b;
}
QTabBar::tab {
    background-color: #2e2e2e;
    color: #e0e0e0;
    padding: 6px 18px;
    border: 1px solid #444444;
    border-bottom: none;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    min-width: 80px;
}
QTabBar::tab:selected {
    background-color: #0b0b0b;
    color: #ffffff;
    border-color: #ef3a47;
    border-bottom: 2px solid #c41230;
}
QTabBar::tab:hover:!selected {
    background-color: #3a3a3a;
}
QTableWidget {
    background-color: #0b0b0b;
    alternate-background-color: #222222;
    color: #e0e0e0;
    gridline-color: #333333;
    border: 1px solid #333333;
}
QTableWidget::item {
    color: #e0e0e0;
    padding: 4px;
}
QHeaderView::section {
    background-color: #2e2e2e;
    color: #e0e0e0;
    border: 1px solid #333333;
    padding: 4px 8px;
    font-weight: bold;
}
QCheckBox {
    color: #e0e0e0;
    spacing: 6px;
}
QCheckBox::indicator {
    width: 14px;
    height: 14px;
    border: 1px solid #555555;
    background-color: #2a2a2a;
    border-radius: 2px;
}
QCheckBox::indicator:checked {
    background-color: #c41230;
    border-color: #ef3a47;
}
"""
