# -- Stylesheet ------------------------------------------------------------

DARK_STYLE = """
QWidget {
    background-color: #1a1a1a;
    color: #e0e0e0;
    font-family: "Segoe UI", "Helvetica Neue", Arial, sans-serif;
}
QGroupBox {
    background-color: #222222;
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
    color: #c0392b;
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
    border-color: #c0392b;
}
QPushButton:pressed {
    background-color: #c0392b;
    color: #ffffff;
}
QPushButton:disabled {
    background-color: #252525;
    color: #555555;
    border-color: #333333;
}
QPushButton[accent="true"] {
    background-color: #8b1a1a;
    color: #ffffff;
    border-color: #c0392b;
}
QPushButton[accent="true"]:hover {
    background-color: #a52222;
}
QPushButton[accent="true"]:pressed {
    background-color: #c0392b;
}
QPushButton[accent="true"]:disabled {
    background-color: #3a2020;
    color: #666666;
    border-color: #442222;
}
QLineEdit, QComboBox {
    background-color: #2a2a2a;
    color: #e0e0e0;
    border: 1px solid #444444;
    border-radius: 4px;
    padding: 4px 8px;
}
QLineEdit:focus, QComboBox:focus {
    border-color: #c0392b;
}
QComboBox::drop-down {
    border: none;
    background: #333333;
    width: 20px;
}
QComboBox QAbstractItemView {
    background-color: #2a2a2a;
    color: #e0e0e0;
    selection-background-color: #c0392b;
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
    background-color: #1a1a1a;
}
QScrollBar:vertical {
    background: #1a1a1a;
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
    background-color: #c0392b;
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
    background-color: #1a1a1a;
    color: #e0e0e0;
}
QTabWidget::pane {
    border: 1px solid #333333;
    background-color: #1a1a1a;
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
    background-color: #1a1a1a;
    color: #ffffff;
    border-color: #c0392b;
    border-bottom: 2px solid #c0392b;
}
QTabBar::tab:hover:!selected {
    background-color: #3a3a3a;
}
QTableWidget {
    background-color: #1a1a1a;
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
    background-color: #c0392b;
    border-color: #c0392b;
}
"""