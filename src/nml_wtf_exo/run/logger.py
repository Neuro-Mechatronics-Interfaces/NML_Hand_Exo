import sys
from PyQt5.QtWidgets import QApplication
from nml_wtf_exo.gui.LoggerApp import LoggerApp

def main():
    app = QApplication(sys.argv)
    window = LoggerApp()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
