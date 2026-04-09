#!/usr/bin/env python3
"""
Quick test to verify the modified GUI loads without import/syntax errors.
Run this to validate the threading changes before connecting to actual hardware.
"""

import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

try:
    from PyQt5.QtWidgets import QApplication, QMessageBox
    from nml_hand_exo.applications.hand_exo_gui import HandExoGUI, ConnectionWorker
    
    print("✓ All imports successful")
    print("✓ HandExoGUI class loaded")
    print("✓ ConnectionWorker class loaded")
    
    # Quick instantiation test (without showing GUI)
    app = QApplication(sys.argv)
    gui = HandExoGUI()
    print("✓ GUI initialized successfully")
    print("✓ Threading imports (QThread, pyqtSignal) working")
    print("\n✅ All checks passed! The GUI threading fix is syntactically correct.")
    print("\nNext steps:")
    print("1. Activate your Python 3.10+ venv (.handexo311)")
    print("2. Run: python -m nml_hand_exo.applications.hand_exo_gui")
    print("3. Test connection to COM13 — GUI should stay responsive during the 5s handshake")
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
