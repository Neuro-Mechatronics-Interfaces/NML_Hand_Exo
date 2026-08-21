@echo off
REM Double-click launcher for the NML Hand Exoskeleton operator GUI.
setlocal
cd /d "%~dp0"

if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" launch_gui.py
) else (
    python launch_gui.py
)

if errorlevel 1 pause
endlocal
