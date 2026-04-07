@echo off
setlocal

if not exist .venv\Scripts\python.exe (
  echo [ERROR] Missing .venv\Scripts\python.exe
  echo [HINT] Run: python scripts\setup_ai_submodule_env.py
  exit /b 1
)

.venv\Scripts\python.exe scripts\run_exo_visualizer.py %*
