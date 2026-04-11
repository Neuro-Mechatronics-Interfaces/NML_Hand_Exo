@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
set "REPO_ROOT=%SCRIPT_DIR%.."
pushd "%REPO_ROOT%" >nul

if not exist .venv\Scripts\python.exe (
  echo [ERROR] Missing .venv\Scripts\python.exe
  echo [HINT] Run: python scripts\setup_ai_submodule_env.py
  popd >nul
  exit /b 1
)

if exist .venv\Scripts\pythonw.exe (
  start "NML AI Assist GUI" /D "%REPO_ROOT%" .venv\Scripts\pythonw.exe scripts\run_ai_assist_gui.py %*
) else (
  start "NML AI Assist GUI" /D "%REPO_ROOT%" .venv\Scripts\python.exe scripts\run_ai_assist_gui.py %*
)

popd >nul
