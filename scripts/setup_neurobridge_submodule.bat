@echo off
setlocal

set "SUBMODULE_PATH=external\NeuroBridge"
set "SUBMODULE_URL=%~1"

if "%SUBMODULE_URL%"=="" (
  set "SUBMODULE_URL=https://github.com/JShulgach/NeuroBridge.git"
)

if not exist ".git" (
  echo [ERROR] Run this from the NML_Hand_Exo repository root.
  exit /b 1
)

if not exist "%SUBMODULE_PATH%\.git" (
  echo [INFO] Adding NeuroBridge submodule at %SUBMODULE_PATH%
  git -c protocol.file.allow=always submodule add "%SUBMODULE_URL%" "%SUBMODULE_PATH%"
  if errorlevel 1 exit /b 1
) else (
  echo [INFO] NeuroBridge submodule already exists at %SUBMODULE_PATH%
)

echo [INFO] Initializing/updating submodules
 git submodule update --init --recursive
if errorlevel 1 exit /b 1

echo [INFO] Bootstrapping Python environment and installs
python scripts\setup_ai_submodule_env.py --submodule-path "%SUBMODULE_PATH%"
if errorlevel 1 exit /b 1

echo.
echo [DONE] NeuroBridge submodule + AI environment are ready.
echo [NEXT] AI agent: scripts\run_ai_agent.bat --bundle nml_default
echo [NEXT] Visualizer: scripts\run_exo_visualizer.bat
