@echo off
setlocal
set OPENBLAS_NUM_THREADS=1
set OMP_NUM_THREADS=1
cd /d "%~dp0"
where uv >nul 2>nul
if errorlevel 1 (
  echo uv was not found. Install uv, then run this launcher again.
  pause
  exit /b 1
)
echo Preparing the simulation and geometry tools. First launch needs internet.
rem Keep running controller entrypoints installed; the viewer runs from source.
uv sync --extra sensor-identification --no-install-project --inexact
if errorlevel 1 (
  echo Setup failed. See the error above.
  pause
  exit /b 1
)
echo Keep this window open while using the simulation. Press Ctrl+C to stop.
uv run --no-sync python digital_twin/start_sensor_simulation.py
if errorlevel 1 pause
