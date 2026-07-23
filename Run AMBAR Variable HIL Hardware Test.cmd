@echo off
setlocal
cd /d "%~dp0"

set "AMBAR_PORT=COM6"
if not "%~1"=="" (
  set "AMBAR_PORT=%~1"
) else (
  set /p "AMBAR_PORT=Enter the AMBAR USB COM port [COM6]: "
  if not defined AMBAR_PORT set "AMBAR_PORT=COM6"
)

echo.
echo Starting one nominal causal VARIABLE_HIL case on %AMBAR_PORT%.
echo The launcher will still require you to type CLOSED before software HOME.
echo.
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\run_variable_hil.ps1" -Hardware -AllowActuatorMotion -Port "%AMBAR_PORT%" -Cycles 1 -CaseList nominal -DwellSeconds 0 -MaxTimeSeconds 30
set "RESULT=%ERRORLEVEL%"
echo.
if "%RESULT%"=="0" (
  echo AMBAR VARIABLE_HIL session ended cleanly.
) else (
  echo AMBAR VARIABLE_HIL stopped with exit code %RESULT%.
  echo Review the terminal safety message and latest result snapshot before restarting.
)
pause
exit /b %RESULT%
