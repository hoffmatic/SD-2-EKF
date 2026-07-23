@echo off
setlocal
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\build_flash_stm32.ps1" -Profile ContinuousHil %*
set "RESULT=%ERRORLEVEL%"
echo.
if "%RESULT%"=="0" (
  echo AMBAR CONTINUOUS_HIL was built, flashed, verified, and identified over USB.
  echo Motor motion was not commanded. Manually close the mechanism before software HOME.
  echo XACTUAL is internal TMC ramp state, not endpoint or encoder feedback.
) else (
  echo AMBAR CONTINUOUS_HIL build/flash stopped with exit code %RESULT%.
  echo The continuous supervisor will not be safe to start until this succeeds.
)
pause
exit /b %RESULT%
