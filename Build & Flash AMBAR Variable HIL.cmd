@echo off
setlocal
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\build_flash_stm32.ps1" -Profile VariableHil %*
set "RESULT=%ERRORLEVEL%"
echo.
if "%RESULT%"=="0" (
  echo AMBAR VARIABLE_HIL was built, flashed, verified, and identified over USB.
  echo Motor motion was not commanded by this build and flash step.
  echo Before a run: manually close, declare software HOME, and verify the full config readback.
  echo XACTUAL is internal TMC ramp state, not endpoint or encoder feedback.
) else (
  echo AMBAR VARIABLE_HIL build/flash stopped with exit code %RESULT%.
  echo The causal runner will refuse to arm until the profile and config checks pass.
)
pause
exit /b %RESULT%
