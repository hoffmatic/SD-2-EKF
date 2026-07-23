@echo off
setlocal
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\build_flash_stm32.ps1" -Profile Normal %*
set "RESULT=%ERRORLEVEL%"
echo.
if "%RESULT%"=="0" (
  echo AMBAR NORMAL was built, flashed, verified, and identified over USB.
) else (
  echo AMBAR NORMAL build/flash stopped with exit code %RESULT%.
  echo No test session should be started until the failure is corrected.
)
pause
exit /b %RESULT%
