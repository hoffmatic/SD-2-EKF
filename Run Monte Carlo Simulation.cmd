@echo off
setlocal
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\run_monte_carlo.ps1" %*
set "RESULT=%ERRORLEVEL%"
echo.
if "%RESULT%"=="0" (
  echo Campaign acceptance checks passed.
) else if "%RESULT%"=="1" (
  echo Campaign completed with one or more failed acceptance checks.
  echo Review the exact results directory printed above.
) else (
  echo Campaign execution failed before a complete result was produced.
  echo Review the error and any checkpoint directory printed above.
)
pause
exit /b %RESULT%
