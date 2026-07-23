@echo off
setlocal
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\run_continuous_hil.ps1" %*
set "RESULT=%ERRORLEVEL%"
echo.
if "%RESULT%"=="0" (
  echo Continuous HIL session ended cleanly.
) else (
  echo Continuous HIL session stopped with exit code %RESULT%.
  echo Review the session verdict, latest dashboard snapshot, and terminal message before restarting.
)
pause
exit /b %RESULT%
