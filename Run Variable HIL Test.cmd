@echo off
setlocal
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\run_variable_hil.ps1" %*
exit /b %ERRORLEVEL%
