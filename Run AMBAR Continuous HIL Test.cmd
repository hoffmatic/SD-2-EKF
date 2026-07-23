@echo off
setlocal
call "%~dp0Run Continuous HIL Test.cmd" %*
exit /b %ERRORLEVEL%
