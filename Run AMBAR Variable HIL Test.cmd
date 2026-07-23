@echo off
setlocal
call "%~dp0Run Variable HIL Test.cmd" %*
exit /b %ERRORLEVEL%
