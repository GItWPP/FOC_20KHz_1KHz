@echo off
setlocal
title FPGA FOC Motor Console
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0start_speed_monitor.ps1" %*
if errorlevel 1 pause
endlocal
