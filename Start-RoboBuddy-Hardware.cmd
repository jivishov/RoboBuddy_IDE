@echo off
powershell.exe -NoProfile -File "%~dp0Start-RoboBuddy-Hardware.ps1" %*
if errorlevel 1 pause

