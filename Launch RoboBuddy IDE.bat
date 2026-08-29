@echo off
setlocal

cd /d "%~dp0"
set "PORT=8765"
set "APP_URL=http://127.0.0.1:%PORT%/"

rem Reuse an already-running local server when the launcher is clicked again.
powershell -NoProfile -Command "try { Invoke-WebRequest -UseBasicParsing -Uri '%APP_URL%' -TimeoutSec 1 | Out-Null; exit 0 } catch { exit 1 }" >nul 2>&1
if not errorlevel 1 goto :openApp

where py >nul 2>&1
if not errorlevel 1 (
  start "RoboBuddy IDE Server" /min cmd /d /c "cd /d ""%~dp0"" && py -3 -m http.server %PORT% --bind 127.0.0.1"
  goto :waitForServer
)

where python >nul 2>&1
if not errorlevel 1 (
  start "RoboBuddy IDE Server" /min cmd /d /c "cd /d ""%~dp0"" && python -m http.server %PORT% --bind 127.0.0.1"
  goto :waitForServer
)

echo Python was not found. Install Python 3, then run this launcher again.
pause
exit /b 1

:waitForServer
for /L %%I in (1,1,10) do (
  powershell -NoProfile -Command "try { Invoke-WebRequest -UseBasicParsing -Uri '%APP_URL%' -TimeoutSec 1 | Out-Null; exit 0 } catch { exit 1 }" >nul 2>&1
  if not errorlevel 1 goto :openApp
  powershell -NoProfile -Command "Start-Sleep -Seconds 1"
)

:openApp
start "" "%APP_URL%"
endlocal
