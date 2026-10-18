param([switch]$Demo, [int]$Port = 8765, [switch]$NoBrowser)
$ErrorActionPreference = 'Stop'
if ($Port -lt 1024 -or $Port -gt 65535) { throw 'Port must be 1024..65535.' }
$taskRoot = $PSScriptRoot
$taskPython = Join-Path $taskRoot '.hardware-venv\Scripts\python.exe'
if (-not (Test-Path -LiteralPath $taskPython)) {
    if (-not $Demo) { throw 'Run Setup-RoboBuddy-Hardware.ps1 once before starting the real helper.' }
    $taskPython = (Get-Command python -ErrorAction Stop).Source
}
$baseUrl = "http://127.0.0.1:$Port"
$mode = if ($Demo) { 'demo' } else { 'hardware' }
$alreadyRunning = $false
try {
    $existing = Invoke-RestMethod -Uri "$baseUrl/api/health" -TimeoutSec 2
    if ($existing.apiVersion -ne 'robobuddy.so101.hardware.v1' -or $existing.mode -ne $mode) {
        throw "Port $Port belongs to another service or helper mode. Choose a different port."
    }
    $alreadyRunning = $true
} catch {
    if ($_.Exception.Message -like 'Port * belongs*') { throw }
}
if (-not $alreadyRunning) {
    if (-not $Demo) {
        Push-Location -LiteralPath $taskRoot
        try {
            & $taskPython -m hardware_bridge.check_driver
            if ($LASTEXITCODE -ne 0) { throw 'Run Setup-RoboBuddy-Hardware.ps1 to repair hardware dependencies.' }
        } finally { Pop-Location }
    }
    $logRoot = Join-Path $taskRoot '.hardware-logs'
    New-Item -ItemType Directory -Path $logRoot -Force | Out-Null
    $taskArguments = @('-m', 'hardware_bridge.server', '--port', "$Port")
    if ($Demo) { $taskArguments += '--demo' }
    $helperProcess = Start-Process -FilePath $taskPython -ArgumentList $taskArguments -WorkingDirectory $taskRoot -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logRoot "helper-$Port.out.log") -RedirectStandardError (Join-Path $logRoot "helper-$Port.err.log")
    $deadline = (Get-Date).AddSeconds(30)
    $ready = $false
    while ((Get-Date) -lt $deadline -and -not $helperProcess.HasExited) {
        try {
            $health = Invoke-RestMethod -Uri "$baseUrl/api/health" -TimeoutSec 1
            if ($health.apiVersion -eq 'robobuddy.so101.hardware.v1' -and $health.mode -eq $mode) { $ready = $true; break }
        } catch {}
        Start-Sleep -Milliseconds 200
    }
    if (-not $ready) { throw "Helper startup failed. Read .hardware-logs/helper-$Port.err.log. No USB connection was attempted." }
}
if (-not $NoBrowser) { Start-Process "$baseUrl/hardware.html" }
Write-Host "Panel ready at $baseUrl/hardware.html. No serial port was opened."

