param([switch]$ReuseInstalledPython)
$ErrorActionPreference = 'Stop'
$taskRoot = $PSScriptRoot
$venvRoot = Join-Path $taskRoot '.hardware-venv'
$venvPython = Join-Path $venvRoot 'Scripts\python.exe'
if (-not (Test-Path -LiteralPath $venvPython)) {
    $taskPython = $null
    if (Get-Command py -ErrorAction SilentlyContinue) {
        $candidate = & py -3.12 -c "import sys; print(sys.executable)" 2>$null
        if ($LASTEXITCODE -eq 0) { $taskPython = $candidate.Trim() }
    }
    if (-not $taskPython) {
        $taskPython = (Get-Command python -ErrorAction Stop).Source
        & $taskPython -c "import sys; raise SystemExit(0 if sys.version_info >= (3,12) else 1)"
        if ($LASTEXITCODE -ne 0) { throw 'Install Python 3.12 or later from python.org, then rerun this setup.' }
    }
    if ($ReuseInstalledPython) { & $taskPython -m venv --system-site-packages $venvRoot }
    else { & $taskPython -m venv $venvRoot }
    if ($LASTEXITCODE -ne 0) { throw 'The hardware environment could not be created.' }
}
& $venvPython -m pip install -r (Join-Path $taskRoot 'hardware_bridge\requirements.txt')
if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed. No USB connection was attempted.' }
Push-Location -LiteralPath $taskRoot
try {
    & $venvPython -m hardware_bridge.check_driver
    if ($LASTEXITCODE -ne 0) { throw 'Hardware driver check failed.' }
} finally { Pop-Location }
Write-Host 'Setup complete. Start-RoboBuddy-Hardware.cmd opens the panel without connecting USB.'

