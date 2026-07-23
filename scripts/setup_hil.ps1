param()

<#
Architecture role: create the Python environment used by continuous RocketPy
hardware-in-the-loop testing. The finite SIL workflow remains valid; this adds
only the pinned serial dependency required by the sole COM-port owner.
#>

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..")
$venvPython = Join-Path $repoRoot ".venv\Scripts\python.exe"

if (-not (Test-Path -LiteralPath $venvPython)) {
    & (Join-Path $scriptDir "setup_rocketpy.ps1")
    if ($LASTEXITCODE -ne 0) {
        throw "The RocketPy Python environment could not be created."
    }
}

Write-Host "Installing pinned AMBAR continuous-HIL dependencies..."
& $venvPython -m pip install --disable-pip-version-check -r (Join-Path $repoRoot "requirements-hil.txt")
if ($LASTEXITCODE -ne 0) {
    throw "Installation of the AMBAR continuous-HIL dependencies failed."
}

& $venvPython -c "import importlib.metadata as m, sqlite3; assert m.version('rocketpy') == '1.12.1'; assert m.version('pyserial') == '3.5'; print('RocketPy', m.version('rocketpy'), '| PySerial', m.version('pyserial'), '| SQLite', sqlite3.sqlite_version)"
if ($LASTEXITCODE -ne 0) {
    throw "The AMBAR continuous-HIL Python environment failed its import/version check."
}

Write-Host "AMBAR continuous-HIL environment ready."
