param()

<#
Architecture role: one-time local dependency setup for the physics simulation.
Only RocketPy and its pinned Python dependencies belong in .venv; the C++ core
and the standard-library UI server do not depend on this environment.
#>

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..")
$venvPython = Join-Path $repoRoot ".venv\Scripts\python.exe"

if (-not (Test-Path $venvPython)) {
    if (Get-Command python -ErrorAction SilentlyContinue) {
        $basePython = (Get-Command python).Source
        $baseArguments = @()
    } elseif (Get-Command py -ErrorAction SilentlyContinue) {
        $basePython = (Get-Command py).Source
        $baseArguments = @("-3")
    } else {
        throw "Python 3 was not found. Install Python 3.10 or newer, then rerun this script."
    }

    Write-Host "Creating the local RocketPy environment..."
    & $basePython @baseArguments -m venv (Join-Path $repoRoot ".venv")
}

Write-Host "Installing the pinned simulation dependencies..."
& $venvPython -m pip install --disable-pip-version-check -r (Join-Path $repoRoot "requirements-simulation.txt")
Write-Host "RocketPy environment ready."
