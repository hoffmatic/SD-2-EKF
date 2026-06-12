param(
    [int]$Port = 8765
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..")
$bundledPython = Join-Path $env:USERPROFILE ".cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
$pythonArguments = @()

if (Test-Path $bundledPython) {
    $python = $bundledPython
} else {
    $pythonCommand = Get-Command python -ErrorAction SilentlyContinue
    if ($pythonCommand) {
        $python = $pythonCommand.Source
    } else {
        $pythonLauncher = Get-Command py -ErrorAction SilentlyContinue
        if (-not $pythonLauncher) {
            throw "Python was not found. Install Python 3 or run this UI from Codex Desktop."
        }
        $python = $pythonLauncher.Source
        $pythonArguments = @("-3")
    }
}

Push-Location $repoRoot
try {
    Write-Host "Starting AMBAR Simulation Console..."
    Write-Host "The server will print the local URL below. Keep this window open while using the UI."
    & $python @pythonArguments ".\scripts\simulation_ui_server.py" "--port" $Port
} finally {
    Pop-Location
}
