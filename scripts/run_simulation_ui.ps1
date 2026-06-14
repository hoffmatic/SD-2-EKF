param(
    [int]$Port = 8765
)

<#
Architecture role: human-facing UI launcher.
This file only locates Python and starts simulation_ui_server.py. The server
serves ui/ and delegates test execution to the same PowerShell scripts used at
the command line, so browser and terminal runs exercise identical programs.
#>

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..")
$pythonArguments = @()

$pythonCommand = Get-Command python -ErrorAction SilentlyContinue
if ($pythonCommand) {
    $python = $pythonCommand.Source
} else {
    $pythonLauncher = Get-Command py -ErrorAction SilentlyContinue
    if (-not $pythonLauncher) {
        throw "Python was not found. Install Python 3, then run this script again."
    }
    $python = $pythonLauncher.Source
    $pythonArguments = @("-3")
}

Push-Location $repoRoot
try {
    Write-Host "Starting AMBAR Simulation Console..."
    Write-Host "The server will print the local URL below. Keep this window open while using the UI."
    & $python @pythonArguments ".\scripts\simulation_ui_server.py" "--port" $Port
} finally {
    Pop-Location
}
