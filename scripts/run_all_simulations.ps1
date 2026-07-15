param(
    [switch]$SkipBuild,
    [string]$OverridesPath
)

<#
Architecture role: top-level command-line test orchestrator.
It runs the fast native suites first, then the RocketPy closed-loop suite. The
browser UI's Run All action reaches the same path through simulation_ui_server.py.
The several-minute production Monte Carlo campaign is intentionally separate.
#>

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..")

Push-Location $repoRoot
try {
    if ($SkipBuild) {
        & ".\scripts\run_sandboxes.ps1" -SkipBuild
    } else {
        & ".\scripts\run_sandboxes.ps1"
    }

    Write-Host ""
    Write-Host "============================================================"
    Write-Host "Running sim_rocketpy_physics"
    Write-Host "============================================================"
    if ($SkipBuild) {
        & ".\scripts\run_rocketpy_sim.ps1" -SkipBuild -OverridesPath $OverridesPath
    } else {
        & ".\scripts\run_rocketpy_sim.ps1" -OverridesPath $OverridesPath
    }
} finally {
    Pop-Location
}
