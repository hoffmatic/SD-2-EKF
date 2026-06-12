param(
    [switch]$SkipBuild
)

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
        & ".\scripts\run_rocketpy_sim.ps1" -SkipBuild
    } else {
        & ".\scripts\run_rocketpy_sim.ps1"
    }
} finally {
    Pop-Location
}
