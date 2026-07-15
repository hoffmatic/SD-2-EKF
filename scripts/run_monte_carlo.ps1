param(
    [int]$BaselineRuns = 2,
    [int]$RandomRuns = 50,
    [int]$Seed = 20260715,
    [string]$OutputDirectory,
    [string]$BaseOverridesPath,
    [string]$StudyConfigPath,
    [switch]$SkipBuild
)

<#
Architecture role: one-command production-controller robustness campaign.
The default performs two identical baseline runs followed by 50 reproducible
Latin-hypercube trials and writes per-run/aggregate CSV files.  It never opens
the STM32 COM port and never moves the physical motor.
#>

$ErrorActionPreference = "Stop"
trap {
    Write-Error $_
    exit 2
}
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..")
$venvPython = Join-Path $repoRoot ".venv\Scripts\python.exe"

if (-not (Test-Path $venvPython)) {
    & (Join-Path $scriptDir "setup_rocketpy.ps1")
}

$bridgeArguments = @{}
if ($SkipBuild) { $bridgeArguments.SkipBuild = $true }
$bridge = & (Join-Path $scriptDir "build_stm32_controller_bridge.ps1") @bridgeArguments

$arguments = @(
    (Join-Path $repoRoot "sim\rocketpy\run_monte_carlo.py"),
    "--bridge", $bridge,
    "--baseline-runs", $BaselineRuns,
    "--random-runs", $RandomRuns,
    "--seed", $Seed
)
if ($OutputDirectory) {
    $arguments += @("--output-dir", $OutputDirectory)
}
if ($BaseOverridesPath) {
    $arguments += @("--base-overrides", (Resolve-Path $BaseOverridesPath))
}
if ($StudyConfigPath) {
    $arguments += @("--study-config", (Resolve-Path $StudyConfigPath))
}

Push-Location $repoRoot
try {
    & $venvPython @arguments
    exit $LASTEXITCODE
} finally {
    Pop-Location
}
