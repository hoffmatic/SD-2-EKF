param(
    [switch]$SkipBuild,
    [string]$OverridesPath
)

<#
Architecture role: RocketPy adapter launcher.
It ensures the Python environment exists, builds the production-C controller
bridge from the same ambar_ekf.c and ambar_flight.c files used by the STM32,
and passes that executable to sim/rocketpy/run_rocketpy_sim.py.
#>

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..")
$venvPython = Join-Path $repoRoot ".venv\Scripts\python.exe"
$bridgeBuilder = Join-Path $scriptDir "build_stm32_controller_bridge.ps1"

if (-not (Test-Path $venvPython)) {
    & (Join-Path $scriptDir "setup_rocketpy.ps1")
}

$simulationExitCode = 1
Push-Location $repoRoot
try {
    $bridgeArguments = @{}
    if ($SkipBuild) { $bridgeArguments.SkipBuild = $true }
    $bridge = & $bridgeBuilder @bridgeArguments

    $simulationArguments = @(".\sim\rocketpy\run_rocketpy_sim.py", "--bridge", $bridge)
    if ($OverridesPath) {
        $resolvedOverrides = Resolve-Path $OverridesPath
        $simulationArguments += @("--overrides", $resolvedOverrides)
    }
    & $venvPython @simulationArguments
    # A nonzero result means the simulation completed but one or more evidence
    # checks failed.  Preserve that status without printing a misleading
    # PowerShell exception/stack trace.
    $simulationExitCode = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $simulationExitCode
