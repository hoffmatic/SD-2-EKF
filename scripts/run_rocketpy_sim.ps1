param(
    [switch]$SkipBuild,
    [string]$OverridesPath
)

<#
Architecture role: RocketPy adapter launcher.
It ensures the Python environment exists, builds sim/controller_bridge.cpp, and
passes that executable to sim/rocketpy/run_rocketpy_sim.py. The Python model and
C++ controller therefore remain separate while running in one closed loop.
#>

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..")
$venvPython = Join-Path $repoRoot ".venv\Scripts\python.exe"
$bridge = Join-Path $repoRoot "build\ambar_controller_bridge.exe"

if (-not (Test-Path $venvPython)) {
    & (Join-Path $scriptDir "setup_rocketpy.ps1")
}

Push-Location $repoRoot
try {
    if (-not $SkipBuild -or -not (Test-Path $bridge)) {
        $compiler = Get-Command g++ -ErrorAction SilentlyContinue
        if (-not $compiler) {
            $compiler = Get-Command clang++ -ErrorAction SilentlyContinue
        }
        if (-not $compiler) {
            $wingetRoot = Join-Path $env:LOCALAPPDATA "Microsoft\WinGet\Packages\MartinStorsjo.LLVM-MinGW.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe"
            $compiler = Get-ChildItem -Path $wingetRoot -Recurse -Filter "x86_64-w64-mingw32-g++.exe" -ErrorAction SilentlyContinue |
                Sort-Object FullName -Descending | Select-Object -First 1
        }
        if (-not $compiler) {
            throw "No C++ compiler found for the RocketPy controller bridge."
        }
        $compilerPath = if ($compiler.Source) { $compiler.Source } else { $compiler.FullName }
        New-Item -ItemType Directory -Force -Path ".\build" | Out-Null
        & $compilerPath -std=c++17 -Wall -Wextra -Wpedantic -static -I include src\ambar_airbrake.cpp sim\controller_bridge.cpp -o $bridge
        if ($LASTEXITCODE -ne 0) { throw "Controller bridge build failed." }
    }

    $simulationArguments = @(".\sim\rocketpy\run_rocketpy_sim.py", "--bridge", $bridge)
    if ($OverridesPath) {
        $resolvedOverrides = Resolve-Path $OverridesPath
        $simulationArguments += @("--overrides", $resolvedOverrides)
    }
    & $venvPython @simulationArguments
    if ($LASTEXITCODE -ne 0) { throw "RocketPy simulation failed." }
} finally {
    Pop-Location
}
