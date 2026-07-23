param(
    [switch]$SkipBuild
)

<#
Architecture role: build the persistent production-C simulation bridge.
Both the one-run RocketPy adapter and the Monte Carlo campaign call this file so
they cannot accidentally compile different controller implementations.
#>

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..")
$bridge = Join-Path $repoRoot "build\ambar_stm32_controller_bridge.exe"

if ($SkipBuild) {
    Write-Host "Rebuilding the STM32-C bridge even with -SkipBuild so campaign provenance cannot reference a stale binary."
}

# This target is small and safety-relevant, so it is always rebuilt.  The
# native C++ suites may honor -SkipBuild, but a Monte Carlo row must never hash
# today's C sources while executing yesterday's bridge executable.
$compiler = Get-Command gcc -ErrorAction SilentlyContinue
if (-not $compiler) {
    $compiler = Get-Command clang -ErrorAction SilentlyContinue
}
if (-not $compiler) {
    $wingetRoot = Join-Path $env:LOCALAPPDATA "Microsoft\WinGet\Packages\MartinStorsjo.LLVM-MinGW.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe"
    $compiler = Get-ChildItem -Path $wingetRoot -Recurse -Filter "gcc.exe" -ErrorAction SilentlyContinue |
        Sort-Object FullName -Descending |
        Select-Object -First 1
}
if (-not $compiler) {
    throw "No C compiler found for the STM32 simulation bridge."
}

$compilerPath = if ($compiler.Source) { $compiler.Source } else { $compiler.FullName }
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $bridge) | Out-Null

Push-Location $repoRoot
try {
    Write-Host "Building the production STM32 controller bridge with $compilerPath"
    & $compilerPath `
        -std=c11 `
        -Wall `
        -Wextra `
        -Wpedantic `
        -static `
        -I firmware\stm32_airbrake_pcb\Core\Inc `
        firmware\stm32_airbrake_pcb\Core\Src\ambar_ekf.c `
        firmware\stm32_airbrake_pcb\Core\Src\ambar_flight.c `
        sim\stm32_controller_bridge.c `
        -lm `
        -o $bridge
    if ($LASTEXITCODE -ne 0) {
        throw "STM32 controller bridge build failed."
    }
    # Let the native process finish before selecting its first output line.
    # Piping it directly into Select-Object -First can close the pipe early and
    # leave LASTEXITCODE at -1 even though the compiler and bridge build passed.
    $compilerVersionOutput = @(& $compilerPath --version)
    if ($LASTEXITCODE -ne 0) {
        throw "Unable to read the STM32 controller bridge compiler version."
    }
    $compilerVersion = $compilerVersionOutput | Select-Object -First 1
    $buildMetadata = [System.IO.Path]::ChangeExtension($bridge, ".build.txt")
    @(
        "compiler=$compilerPath"
        "compiler_version=$compilerVersion"
        "language_standard=c11"
        "flags=-Wall -Wextra -Wpedantic -static -lm"
        "sources=ambar_ekf.c;ambar_flight.c;stm32_controller_bridge.c"
    ) | Set-Content -LiteralPath $buildMetadata -Encoding UTF8
} finally {
    Pop-Location
}
# Emit only the resolved executable path on the success stream. Callers use it
# as the --bridge argument while build narration remains on the host stream.
Write-Output $bridge
