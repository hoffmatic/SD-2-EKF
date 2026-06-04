param(
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..")
$buildDir = Join-Path $repoRoot "build"

function Find-CppCompiler {
    if ($env:CXX -and (Test-Path $env:CXX)) {
        return $env:CXX
    }

    foreach ($compilerName in @("g++", "clang++", "c++")) {
        $namedCompiler = Get-Command $compilerName -ErrorAction SilentlyContinue
        if ($namedCompiler) {
            return $namedCompiler.Source
        }
    }

    $wingetRoot = Join-Path $env:LOCALAPPDATA "Microsoft\WinGet\Packages\MartinStorsjo.LLVM-MinGW.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe"
    if (Test-Path $wingetRoot) {
        $mingwCompiler = Get-ChildItem -Path $wingetRoot -Recurse -Filter "x86_64-w64-mingw32-g++.exe" |
            Sort-Object FullName -Descending |
            Select-Object -First 1
        if ($mingwCompiler) {
            return $mingwCompiler.FullName
        }
    }

    throw "No C++ compiler found. Install LLVM-MinGW, MSYS2, or another g++/clang++ compiler, then run this again."
}

function Build-Program {
    param(
        [string]$Compiler,
        [string]$Output,
        [string[]]$Sources
    )

    $arguments = @(
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-Wpedantic",
        "-static",
        "-I",
        "include"
    ) + $Sources + @("-o", $Output)

    Write-Host "Building $Output"
    & $Compiler @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed for $Output."
    }
}

function Run-Program {
    param([string]$Path)

    Write-Host ""
    Write-Host "============================================================"
    Write-Host "Running $(Split-Path -Leaf $Path)"
    Write-Host "============================================================"
    & $Path
    if ($LASTEXITCODE -ne 0) {
        throw "Program failed: $Path."
    }
}

Push-Location $repoRoot
try {
    New-Item -ItemType Directory -Force -Path $buildDir | Out-Null

    if (-not $SkipBuild) {
        $compiler = Find-CppCompiler
        Write-Host "Using compiler: $compiler"

        Build-Program $compiler "build\rocket_airbrake_ekf.exe" @("src\ambar_airbrake.cpp", "src\main.cpp")
        Build-Program $compiler "build\sim_flight_sandbox.exe" @("src\ambar_airbrake.cpp", "sim\flight_sandbox.cpp")
        Build-Program $compiler "build\sim_electronics_sandbox.exe" @("sim\electronics_sandbox.cpp")
        Build-Program $compiler "build\sim_actuator_sandbox.exe" @("sim\actuator_sandbox.cpp")
    }

    Run-Program "build\rocket_airbrake_ekf.exe"
    Run-Program "build\sim_flight_sandbox.exe"
    Run-Program "build\sim_electronics_sandbox.exe"
    Run-Program "build\sim_actuator_sandbox.exe"

    Write-Host ""
    Write-Host "Done. Read docs\beginner_quick_start.md and docs\simulation_sandboxes.md for what the results mean."
} finally {
    Pop-Location
}
