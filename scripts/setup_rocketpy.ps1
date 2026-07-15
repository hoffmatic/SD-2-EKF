param()

<#
Architecture role: one-time local dependency setup for the physics simulation.
Only RocketPy and its pinned Python dependencies belong in .venv; the production
C bridge and the standard-library UI server do not depend on this environment.
#>

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $scriptDir "..")
$venvPython = Join-Path $repoRoot ".venv\Scripts\python.exe"

if (-not (Test-Path $venvPython)) {
    $candidates = @()
    $pyLauncher = Get-Command py -ErrorAction SilentlyContinue
    if ($pyLauncher) {
        $candidates += ,@($pyLauncher.Source, @("-3"))
    }
    $pythonCommand = Get-Command python -ErrorAction SilentlyContinue
    if ($pythonCommand) {
        $candidates += ,@($pythonCommand.Source, @())
    }
    $codexPython = Join-Path $env:USERPROFILE ".cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
    if (Test-Path $codexPython) {
        $candidates += ,@($codexPython, @())
    }

    $basePython = $null
    $baseArguments = @()
    foreach ($candidate in $candidates) {
        $candidatePath = $candidate[0]
        $candidateArguments = $candidate[1]
        $candidateExitCode = 1
        try {
            & $candidatePath @candidateArguments -c "import sys; assert sys.version_info >= (3, 10)" 2>$null
            $candidateExitCode = $LASTEXITCODE
        } catch {
            # A Windows Store execution alias can exist while no real Python is
            # installed. Treat that candidate as unavailable and continue to
            # the next real interpreter, including the bundled Codex runtime.
            $candidateExitCode = 1
        }
        if ($candidateExitCode -eq 0) {
            $basePython = $candidatePath
            $baseArguments = $candidateArguments
            break
        }
    }

    if (-not $basePython) {
        throw "Python 3 was not found. Install Python 3.10 or newer, then rerun this script."
    }

    Write-Host "Creating the local RocketPy environment with $basePython..."
    & $basePython @baseArguments -m venv (Join-Path $repoRoot ".venv")
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path $venvPython)) {
        throw "Python was found, but creation of the local .venv failed."
    }
}

Write-Host "Installing the pinned simulation dependencies..."
& $venvPython -m pip install --disable-pip-version-check -r (Join-Path $repoRoot "requirements-simulation.txt")
if ($LASTEXITCODE -ne 0) {
    throw "Installation of the pinned simulation dependencies failed."
}
Write-Host "RocketPy environment ready."
