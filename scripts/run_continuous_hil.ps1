param(
    [string]$Port,
    [string]$Resume,
    [switch]$RebuildDatabase,
    [string]$ResultsRoot = (Join-Path $env:LOCALAPPDATA "AMBAR\TestRuns"),
    [string]$MirrorRoot = (Join-Path $HOME "OneDrive\Desktop\AMBAR_Continuous_Test_Results\latest"),
    [long]$MasterSeed,
    [int]$BatchSize = 50,
    [int]$BaselineInterval = 10,
    [double]$DwellSeconds = 30.0,
    [int]$MaxCycles = 0,
    [int]$DashboardPort = 52101,
    [int]$DashboardUdpPort = 52102,
    [switch]$AcceptCurrentPositionHome,
    [switch]$NoBrowser
)

<#
Architecture role: one-command continuous-HIL operator launcher. The Python
supervisor remains the sole COM-port owner; this wrapper starts the read-only
dashboard, then keeps the safety-critical supervisor in the foreground.
#>

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path (Join-Path $scriptDir "..")).Path
$python = Join-Path $repoRoot ".venv\Scripts\python.exe"
$supervisor = Join-Path $scriptDir "run_continuous_hil.py"
$dashboard = Join-Path $scriptDir "continuous_hil_dashboard.py"

if ($RebuildDatabase -and -not $Resume) {
    throw "-RebuildDatabase requires -Resume <session-directory>."
}

Write-Warning "This setup has no HOME/FULL switches and no independent position encoder."
Write-Host "Before continuing, manually place the mechanism fully closed."
Write-Host "CMD_HOME will set the current TMC5240 ramp position to XACTUAL=0 without seeking."
Write-Host "The 0 -> 153600 -> 0 evidence is internal target/XACTUAL state, not proof of physical travel."
if (-not $AcceptCurrentPositionHome) {
    $confirmation = Read-Host "Type CLOSED to confirm the mechanism is manually fully closed"
    if ($confirmation.Trim() -cne "CLOSED") {
        throw "Software HOME was not authorized. No dashboard, serial port, or motor workflow was started."
    }
    $AcceptCurrentPositionHome = $true
}

& (Join-Path $scriptDir "setup_hil.ps1")
if ($LASTEXITCODE -ne 0) {
    throw "Continuous-HIL dependency setup failed."
}
$bridge = & (Join-Path $scriptDir "build_stm32_controller_bridge.ps1")
if ($LASTEXITCODE -ne 0 -or -not (Test-Path -LiteralPath $bridge -PathType Leaf)) {
    throw "The production STM32 controller bridge could not be built."
}
if (-not (Test-Path -LiteralPath $supervisor -PathType Leaf)) {
    throw "Continuous-HIL supervisor is missing: $supervisor"
}
if (-not (Test-Path -LiteralPath $dashboard -PathType Leaf)) {
    throw "Continuous-HIL dashboard server is missing: $dashboard"
}

$ResultsRoot = [IO.Path]::GetFullPath($ResultsRoot)
$MirrorRoot = [IO.Path]::GetFullPath($MirrorRoot)
New-Item -ItemType Directory -Path $ResultsRoot -Force | Out-Null

if ($Resume) {
    $sessionDir = [IO.Path]::GetFullPath($Resume)
    if (-not (Test-Path -LiteralPath $sessionDir -PathType Container)) {
        throw "Resume session directory does not exist: $sessionDir"
    }
    $sessionId = Split-Path -Leaf $sessionDir
} else {
    $timestamp = [DateTime]::UtcNow.ToString("yyyyMMddTHHmmssZ")
    $suffix = [Guid]::NewGuid().ToString("N").Substring(0, 8)
    $sessionId = "$timestamp-$suffix"
    $sessionDir = Join-Path $ResultsRoot $sessionId
}
$databasePath = Join-Path $sessionDir "campaign.sqlite3"
$dashboardUrl = "http://127.0.0.1:$DashboardPort"

function Quote-ProcessArgument {
    param([string]$Value)
    return '"' + $Value.Replace('"', '\"') + '"'
}

$dashboardArgumentValues = @(
    $dashboard,
    "--db", $databasePath,
    "--results-root", $ResultsRoot,
    "--mirror-root", $MirrorRoot,
    "--port", [string]$DashboardPort,
    "--udp-port", [string]$DashboardUdpPort,
    "--checkpoint-runs", "10"
)
$dashboardArgumentLine = ($dashboardArgumentValues | ForEach-Object { Quote-ProcessArgument ([string]$_) }) -join " "
$dashboardProcess = Start-Process -FilePath $python -ArgumentList $dashboardArgumentLine -PassThru -WindowStyle Hidden

try {
    $healthy = $false
    $deadline = [DateTime]::UtcNow.AddSeconds(12)
    while ([DateTime]::UtcNow -lt $deadline) {
        if ($dashboardProcess.HasExited) {
            throw "The dashboard process exited during startup with code $($dashboardProcess.ExitCode)."
        }
        try {
            $health = Invoke-RestMethod -Uri "$dashboardUrl/api/health" -TimeoutSec 1
            if ($health.ok) {
                $healthy = $true
                break
            }
        } catch {
            Start-Sleep -Milliseconds 250
        }
    }
    if (-not $healthy) {
        throw "The dashboard did not become healthy at $dashboardUrl."
    }

    Write-Host "AMBAR continuous-HIL session: $sessionId"
    Write-Host "Results directory: $sessionDir"
    Write-Host "Dashboard: $dashboardUrl"
    if (-not $NoBrowser) {
        Start-Process $dashboardUrl
    }

    $supervisorArguments = @(
        $supervisor,
        "--session-id", $sessionId,
        "--results-root", $ResultsRoot,
        "--mirror-root", $MirrorRoot,
        "--batch-size", [string]$BatchSize,
        "--baseline-interval", [string]$BaselineInterval,
        "--dwell-s", [string]$DwellSeconds,
        "--gui-udp-port", [string]$DashboardUdpPort,
        "--max-cycles", [string]$MaxCycles,
        "--bridge", $bridge,
        "--accept-current-position-home"
    )
    if ($Port) {
        $supervisorArguments += @("--port", $Port)
    }
    if ($Resume) {
        $supervisorArguments += @("--resume", $sessionDir)
    }
    if ($RebuildDatabase) {
        $supervisorArguments += "--rebuild-database"
    }
    if ($PSBoundParameters.ContainsKey("MasterSeed")) {
        $supervisorArguments += @("--master-seed", [string]$MasterSeed)
    }

    Push-Location $repoRoot
    try {
        & $python @supervisorArguments
        $supervisorExitCode = $LASTEXITCODE
    } finally {
        Pop-Location
    }
} finally {
    if (Test-Path -LiteralPath $databasePath) {
        $snapshotArguments = @(
            $dashboard,
            "--db", $databasePath,
            "--results-root", $ResultsRoot,
            "--mirror-root", $MirrorRoot,
            "--snapshot-latest",
            "--snapshot-only"
        )
        & $python @snapshotArguments
        if ($LASTEXITCODE -ne 0) {
            Write-Warning "The final portable dashboard snapshot could not be generated."
        }
    }
    if ($dashboardProcess -and -not $dashboardProcess.HasExited) {
        Stop-Process -Id $dashboardProcess.Id -Force
    }
}

if ($null -eq $supervisorExitCode) {
    exit 2
}
exit $supervisorExitCode
