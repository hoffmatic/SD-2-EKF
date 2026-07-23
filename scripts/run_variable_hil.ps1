param(
    [string]$Port,
    [switch]$Hardware,
    [switch]$AllowActuatorMotion,
    [switch]$AcceptCurrentPositionHome,
    [int]$Cycles = 25,
    [string]$CaseList = "low,nominal,high",
    [double]$DwellSeconds = 30.0,
    [double]$MaxTimeSeconds = 30.0,
    [string]$ResultsRoot = (Join-Path $env:LOCALAPPDATA "AMBAR\VariableHilRuns"),
    [string]$MirrorRoot = (Join-Path ([Environment]::GetFolderPath("Desktop")) "AMBAR_Variable_HIL_Results\latest"),
    [int]$DashboardPort = 52111,
    [int]$DashboardUdpPort = 52112,
    [switch]$NoBrowser
)

<#
Architecture role: safe one-command launcher for the causally coupled 50 Hz
VARIABLE_HIL workflow. The Python runner remains the sole COM-port owner. The
dashboard is read-only and receives the separate VARIABLE_HIL SQLite database.

Without -Hardware this wrapper runs only the deterministic fake transport. It
does not enumerate or open serial ports. Hardware mode requires all explicit
motion gates before dependency setup, dashboard startup, or runner startup.
#>

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path (Join-Path $scriptDir "..")).Path
$python = Join-Path $repoRoot ".venv\Scripts\python.exe"
$runner = Join-Path $scriptDir "run_variable_hil.py"
$dashboard = Join-Path $scriptDir "continuous_hil_dashboard.py"
$variableConfig = Join-Path $repoRoot "sim\rocketpy\variable_hil_m5_config.json"

if ($Cycles -le 0) {
    throw "-Cycles must be positive."
}
if ($DwellSeconds -lt 0.0) {
    throw "-DwellSeconds must be nonnegative."
}
if ($MaxTimeSeconds -le 0.0) {
    throw "-MaxTimeSeconds must be positive."
}

if ($Hardware) {
    if (-not $AllowActuatorMotion) {
        throw "Hardware mode requires -AllowActuatorMotion. Nothing was started."
    }
    if ([string]::IsNullOrWhiteSpace($Port)) {
        throw "Hardware mode requires an explicit -Port COMx. Nothing was started."
    }

    Write-Warning "VARIABLE_HIL can move the airbrake actuator throughout every RocketPy run."
    Write-Warning "This setup has no HOME/FULL switches and no independent position encoder."
    Write-Host "Manually place the mechanism fully closed before software HOME is declared."
    Write-Host "TMC5240 XACTUAL is internal ramp-generator state, not proof of physical blade position."
    if (-not $AcceptCurrentPositionHome) {
        $confirmation = Read-Host "Type CLOSED to declare the current mechanism position fully closed"
        if ($confirmation.Trim() -cne "CLOSED") {
            throw "Software HOME was not authorized. No dashboard, serial port, or motor workflow was started."
        }
        $AcceptCurrentPositionHome = $true
    }
} elseif (
    $AllowActuatorMotion -or
    $AcceptCurrentPositionHome -or
    -not [string]::IsNullOrWhiteSpace($Port)
) {
    throw "Port and actuator-motion options require -Hardware. Nothing was started."
}

& (Join-Path $scriptDir "setup_hil.ps1")
if ($LASTEXITCODE -ne 0) {
    throw "VARIABLE_HIL dependency setup failed."
}
foreach ($requiredFile in @($python, $runner, $dashboard, $variableConfig)) {
    if (-not (Test-Path -LiteralPath $requiredFile -PathType Leaf)) {
        throw "Required VARIABLE_HIL file is missing: $requiredFile"
    }
}

$ResultsRoot = [IO.Path]::GetFullPath($ResultsRoot)
$MirrorRoot = [IO.Path]::GetFullPath($MirrorRoot)
New-Item -ItemType Directory -Path $ResultsRoot -Force | Out-Null

$timestamp = [DateTime]::UtcNow.ToString("yyyyMMddTHHmmssZ")
$suffix = [Guid]::NewGuid().ToString("N").Substring(0, 8)
$sessionId = "$timestamp-$suffix"
$sessionDir = Join-Path $ResultsRoot $sessionId
$databasePath = Join-Path $sessionDir "campaign.sqlite3"
$dashboardUrl = "http://127.0.0.1:$DashboardPort"
$dashboardProcess = $null
$runnerExitCode = $null

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
    "--checkpoint-runs", "5"
)
$dashboardArgumentLine = ($dashboardArgumentValues | ForEach-Object {
    Quote-ProcessArgument ([string]$_)
}) -join " "
$dashboardProcess = Start-Process `
    -FilePath $python `
    -ArgumentList $dashboardArgumentLine `
    -PassThru `
    -WindowStyle Hidden

try {
    $healthy = $false
    $healthDeadline = [DateTime]::UtcNow.AddSeconds(12)
    while ([DateTime]::UtcNow -lt $healthDeadline) {
        if ($dashboardProcess.HasExited) {
            throw "The dashboard exited during startup with code $($dashboardProcess.ExitCode)."
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

    Write-Host "AMBAR VARIABLE_HIL session: $sessionId"
    Write-Host "Mode: $(if ($Hardware) { 'HARDWARE' } else { 'DETERMINISTIC FAKE TRANSPORT' })"
    Write-Host "Results directory: $sessionDir"
    Write-Host "Dashboard: $dashboardUrl"
    if (-not $NoBrowser) {
        Start-Process $dashboardUrl
    }

    $runnerArguments = @(
        $runner,
        "--session-id", $sessionId,
        "--results-root", $ResultsRoot,
        "--variable-config", $variableConfig,
        "--cycles", [string]$Cycles,
        "--case-list", $CaseList,
        "--dwell-s", [string]$DwellSeconds,
        "--max-time-s", [string]$MaxTimeSeconds
    )
    if ($Hardware) {
        $runnerArguments += @(
            "--hardware",
            "--allow-actuator-motion",
            "--accept-current-position-home",
            "--port", $Port
        )
    }

    Push-Location $repoRoot
    try {
        & $python @runnerArguments
        $runnerExitCode = $LASTEXITCODE
    } finally {
        Pop-Location
    }
} finally {
    if (Test-Path -LiteralPath $databasePath -PathType Leaf) {
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
            Write-Warning "The final portable VARIABLE_HIL dashboard snapshot could not be generated."
        }
    }
    if ($dashboardProcess -and -not $dashboardProcess.HasExited) {
        Stop-Process -Id $dashboardProcess.Id -Force
    }
}

if ($null -eq $runnerExitCode) {
    exit 2
}
exit $runnerExitCode
