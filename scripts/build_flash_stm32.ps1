param(
    [Parameter(Mandatory = $true)]
    [ValidateSet("Normal", "ContinuousHil", "VariableHil")]
    [string]$Profile,
    [switch]$BuildOnly,
    [switch]$ValidateOnly,
    [string]$CubeIdePath,
    [string]$ProgrammerPath,
    [string]$HeadlessWorkspace,
    [string]$StLinkSerial,
    [string]$PythonPath,
    [string]$SerialPort,
    [ValidateRange(1.0, 120.0)]
    [double]$UsbReconnectSeconds = 20.0,
    [ValidateRange(1.0, 30.0)]
    [double]$RuntimeListenSeconds = 3.0
)

<#
Architecture role: deterministic headless build and optional verified flash for
the explicitly identified STM32 images. It never edits firmware sources or
selects a build by an implicit IDE default.
#>

$ErrorActionPreference = "Stop"
trap {
    Write-Error $_
    exit 2
}

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path (Join-Path $scriptDir "..")).Path
$projectDir = Join-Path $repoRoot "firmware\stm32_airbrake_pcb"
$projectName = "MCU Project EKF PCB Copy"
$runtimeVerifier = Join-Path $projectDir "tools\usb_protocol\verify_firmware_profile.py"

$profileMap = @{
    Normal = @{
        Configuration = "Normal"
        ArtifactStem = "ambar_normal"
        ExpectedDefine = "AMBAR_BUILD_PROFILE_NORMAL"
    }
    ContinuousHil = @{
        Configuration = "Continuous_HIL"
        ArtifactStem = "ambar_continuous_hil"
        ExpectedDefine = "AMBAR_BUILD_PROFILE_CONTINUOUS_HIL"
    }
    VariableHil = @{
        Configuration = "Variable_HIL"
        ArtifactStem = "ambar_variable_hil"
        ExpectedDefine = "AMBAR_BUILD_PROFILE_VARIABLE_HIL"
    }
}
$selected = $profileMap[$Profile]
$configuration = $selected.Configuration
$artifactStem = $selected.ArtifactStem
$elfPath = Join-Path $projectDir "$configuration\$artifactStem.elf"

function Resolve-FirstTool {
    param(
        [string]$ExplicitPath,
        [string]$ToolName,
        [string[]]$SearchRoots
    )

    if ($ExplicitPath) {
        if (-not (Test-Path -LiteralPath $ExplicitPath -PathType Leaf)) {
            throw "$ToolName was not found at the requested path: $ExplicitPath"
        }
        return (Resolve-Path -LiteralPath $ExplicitPath).Path
    }

    $command = Get-Command $ToolName -ErrorAction SilentlyContinue
    if ($command -and $command.Source) {
        return $command.Source
    }

    $matches = foreach ($root in $SearchRoots) {
        if (Test-Path -LiteralPath $root) {
            Get-ChildItem -LiteralPath $root -Recurse -Filter $ToolName -File -ErrorAction SilentlyContinue
        }
    }
    $match = $matches | Sort-Object FullName -Descending | Select-Object -First 1
    if (-not $match) {
        throw "$ToolName was not found. Install STM32CubeIDE/CubeProgrammer or pass its explicit path."
    }
    return $match.FullName
}

function Resolve-PythonRuntime {
    param([string]$ExplicitPath)

    if ($ExplicitPath) {
        if (-not (Test-Path -LiteralPath $ExplicitPath -PathType Leaf)) {
            throw "Python was not found at the requested path: $ExplicitPath"
        }
        return (Resolve-Path -LiteralPath $ExplicitPath).Path
    }

    $repositoryPython = Join-Path $repoRoot ".venv\Scripts\python.exe"
    if (Test-Path -LiteralPath $repositoryPython -PathType Leaf) {
        return (Resolve-Path -LiteralPath $repositoryPython).Path
    }

    foreach ($commandName in @("python.exe", "py.exe")) {
        $command = Get-Command $commandName -ErrorAction SilentlyContinue
        if ($command -and $command.Source) {
            return $command.Source
        }
    }

    throw "Python was not found. Run scripts\setup_hil.ps1 or pass -PythonPath."
}

$searchRoots = @(
    "C:\ST",
    (Join-Path $env:ProgramFiles "STMicroelectronics"),
    (Join-Path ${env:ProgramFiles(x86)} "STMicroelectronics")
) | Where-Object { $_ }

$ide = Resolve-FirstTool -ExplicitPath $CubeIdePath -ToolName "stm32cubeidec.exe" -SearchRoots $searchRoots
$ideRoot = Split-Path -Parent $ide
$programmerRoots = @($ideRoot) + $searchRoots
$programmer = Resolve-FirstTool -ExplicitPath $ProgrammerPath -ToolName "STM32_Programmer_CLI.exe" -SearchRoots $programmerRoots
$sizeTool = Resolve-FirstTool -ToolName "arm-none-eabi-size.exe" -SearchRoots @($ideRoot)
$runtimePython = $null
if (-not $BuildOnly) {
    if (-not (Test-Path -LiteralPath $runtimeVerifier -PathType Leaf)) {
        throw "The read-only runtime profile verifier is missing: $runtimeVerifier"
    }
    $runtimePython = Resolve-PythonRuntime -ExplicitPath $PythonPath
    & $runtimePython -c "import serial; print('PySerial ' + serial.__version__)"
    if ($LASTEXITCODE -ne 0) {
        throw "PySerial is unavailable in $runtimePython. Run scripts\setup_hil.ps1."
    }
}

if (-not $HeadlessWorkspace) {
    $HeadlessWorkspace = Join-Path $repoRoot "build\cubeide-headless"
}
$HeadlessWorkspace = [IO.Path]::GetFullPath($HeadlessWorkspace)

$cprojectPath = Join-Path $projectDir ".cproject"
if (-not (Test-Path -LiteralPath $cprojectPath)) {
    throw "STM32CubeIDE project metadata is missing: $cprojectPath"
}
$cprojectText = Get-Content -Raw -LiteralPath $cprojectPath
if ($cprojectText -notmatch ('name="' + [regex]::Escape($configuration) + '"')) {
    throw "CubeIDE configuration '$configuration' is missing from .cproject."
}
if ($cprojectText -notmatch ('artifactName="' + [regex]::Escape($artifactStem) + '"')) {
    throw "CubeIDE configuration '$configuration' does not produce $artifactStem.elf."
}
if ($cprojectText -notmatch [regex]::Escape($selected.ExpectedDefine)) {
    throw "CubeIDE configuration '$configuration' is missing its explicit build-profile define."
}

Write-Host "AMBAR STM32 profile validation"
Write-Host "  Profile:       $Profile"
Write-Host "  Configuration: $configuration"
Write-Host "  Project:       $projectDir"
Write-Host "  CubeIDE CLI:   $ide"
Write-Host "  Programmer:    $programmer"
Write-Host "  ELF:           $elfPath"
if ($runtimePython) {
    Write-Host "  Runtime check: $runtimeVerifier"
    Write-Host "  Python:        $runtimePython"
}

if ($ValidateOnly) {
    Write-Host "Build/flash prerequisites are valid."
    exit 0
}

New-Item -ItemType Directory -Path $HeadlessWorkspace -Force | Out-Null
$workspaceProjectMarker = Join-Path $HeadlessWorkspace ".metadata\.plugins\org.eclipse.core.resources\.projects\$projectName\.location"
$ideArguments = @(
    "--launcher.suppressErrors",
    "-nosplash",
    "-data", $HeadlessWorkspace,
    "-application", "org.eclipse.cdt.managedbuilder.core.headlessbuild"
)
if (-not (Test-Path -LiteralPath $workspaceProjectMarker)) {
    $ideArguments += @("-import", $projectDir)
}
$ideArguments += @(
    "-cleanBuild", "$projectName/$configuration",
    "-no-indexer",
    "-printErrorMarkers"
)

$buildStartedUtc = [DateTime]::UtcNow
Write-Host "Building $projectName/$configuration..."
& $ide @ideArguments
if ($LASTEXITCODE -ne 0) {
    throw "STM32CubeIDE headless build failed with exit code $LASTEXITCODE."
}

if (-not (Test-Path -LiteralPath $elfPath -PathType Leaf)) {
    throw "CubeIDE reported success but did not produce the expected ELF: $elfPath"
}
$elf = Get-Item -LiteralPath $elfPath
if ($elf.LastWriteTimeUtc -lt $buildStartedUtc.AddSeconds(-5)) {
    throw "The expected ELF was not refreshed by this build: $elfPath"
}
if ($elf.Length -lt 20000 -or $elf.Length -gt 33554432) {
    throw "The ELF container size is outside the guarded 20 KiB..32 MiB range: $($elf.Length) bytes"
}

Write-Host "Built fresh ELF ($($elf.Length) bytes). Section sizes:"
$sizeOutput = @(& $sizeTool $elfPath 2>&1)
$sizeExitCode = $LASTEXITCODE
$sizeOutput | ForEach-Object { Write-Host ([string]$_) }
if ($sizeExitCode -ne 0) {
    throw "arm-none-eabi-size could not inspect the built ELF."
}
$sizeRow = $sizeOutput |
    Where-Object { [string]$_ -match "^\s*(\d+)\s+(\d+)\s+(\d+)\s+\d+\s+[0-9A-Fa-f]+\s+" } |
    Select-Object -Last 1
if (-not $sizeRow -or [string]$sizeRow -notmatch "^\s*(\d+)\s+(\d+)\s+(\d+)\s+") {
    throw "Could not parse text/data/bss section sizes from arm-none-eabi-size."
}
$textBytes = [int64]$Matches[1]
$dataBytes = [int64]$Matches[2]
$bssBytes = [int64]$Matches[3]
$flashBytes = $textBytes + $dataBytes
$ramBytes = $dataBytes + $bssBytes
if ($flashBytes -lt 20480 -or $flashBytes -gt 1048576) {
    throw "Loadable flash usage is outside the guarded 20 KiB..1 MiB range: $flashBytes bytes"
}
if ($ramBytes -gt 655360) {
    throw "Static RAM usage exceeds the guarded STM32H562 640 KiB budget: $ramBytes bytes"
}
Write-Host "Validated loadable usage: flash=$flashBytes bytes, static RAM=$ramBytes bytes."

if ($BuildOnly) {
    Write-Host "Build-only request complete: $elfPath"
    exit 0
}

Write-Host "Checking for an ST-LINK probe..."
$probeOutput = @(& $programmer -l st-link-only 2>&1)
$probeExitCode = $LASTEXITCODE
$probeOutput | ForEach-Object { Write-Host ([string]$_) }
if ($probeExitCode -ne 0) {
    throw "STM32CubeProgrammer could not enumerate ST-LINK probes."
}
$probeSerials = @(
    $probeOutput |
        ForEach-Object {
            if ([string]$_ -match "ST-LINK SN\s*:\s*(\S+)") {
                $Matches[1]
            }
        }
)
if ($StLinkSerial) {
    if ($probeSerials -notcontains $StLinkSerial) {
        throw "Requested ST-LINK serial '$StLinkSerial' was not found. Detected: $($probeSerials -join ', ')"
    }
} elseif ($probeSerials.Count -ne 1) {
    throw "Expected exactly one ST-LINK probe but found $($probeSerials.Count). Pass -StLinkSerial explicitly."
}

$connectArguments = @("port=SWD", "mode=UR", "reset=HWrst")
if ($StLinkSerial) {
    $connectArguments += "sn=$StLinkSerial"
}

Write-Host "Flashing and verifying $artifactStem.elf..."
& $programmer -c @connectArguments -w $elfPath -v -rst
if ($LASTEXITCODE -ne 0) {
    throw "STM32CubeProgrammer flash/verify/reset failed with exit code $LASTEXITCODE."
}

Write-Host "Flash verified. Waiting for read-only runtime profile evidence..."
$runtimeArguments = @(
    $runtimeVerifier,
    "--profile", $Profile,
    "--reconnect-seconds",
    $UsbReconnectSeconds.ToString([Globalization.CultureInfo]::InvariantCulture),
    "--listen-seconds",
    $RuntimeListenSeconds.ToString([Globalization.CultureInfo]::InvariantCulture)
)
if ($SerialPort) {
    $runtimeArguments += @("--port", $SerialPort)
}
& $runtimePython @runtimeArguments
if ($LASTEXITCODE -ne 0) {
    throw "The flashed image did not pass read-only runtime profile verification."
}

Write-Host "Flash and runtime profile verified. No motion-capable command was sent."
Write-Host "The selected HIL supervisor must still complete its full safety preflight before permitting motion."
Write-Host "Completed profile: $Profile"
exit 0
