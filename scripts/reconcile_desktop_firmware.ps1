param(
    [string]$DesktopProject = (Join-Path $env:OneDrive "Desktop\stm32_airbrake_pcb"),
    [string]$CanonicalProject
)

<#
Replace the stale Desktop STM32 project directory with a junction to the
canonical repository project. The original directory is retained as a
timestamped backup. The script deliberately refuses to run while CubeIDE is
open so an active workspace cannot be redirected underneath the IDE.
#>

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = (Resolve-Path (Join-Path $scriptDir "..")).Path
if (-not $CanonicalProject) {
    $CanonicalProject = Join-Path $repoRoot "firmware\stm32_airbrake_pcb"
}

$canonical = (Resolve-Path -LiteralPath $CanonicalProject).Path
$desktopParent = (Resolve-Path -LiteralPath (Split-Path -Parent $DesktopProject)).Path
$desktopLeaf = Split-Path -Leaf $DesktopProject
$desktop = Join-Path $desktopParent $desktopLeaf
$expectedDesktopParent = (Resolve-Path -LiteralPath (Join-Path $env:OneDrive "Desktop")).Path

if ($desktopParent -ne $expectedDesktopParent) {
    throw "Refusing to move a project outside the OneDrive Desktop: $desktop"
}
if (-not $canonical.StartsWith($repoRoot, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "Canonical project must remain inside the repository: $canonical"
}

$cubeProcesses = Get-CimInstance Win32_Process |
    Where-Object { $_.Name -in @("stm32cubeide.exe", "stm32cubeidec.exe") }
if ($cubeProcesses) {
    throw "Close STM32CubeIDE before reconciling the Desktop project."
}

$existing = Get-Item -LiteralPath $desktop -Force -ErrorAction SilentlyContinue
if ($existing -and ($existing.Attributes -band [IO.FileAttributes]::ReparsePoint)) {
    $target = $existing.Target
    if ($target -eq $canonical -or ($target -is [array] -and $target -contains $canonical)) {
        Write-Host "Desktop project already points to the canonical repository."
        exit 0
    }
    throw "Desktop project is already a link to a different target: $target"
}

if ($existing) {
    $stamp = Get-Date -Format "yyyyMMdd-HHmmss"
    $backup = Join-Path $desktopParent "${desktopLeaf}_backup_$stamp"
    if (Test-Path -LiteralPath $backup) {
        throw "Backup destination already exists: $backup"
    }

    $resolvedExisting = (Resolve-Path -LiteralPath $desktop).Path
    if (-not $resolvedExisting.StartsWith(
            $expectedDesktopParent + [IO.Path]::DirectorySeparatorChar,
            [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Resolved Desktop project escaped the intended Desktop folder: $resolvedExisting"
    }

    Write-Host "Moving the existing Desktop copy to:"
    Write-Host "  $backup"
    Move-Item -LiteralPath $resolvedExisting -Destination $backup
}

Write-Host "Creating Desktop junction:"
Write-Host "  $desktop"
Write-Host "    -> $canonical"
New-Item -ItemType Junction -Path $desktop -Target $canonical | Out-Null

$created = Get-Item -LiteralPath $desktop -Force
if (-not ($created.Attributes -band [IO.FileAttributes]::ReparsePoint)) {
    throw "The Desktop junction was not created successfully."
}

Write-Host "Canonical firmware source is now the only active Desktop project path."
