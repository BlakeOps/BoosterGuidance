# Compiles and runs the flight-logging-v2 format gate (task 5.4).
# Usage:
#   .tools\diag\run-log-format-check.ps1                 # current build
#   .tools\diag\run-log-format-check.ps1 <dir-with-dll>  # e.g. a backup
param(
    [string]$BgDir = "$PSScriptRoot\..\..\Source\bin\Release"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$csc = Get-ChildItem (Join-Path $root ".tools") -Recurse -Filter csc.exe |
       Where-Object FullName -match "net472" | Select-Object -First 1 -ExpandProperty FullName
if (-not $csc) { throw "Roslyn csc.exe not found" }
$out = Join-Path $PSScriptRoot "LogFormatCheck.exe"

& $csc /nologo /out:"$out" `
    "/r:$(Join-Path $root 'Source\bin\Release')\BoosterGuidance.dll" `
    (Join-Path $PSScriptRoot "LogFormatCheck.cs")
if ($LASTEXITCODE -ne 0) { throw "csc failed" }
$KSP = if ($env:KSPDIR) { $env:KSPDIR } else { "D:\GamePlatform\Steam\steamapps\common\Kerbal Space Program" }
$M = Join-Path $KSP "KSP_x64_Data\Managed"
& $out $BgDir (Join-Path $root "Source\Core\BLController.cs") $M
exit $LASTEXITCODE
