# Compiles and runs the pullback regression harness (f141 runaway-aim) against
# a BoosterGuidance build. Usage:
#   .tools\diag\run-pullback-replay.ps1                 # deployed build (GREEN)
#   .tools\diag\run-pullback-replay.ps1 <dir-with-dll>  # e.g. the A1F18BC4 backup (RED)
param(
    [string]$BgDir = "$PSScriptRoot\..\..\Source\bin\Release"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$csc = Get-ChildItem (Join-Path $root ".tools") -Recurse -Filter csc.exe |
       Where-Object FullName -match "net472" | Select-Object -First 1 -ExpandProperty FullName
if (-not $csc) { throw "Roslyn csc.exe not found" }
$KSP = if ($env:KSPDIR) { $env:KSPDIR } else { "D:\GamePlatform\Steam\steamapps\common\Kerbal Space Program" }
$M = Join-Path $KSP "KSP_x64_Data\Managed"
$out = Join-Path $PSScriptRoot "PullbackReplay.exe"

& $csc /nologo /out:"$out" `
    "/r:$(Join-Path $root 'Source\bin\Release')\BoosterGuidance.dll" `
    "/r:$M\Assembly-CSharp.dll" `
    "/r:$M\UnityEngine.dll" `
    "/r:$M\UnityEngine.CoreModule.dll" `
    "/r:System.dll" `
    (Join-Path $PSScriptRoot "PullbackReplay.cs")
if ($LASTEXITCODE -ne 0) { throw "csc failed" }
& $out $BgDir $M
exit $LASTEXITCODE
