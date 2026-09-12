# Compiles and runs the suicide-throttle replay harness against a flight log.
# Usage: .tools\diag\run-throttle-replay.ps1 [flight.dat]
param(
    [string]$Log = "$PSScriptRoot\..\..\.claude\flight25.dat"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$csc = Get-ChildItem (Join-Path $root ".tools") -Recurse -Filter csc.exe |
       Where-Object FullName -match "net472" | Select-Object -First 1 -ExpandProperty FullName
if (-not $csc) { throw "Roslyn csc.exe not found" }
$KSP = if ($env:KSPDIR) { $env:KSPDIR } else { "D:\GamePlatform\Steam\steamapps\common\Kerbal Space Program" }
$M = Join-Path $KSP "KSP_x64_Data\Managed"
$bgDir = Join-Path $root "Source\bin\Release"
$out = Join-Path $PSScriptRoot "ThrottleReplay.exe"

& $csc /nologo /out:"$out" `
    "/r:$bgDir\BoosterGuidance.dll" `
    "/r:$M\Assembly-CSharp.dll" `
    "/r:$M\UnityEngine.dll" `
    "/r:$M\UnityEngine.CoreModule.dll" `
    "/r:System.dll" `
    (Join-Path $PSScriptRoot "ThrottleReplay.cs")
if ($LASTEXITCODE -ne 0) { throw "csc failed" }
& $out $Log $bgDir $M
exit $LASTEXITCODE
