# Flight-log monitor (2026-10-04 workflow): polls the BG Logs dir, snapshots
# any *.dat that changes into .claude/flightlog_snapshots/ immediately (files
# can be truncated by a later same-name session), and EXITS when an Actual.dat
# stops growing for >=20 s after having grown (= touchdown / logging stopped).
# Exit wakes the parent session -> dispatch analysis subagent -> restart me.
# Usage: powershell -File .tools/diag/watch-flightlogs.ps1
$ErrorActionPreference = "Stop"
$KSP = if ($env:KSPDIR) { $env:KSPDIR } else { "D:\GamePlatform\Steam\steamapps\common\Kerbal Space Program" }
$logs = Join-Path $KSP "Logs\BoosterGuidance"
$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$snap = Join-Path $root ".claude\flightlog_snapshots"
New-Item -ItemType Directory -Force $snap | Out-Null

# Baseline: everything currently present is OLD; only react to changes after start
$size = @{}
$grew = @{}
Get-ChildItem $logs -Filter *.dat | ForEach-Object { $size[$_.FullName] = $_.Length }
Write-Output ("BASELINE " + $size.Count + " files at " + (Get-Date -Format HH:mm:ss))

$stable = 0
$activeActual = $null
while ($true) {
    Start-Sleep -Seconds 5
    $changed = $false
    Get-ChildItem $logs -Filter *.dat | ForEach-Object {
        $f = $_
        if ((-not $size.ContainsKey($f.FullName)) -or ($size[$f.FullName] -ne $f.Length)) {
            $size[$f.FullName] = $f.Length
            $grew[$f.FullName] = $true
            $changed = $true
            try { Copy-Item $f.FullName (Join-Path $snap $f.Name) -Force } catch {}
            if ($f.Name -like "*.Actual.dat") { $script:activeActual = $f.FullName }
        }
    }
    if ($changed) { $stable = 0; continue }
    if ($activeActual -and $grew[$activeActual]) {
        $stable += 5
        if ($stable -ge 20) {
            Write-Output ("STOPWRITE " + (Split-Path $activeActual -Leaf) + " size=" + $size[$activeActual] + " at " + (Get-Date -Format HH:mm:ss))
            exit 0
        }
    }
}
