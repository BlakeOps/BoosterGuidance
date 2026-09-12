# Builds BoosterGuidance.dll without Visual Studio / MSBuild, using the Roslyn
# compiler package in .tools (install once with:
#   .tools\nuget.exe install Microsoft.Net.Compilers.Toolset -OutputDirectory .tools)
# Then deploys to the game via deploy.bat (requires KSPDIR and KSP not running).
param(
    [string]$Configuration = "Release"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$src = Join-Path $root "Source"

if (-not $env:KSPDIR) {
    $env:KSPDIR = "D:\GamePlatform\Steam\steamapps\common\Kerbal Space Program"
    Write-Host "KSPDIR not set, using default: $env:KSPDIR"
}
$KSP = $env:KSPDIR
$M = Join-Path $KSP "KSP_x64_Data\Managed"

$csc = Get-ChildItem (Join-Path $root ".tools") -Recurse -Filter csc.exe |
       Where-Object FullName -match "net472" | Select-Object -First 1 -ExpandProperty FullName
if (-not $csc) { throw "Roslyn csc.exe not found. Run: .tools\nuget.exe install Microsoft.Net.Compilers.Toolset -OutputDirectory .tools" }

Push-Location $src
try {
    $out = Join-Path $src "bin\$Configuration"
    New-Item -ItemType Directory -Force $out | Out-Null

    # Exactly the source files listed in the csproj (some .cs files are excluded dead code)
    $srcs = Select-String -Path BoosterGuidance.csproj -Pattern '<Compile Include="([^"]+)"' -AllMatches |
            ForEach-Object { $_.Matches } | ForEach-Object { "`"$src\$($_.Groups[1].Value)`"" }

    $refs = @()
    Get-ChildItem "$M\Assembly*.dll", "$M\UnityEngine*.dll" | ForEach-Object { $refs += "/r:`"$($_.FullName)`"" }
    $refs += "/r:System.dll"
    $refs += "/r:`"$KSP\GameData\000_ClickThroughBlocker\Plugins\ClickThroughBlocker.dll`""
    $refs += "/r:`"$KSP\GameData\SpaceTuxLibrary\Plugins\KSP_Log.dll`""
    $refs += "/r:`"$KSP\GameData\001_ToolbarControl\Plugins\ToolbarControl.dll`""

    $rsp = Join-Path $out "build.rsp"
    Set-Content -Encoding utf8 $rsp ("/target:library /optimize+ /langversion:latest /out:`"$out\BoosterGuidance.dll`" /nologo " + ($refs -join ' ') + " " + ($srcs -join ' '))
    & $csc "@$rsp"
    if ($LASTEXITCODE -ne 0) { throw "csc failed with exit code $LASTEXITCODE" }
    Write-Host "Built $out\BoosterGuidance.dll"
}
finally {
    Pop-Location
}

if (Get-Process KSP_x64 -ErrorAction SilentlyContinue) {
    Write-Warning "KSP is running - skipping deploy. Close KSP and run deploy.bat manually:"
    Write-Warning "  deploy.bat Source\bin\$Configuration\ BoosterGuidance.dll BoosterGuidance"
    exit 2
}
Push-Location $root # deploy.bat uses paths relative to the repo root
try {
    cmd /c "`"$root\deploy.bat`" Source\bin\$Configuration\ BoosterGuidance.dll BoosterGuidance"
}
finally {
    Pop-Location
}
Write-Host "Deployed to $KSP\GameData\BoosterGuidance"
