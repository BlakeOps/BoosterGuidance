
@echo off

rem H is the destination game folder
rem GAMEDIR is the name of the mod folder (usually the mod name)
rem GAMEDATA is the name of the local GameData
rem VERSIONFILE is the name of the version file, usually the same as GAMEDATA,
rem    but not always

set H=%KSPDIR%
rem KSPDIR-less shells (background deploy watchers) used to xcopy into
rem \GameData on the repo drive - same default as .tools\build.ps1
if "%H%"=="" set H=D:\GamePlatform\Steam\steamapps\common\Kerbal Space Program
set GAMEDIR=BoosterGuidance
set GAMEDATA="GameData"
set VERSIONFILE=%GAMEDIR%.version

if not exist %GAMEDATA%\%GAMEDIR%\Plugins mkdir %GAMEDATA%\%GAMEDIR%\Plugins

copy /Y "%1%2" "%GAMEDATA%\%GAMEDIR%\Plugins"
copy /Y "%1%3".pdb "%GAMEDATA%\%GAMEDIR%\Plugins"

copy /Y %VERSIONFILE% %GAMEDATA%\%GAMEDIR%

xcopy /y /s /I %GAMEDATA%\%GAMEDIR% "%H%\GameData\%GAMEDIR%"

rem pause
