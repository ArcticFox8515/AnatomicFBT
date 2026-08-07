@echo off
setlocal
rem Unregisters the driver directory registered by install-driver.bat.

set "VRPATHS=%LOCALAPPDATA%\openvr\openvrpaths.vrpath"
if not exist "%VRPATHS%" (
    echo Cannot find "%VRPATHS%" - nothing to do.
    pause
    exit /b 1
)

rem No pipe in the PowerShell command: inside a for /f backquote block cmd hands the
rem escaped ^| to powershell verbatim, which then fails to parse the command.
for /f "usebackq delims=" %%i in (`powershell -NoProfile -Command "(ConvertFrom-Json (Get-Content -Raw '%VRPATHS%')).runtime[0]"`) do set "STEAMVR=%%i"
if not defined STEAMVR (
    echo Could not read the SteamVR runtime path from "%VRPATHS%".
    pause
    exit /b 1
)

set "VRPATHREG=%STEAMVR%\bin\win64\vrpathreg.exe"
if not exist "%VRPATHREG%" (
    echo Cannot find "%VRPATHREG%".
    pause
    exit /b 1
)

set "DRIVER=%~dp0build\driver\00trackingcorrector"
echo Unregistering "%DRIVER%"
"%VRPATHREG%" removedriver "%DRIVER%"
"%VRPATHREG%" show
echo.
echo Restart SteamVR for the change to take effect.
pause
