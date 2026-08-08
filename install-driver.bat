@echo off
setlocal
rem Registers build\driver\00trackingcorrector with SteamVR as an external driver
rem (no file copying: SteamVR loads it straight out of the build tree).

set "VRPATHS=%LOCALAPPDATA%\openvr\openvrpaths.vrpath"
if not exist "%VRPATHS%" (
    echo Cannot find "%VRPATHS%" - is SteamVR installed and started at least once?
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
if not exist "%DRIVER%\bin\win64\driver_00trackingcorrector.dll" (
    echo Driver DLL not built yet - run build-driver.bat first.
    pause
    exit /b 1
)

echo Registering "%DRIVER%"
"%VRPATHREG%" adddriver "%DRIVER%"
"%VRPATHREG%" show
echo.
echo Restart SteamVR to load the driver. Log:
echo   %LOCALAPPDATA%\TrackingCorrector\driver-vrserver.log
pause
