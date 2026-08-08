@echo off
rem Builds the SteamVR driver DLL (doc/driver-plan.md).
rem Release, because vrserver.exe would otherwise need the debug CRT to load us.
rem vrserver.exe keeps the DLL open: CLOSE STEAM BEFORE BUILDING.
cmake --build build --config Release --target driver_00trackingcorrector
pause
