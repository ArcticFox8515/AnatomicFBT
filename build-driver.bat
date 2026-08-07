@echo off
rem Builds the step-1 spike driver DLL (doc/driver-plan.md) and the spike client.
rem Release, because vrserver.exe would otherwise need the debug CRT to load us.
rem vrserver.exe keeps the DLL open: CLOSE STEAMVR BEFORE BUILDING.
cmake --build build --config Release --target driver_00trackingcorrector spike_client
pause
