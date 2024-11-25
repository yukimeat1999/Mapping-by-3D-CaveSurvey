@echo off

rem /////////////////////////// CaveScan_Startup.bat ///////////////////////////

rem This file is the CaveScan startup batch file.
rem Created by Y.Fujii on November 25, 2024.
rem License declaration 2.0.

rem //////////////////////////////////////////////////////////////////////////


setlocal ENABLEDELAYEDEXPANSION

rem Directory Move
cd %~dp0
set LAUNCH_DIR=%~dp0
rem WorkSpase Directory
cd %LAUNCH_DIR%..
set CaveScan_DIR=%CD%\

rem If a computer name error occurs, set the following variables.
rem set COMPUTERNAME=yuki-PC

rem Startup of RTC
start "" /d %CaveScan_DIR%PointCloud_Reader\build\src\Debug PointCloud_ReaderComp.exe
start "" /d %CaveScan_DIR%FPS\build\src\Debug FPSComp.exe
start "" /d %CaveScan_DIR%WallDTC\build\src\Debug WallDTCComp.exe
start "" /d %CaveScan_DIR%Contour\build\src\Debug ContourComp.exe
start "" /d %CaveScan_DIR%MapViewer\build\src\Debug MapViewerComp.exe
timeout 10

rem Automatic port connection
cd %LAUNCH_DIR%
rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/PointCloud_Reader0.rtc:File_PointCloud ^
/localhost/%COMPUTERNAME%.host_cxt/FPS0.rtc:PCD ^
--property dataport.interface_type=shared_memory

rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/FPS0.rtc:DownPCD ^
/localhost/%COMPUTERNAME%.host_cxt/WallDTC0.rtc:PCD ^
--property dataport.interface_type=shared_memory

rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/FPS0.rtc:DownPCD ^
/localhost/%COMPUTERNAME%.host_cxt/Contour0.rtc:PCD ^
--property dataport.interface_type=shared_memory

rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/WallDTC0.rtc:PlanWall ^
/localhost/%COMPUTERNAME%.host_cxt/MapViewer0.rtc:PlanWall ^
--property dataport.interface_type=shared_memory

rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/Contour0.rtc:Contour ^
/localhost/%COMPUTERNAME%.host_cxt/MapViewer0.rtc:Contour ^
--property dataport.interface_type=shared_memory

endlocal