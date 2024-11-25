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
start "" /d %CaveScan_DIR%EtheURG\build\src\Debug EtheURGComp.exe
start "" /d %CaveScan_DIR%MeasurementSystem\build\src\Debug MeasurementSystemComp.exe
start "" /d %CaveScan_DIR%Registration\build\src\Debug RegistrationComp.exe
start "" /d %CaveScan_DIR%Analyses\build\src\Debug AnalysesComp.exe
start "" /d %CaveScan_DIR%PointCloud_Viewer\build\src\Debug PointCloud_ViewerComp.exe
start "" /d %CaveScan_DIR%FPS\build\src\Debug FPSComp.exe
start "" /d %CaveScan_DIR%WallDTC\build\src\Debug WallDTCComp.exe
start "" /d %CaveScan_DIR%Contour\build\src\Debug ContourComp.exe
start "" /d %CaveScan_DIR%MapViewer\build\src\Debug MapViewerComp.exe
timeout 10

rem Automatic port connection
cd %LAUNCH_DIR%
rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/EtheURG0.rtc:range ^
/localhost/%COMPUTERNAME%.host_cxt/MeasurementSystem0.rtc:range ^
--property dataport.interface_type=shared_memory

rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/MeasurementSystem0.rtc:new_PointCloud ^
/localhost/%COMPUTERNAME%.host_cxt/Registration0.rtc:new_PointCloud ^
--property dataport.interface_type=shared_memory

rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/MeasurementSystem0.rtc:new_PointCloud ^
/localhost/%COMPUTERNAME%.host_cxt/PointCloud_Viewer0.rtc:new_PointCloud ^
--property dataport.interface_type=shared_memory

rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/Registration0.rtc:merge_PointCloud ^
/localhost/%COMPUTERNAME%.host_cxt/Analyses0.rtc:merge_PointCloud ^
--property dataport.interface_type=shared_memory

rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/Registration0.rtc:merge_PointCloud ^
/localhost/%COMPUTERNAME%.host_cxt/PointCloud_Viewer0.rtc:merge_PointCloud ^
--property dataport.interface_type=shared_memory

rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/Registration0.rtc:Localization ^
/localhost/%COMPUTERNAME%.host_cxt/PointCloud_Viewer0.rtc:Localization ^
--property dataport.interface_type=shared_memory

rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/Analyses0.rtc:analyses_PointCloud ^
/localhost/%COMPUTERNAME%.host_cxt/PointCloud_Viewer0.rtc:analyses_PointCloud ^
--property dataport.interface_type=shared_memory
///////////////////////////////////////////////////////////////////////
rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/Registration0.rtc:merge_PointCloud ^
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

rem Automatically configuration
rtconf /localhost/%COMPUTERNAME%.host_cxt/MeasurementSystem0.rtc set DEVICE_NAME \\.\COM5

endlocal