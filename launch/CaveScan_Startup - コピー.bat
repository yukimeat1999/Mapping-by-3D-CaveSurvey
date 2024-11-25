rem @echo off

rem /////////////////////////// CaveScan_Startup.bat ///////////////////////////

rem This file is the CaveScan startup batch file.
rem Created by Y.Fujii on November 1, 2023.
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
set COMPUTERNAME=DESKTOP

rem Startup of RTC
start "" /d %CaveScan_DIR%PointCloud_Reader\build\src\Debug PointCloud_ReaderComp.exe
start "" /d %CaveScan_DIR%Registration\build\src\Debug RegistrationComp.exe
start "" /d %CaveScan_DIR%Analyses\build\src\Debug AnalysesComp.exe
start "" /d %CaveScan_DIR%PointCloud_Viewer\build\src\Debug PointCloud_ViewerComp.exe
timeout 20

rem Automatic port connection
rem cd %LAUNCH_DIR%
cd %CaveScan_DIR%
rem rtcon ^
rem /localhost/%COMPUTERNAME%.host_cxt/EtheURG0.rtc:range ^
rem /localhost/%COMPUTERNAME%.host_cxt/MeasurementSystem0.rtc:range ^
rem --property dataport.interface_type=shared_memory

rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/PointCloud_Reader0.rtc:File_PointCloud ^
/localhost/%COMPUTERNAME%.host_cxt/Registration0.rtc:new_PointCloud ^
--property dataport.interface_type=shared_memory

rtcon ^
/localhost/%COMPUTERNAME%.host_cxt/PointCloud_Reader0.rtc:File_PointCloud ^
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
timeout 20
rem Automatically configuration
rem rtconf /localhost/%COMPUTERNAME%.host_cxt/MeasurementSystem0.rtc set DEVICE_NAME \\.\COM5

endlocal