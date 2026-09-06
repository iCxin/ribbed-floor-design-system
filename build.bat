@echo off
rem ============================================================================
rem  Hydraulic Ribbed-Floor Auxiliary Design System V1.0 - Windows build script
rem  Auto-detects g++ (MinGW-w64 / WinLibs / MSYS2) or MSVC cl. Pure C++17.
rem ============================================================================
setlocal
cd /d "%~dp0"

set "OUTDIR=build"
set "EXE=%OUTDIR%\ribbed_floor_design_system.exe"
if not exist "%OUTDIR%" mkdir "%OUTDIR%"

rem ---- Prefer g++ found on PATH ----
set "GPP="
for /f "delims=" %%P in ('where g++ 2^>nul') do (
    set "GPP=%%P"
    goto :have_gpp
)

rem ---- Probe WinGet WinLibs install location ----
for /f "delims=" %%P in ('dir /b /s "%LOCALAPPDATA%\Microsoft\WinGet\Packages\g++.exe" 2^>nul') do (
    set "GPP=%%P"
    goto :have_gpp
)

rem ---- Probe common install locations ----
if exist "C:\msys64\mingw64\bin\g++.exe" set "GPP=C:\msys64\mingw64\bin\g++.exe" & goto :have_gpp
if exist "C:\MinGW\bin\g++.exe" set "GPP=C:\MinGW\bin\g++.exe" & goto :have_gpp
goto :try_msvc

:have_gpp
echo [build] g++: %GPP%
"%GPP%" -std=c++17 -Wall -Wextra -O2 -static -static-libgcc -static-libstdc++ -Iinclude -o "%EXE%" src\*.cpp -lws2_32
if errorlevel 1 goto :fail
goto :done

:try_msvc
where cl >nul 2>nul || goto :nocompiler
echo [build] using MSVC cl
cl /nologo /std:c++17 /W4 /EHsc /O2 /Iinclude /utf-8 src\*.cpp ws2_32.lib /Fe:"%EXE%"
if errorlevel 1 goto :fail
goto :done

:done
echo.
echo [build] OK: %EXE%
echo [run]   selftest: %EXE% --selftest
echo [run]   demo:     %EXE% --demo  /  %EXE% --one-way
exit /b 0

:nocompiler
echo [build] ERROR: no C++ compiler found (g++ or cl). Install MinGW-w64 or VS Build Tools.
exit /b 1

:fail
echo [build] compile failed
exit /b 1
