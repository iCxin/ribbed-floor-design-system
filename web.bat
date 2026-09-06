@echo off
rem ============================================================================
rem  One-click Web UI launcher: build if needed, start server, open browser.
rem ============================================================================
setlocal
cd /d "%~dp0"

if not exist "build\ribbed_floor_design_system.exe" (
    echo [web] first run - building...
    call build.bat || exit /b 1
)

start "" http://127.0.0.1:8080/
build\ribbed_floor_design_system.exe --web 8080
endlocal
