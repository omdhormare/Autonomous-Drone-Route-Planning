@echo off
cd /d "%~dp0"
set "PATH=C:\MCA Projects\VisionGuidedLanding\ucrt64\bin;%PATH%"
where cmake >nul 2>nul
if errorlevel 1 (echo CMake missing. Open MSYS2 UCRT64 or update PATH. & pause & exit /b 1)
cmake -S . -B build -G Ninja
if errorlevel 1 (pause & exit /b 1)
cmake --build build
if errorlevel 1 (pause & exit /b 1)
build\drone_route.exe demo
if errorlevel 1 (pause & exit /b 1)
start "" "%~dp0demo.html"
pause
