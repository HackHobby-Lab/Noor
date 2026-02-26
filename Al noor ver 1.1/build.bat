@echo off
REM Build script for Noor Audio Player with USB-MSC
REM This script sets up the ESP-IDF environment and builds the project

echo.
echo ========================================
echo Noor Audio Player - Build Script
echo ========================================
echo.

REM Check if we're in the right directory
if not exist "main\main.c" (
    echo ERROR: main\main.c not found!
    echo Please run this script from the project root directory
    echo Current directory: %CD%
    pause
    exit /b 1
)

echo Workspace: %CD%
echo.

REM Set up ESP-IDF environment
echo Setting up ESP-IDF environment...
call C:\Espressif\idf_cmd_init.bat

if errorlevel 1 (
    echo ERROR: Failed to set up ESP-IDF environment
    pause
    exit /b 1
)

echo.
echo Environment setup complete!
echo.

REM Clean old build
echo Cleaning old build artifacts...
if exist "build" (
    rmdir /s /q "build" 
    echo Build directory removed
)

echo.
echo Starting build...
echo.

REM Run the build
python "C:\Espressif\frameworks\esp-idf-v5.5.2\tools\idf.py" build

if errorlevel 1 (
    echo.
    echo ERROR: Build failed!
    pause
    exit /b 1
)

echo.
echo ========================================
echo Build completed successfully!
echo ========================================
echo.
echo Next steps:
echo 1. Connect ESP32S3 via USB
echo 2. Run: idf.py flash monitor
echo.
pause
