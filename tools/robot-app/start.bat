@echo off
REM Double-click this to open the Spectrum robot app.
REM Desktop shortcut: right-click this file -> Send to -> Desktop (create shortcut)
title Spectrum robot app

cd /d "%~dp0"

where node >nul 2>nul
if errorlevel 1 (
    echo Node.js was not found on this computer.
    echo Install it from https://nodejs.org and run this again.
    echo.
    pause
    exit /b 1
)

if not exist "node_modules" (
    echo First run: installing dependencies. This needs internet, once.
    call npm install
    if errorlevel 1 (
        echo.
        echo npm install failed. The message above says why.
        pause
        exit /b 1
    )
)

call npm run build
if errorlevel 1 (
    echo.
    echo The build failed. The message above says why.
    pause
    exit /b 1
)

node server\index.js --open %1

if errorlevel 1 (
    echo.
    echo The app stopped with an error. The message above says why.
    pause
)
