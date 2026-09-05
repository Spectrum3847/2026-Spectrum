@echo off
REM Double-click this to open the swerve alignment app.
REM To put it on the desktop: right-click this file -> Send to -> Desktop (create shortcut)
title Swerve Alignment

cd /d "%~dp0..\.."

where node >nul 2>nul
if errorlevel 1 (
    echo Node.js was not found on this computer.
    echo Install it from https://nodejs.org and run this again.
    echo.
    pause
    exit /b 1
)

node "tools\swerve-align\server.js"

if errorlevel 1 (
    echo.
    echo The alignment app stopped with an error. The message above says why.
    pause
)
