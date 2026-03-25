# Setup Guide

You will need the following:

* [WPILib](setup.md#wpilib-notes) (2026)
* JDK 17 ([Eclipse Temurin](https://adoptium.net/temurin/releases?version=17) is recommended)
* [PathPlanner](https://pathplanner.dev) (for autons)
* [Git](https://git-scm.com/)

## Clone the repository

In a place you won't forget, open the terminal and type:
```
git clone https://github.com/spectrum3847/2026-Spectrum.git
cd 2026-Spectrum
```

This will clone (copy) the git repository locally to your computer. **Note:** If you perfer a GUI, [GitHub Desktop](https://desktop.github.com/download/) works well. If you are on Linux, there [is a Flatpak](https://flathub.org/en/apps/io.github.shiftey.Desktop) avalable. 

## VSCode

Open the `2026-Spectrum` in WPILib VSCode (or whatever you prefer, really). To use the WPILib sim, press ctrl shift p, and type `sim`. Click WPILib: Simulate Robot Code. Then gradle will build the project. Once it's done, it will ask you to either run in GUI Sim, or use a real driver station. Press `GUI Sim`, it will launch Glass with the sim.

## Java notes:

If you are on Unix (MacOS/Linux), I strongly recommend using [SDKMAN](https://sdkman.io) for managing java installations. To set it up Java 17 (required for the robot code), run `sdk install java 17.0.18-tem` (or whatever the latest Java 17 is). Then if you want to switch to it, run `sdk use java 17.0.18-tem`.

Also, for this robot project Java 17 is required: you cannot use a higher version, as WPILib 2026 does not support it.

## WPILib notes:

Download the latest stable WPILib installer from [GitHub releases](https://github.com/wpilibsuite/allwpilib/releases/). If your feeling extra adventurous, you can download the latest version of WPILib installer from the GitHub actions artifats from the [WPILibInstaller-Avalonia repository](https://github.com/wpilibsuite/WPILibInstaller-Avalonia), if that sounds daunting, check out this [frontend](https://project516.dev/wpilib-nightly-installers). Make sure your downloading the 2026 version of WPILib, as 2027 should have breaking changes and be incompatible with 2026.