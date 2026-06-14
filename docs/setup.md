# Setup Guide

*Audience: New programmers. No prerequisites — start here.*

To set up the project you'll need:

* [WPILib](#wpilib-notes) 2026
* JDK 17 ([Eclipse Temurin](https://adoptium.net/temurin/releases?version=17) is recommended)
* [PathPlanner](https://pathplanner.dev)
* [Git](https://git-scm.com/)

## Clone the Repository

In a terminal, run:

```
git clone https://github.com/spectrum3847/2026-Spectrum.git
cd 2026-Spectrum
```

If you prefer a GUI, [GitHub Desktop](https://desktop.github.com/download/) works well. On Linux there's a [Flatpak version](https://flathub.org/en/apps/io.github.shiftey.Desktop).

## VSCode

Open the `2026-Spectrum` folder in WPILib VSCode (or your preferred editor). To run the simulator, press `Ctrl+Shift+P` and type `sim`, then click **WPILib: Simulate Robot Code**. Gradle will build the project, then prompt you to choose between **GUI Sim** and a real Driver Station. Pick **GUI Sim** — it launches Glass with joysticks and a Field2d view.

## Java Notes

Java 17 is required. WPILib 2026 does not support later versions, and the build will fail with a confusing error if the wrong JDK is active.

On macOS or Linux, [SDKMAN](https://sdkman.io) is the easiest way to manage Java versions. To install Temurin 17:

```
sdk install java 17.0.19-tem
sdk use java 17.0.19-tem
```

Run `java -version` to confirm the active JDK before building.

## WPILib Notes

Download the latest stable WPILib installer from the [GitHub releases page](https://github.com/wpilibsuite/allwpilib/releases/). Make sure you're getting the 2026 release — a 2027 installer will have breaking changes that are incompatible with this project. If you want a nightly build, the [WPILib nightly installer frontend](https://project516.dev/wpilib-nightly-installers) is easier than navigating GitHub Actions artifacts directly.

## Next Steps

See the [Table of Contents](index.md) for the rest of the documentation.

---

*Next: [Variables & Arithmetic](frc-software-basics/variables-arithmetic.md)*
