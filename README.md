# 2026 Spectrum robot code

Robot code for Spectrum 3847's FRC 2026 season.

## Description

This is the working code base for our robots in the 2026 REBUILT FRC Competition. The code is
constantly being worked on, so watch out for bugs and issues.

**To find out what the robot does, read the code.** The `docs/` directory is written for people, team
students and mentors trying to get a job done. It deliberately does not restate what the code
already says, because a copy goes stale the first time someone edits the code and then quietly
misleads. Start at [docs/index.md](docs/index.md).

### Features

* A mechanism base class that wraps the TalonFX configuration and the control mode calls, so each
  mechanism file is short.
* Triggers used as states for activating commands, which all but eliminates the need for complex
  multi-mechanism command groups.
* Simulation classes that let us build a good representation of the complete robot in simulation, so
  more of the code can be tested without the robot.
* Each mechanism refreshes all of a motor's status signals with one Phoenix `refreshAll` per robot
  loop, keyed on the loop counter, so every getter in a loop sees the same sample and the loop makes
  one JNI call per mechanism rather than one per signal.
* LED classes that use the CTRE CANdle for animations, support multiple strips on one port, set
  priorities for when to override a running animation, and allow different animations per control
  mode.

### Build tools and extensions

* **Spotless:** auto-formats all code on each build, so we keep a consistent format across
  programmers.
* **SpotBugs:** catches some common bugs in the software, such as using `=` instead of `==` in a
  conditional.
* **Lombok:** annotations that create getter and setter methods, which means less boilerplate.
* **Error Lens:** highlights errors and makes them easier to see and fix.
* **Git Config User Profiles:** lets several programmers share a computer and still commit under
  their own names.
* **Git Lens:** shows who committed a change and more.
* **SpellRight:** spell check for VSCode.

### Dependencies

* WPILib 2026
* CTRE Phoenix 6, using their swerve control code
* PathPlanner
* DogLog, for logging
* MapleSim, for drivetrain simulation

## Project structure

The shape of the repository, at the top level. Each of these has its own documentation where a
person needs one.

```text
src/           robot code
  main/java      application code
  main/deploy    files pushed to the roboRIO, including the PathPlanner paths and autos
  test           unit tests
docs/          documentation, written for people
tools/         local tools, including the robot app
scripts/       one-off Node helpers
vendordeps/    vendor dependency JSON, checked in
```

The code splits three ways. `src/main/java/frc/robot` is the season's robot application, with one
folder per mechanism and the classes that hold the hardware constants for each physical robot. It
is what changes every season. `src/main/java/frc/spectrumLib` is the code we try to reuse year to
year, and it changes rarely. `src/main/java/frc/rebuilt` is game-specific helpers for 2026 only,
and it is deleted at the end of the season.

The online JavaDoc is at <https://spectrum3847.github.io/2026-Spectrum>.
