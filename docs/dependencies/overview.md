# Dependencies Overview

*Audience: Reference. No prerequisites.*

Each page in this directory covers one library and answers two questions: how does this team use it, and what has bitten us. It does not describe what the library is, and it does not restate what the library's own documentation already says. If you want to know what a class or a method does, read the class. If you want to know which libraries we depend on and at which versions, read [`vendordeps/`](../../vendordeps/): it is machine-readable, and it is the only place a version number needs to be recorded.

## The libraries we depend on

* [WPILib](wpilib.md)
* [CTRE Phoenix 6](phoenix6.md)
* [PathPlannerLib](pathplanner.md)
* [DogLog](doglog.md)
* [MapleSim](maple-sim.md)

To bump a version, install the new vendor JSON through WPILib VSCode's `Manage Vendor Libraries`, replace the old file in `vendordeps/`, and run `./gradlew build` so GradleRIO picks it up.

## JavaDoc link bases are not dependencies

[`build.gradle`](../../build.gradle) registers external JavaDoc link bases so our generated docs cross-reference the APIs we call. Those URLs are not vendor jars, and having one there does not mean the library compiles. REV Robotics and Phoenix v5 both have a link base and neither has a vendor JSON: this robot is Phoenix 6 only, and there are no REV controllers on it. If you find yourself writing code against either, drop the matching JSON into `vendordeps/` so it actually compiles.

## Adding a new library

1. Install the vendor JSON with `Manage Vendor Libraries`, then `Install new library (online)`, in WPILib VSCode. The file lands in `vendordeps/`.
2. Run `./gradlew build` so GradleRIO fetches the jar and the native bits.
3. Add the library's JavaDoc URL to the `javadoc.options.setLinks(...)` list in [`build.gradle`](../../build.gradle). This is the step people forget, and without it every type from that library renders as unlinked plain text in our generated docs.
4. Add the library to [`docs/index.md`](../index.md), and write a page in this directory if there is a way of using it that a newcomer would get wrong.

If you want the bigger picture on the surrounding subsystems before diving into a specific library, [2026 Season Specific](../other-guides/2026-season-specific.md) is the place to start.
