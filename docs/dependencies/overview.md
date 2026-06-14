# Dependencies Overview

*Audience: Reference. No prerequisites.*

This section walks through how each third-party library actually shows up in the codebase: which files use it, what conventions we've settled on, and the gotchas we've already hit. The point isn't to replace the official docs — it's to skip past the "hello world" examples and get straight to "how do we do it here."

## What's Actually on the Classpath

Vendor JSONs live in [`vendordeps/`](../../vendordeps/) and pin the exact versions:

| Library | Version | Page |
| --- | --- | --- |
| WPILib (allwpilib + New Commands) | 2026 | [WPILib](wpilib.md) |
| CTRE Phoenix 6 | 26.1.3 | [Phoenix 6](phoenix6.md) |
| PathPlannerLib | 2026.1.2 | [PathPlanner](pathplanner.md) |
| DogLog | 2026.5.0 | [DogLog](doglog.md) |
| MapleSim (IronMaple) | 0.4.0-beta | [MapleSim](maple-sim.md) |
| PhotonLib | v2026.3.4 | [PhotonVision](photonvision.md) |

To bump a version, swap the JSON via WPILib VSCode's `Manage Vendor Libraries`, then `./gradlew build`.

## What's Not Actually on the Classpath

`build.gradle` also registers a few JavaDoc-only link bases. These aren't vendor jars we depend on — they just let our generated JavaDoc cross-link to external APIs:

* REV Robotics. No REV motor controllers in `2026-Spectrum`, but the link is there in case someone pulls in REV code.
* Phoenix v5. Same story. Phoenix 6 is a clean break, and we're all-in on it.
* Java 17 stdlib. Not a dep, just a cross-reference target.

If you do find yourself adding REV or Phoenix v5 code, drop the matching JSON into `vendordeps/` so it actually compiles.

## Adding a New Library

1. Use `Manage Vendor Libraries → Install new library (online)` in WPILib VSCode and paste the vendor's URL. The JSON lands in `vendordeps/`.
2. Run `./gradlew build` so GradleRIO fetches the jar and native bits.
3. Add the JavaDoc link base to `build.gradle`'s `javadoc.options.setLinks(...)` block so cross-references resolve.
4. Write a page in this directory that documents how *we're* using it, and link it from the table above and from [`index.md`](../index.md).

If you want the bigger picture on the surrounding subsystems before diving into a specific library, [2026 Season Specific](../other-guides/2026-season-specific.md) is the place to start.
