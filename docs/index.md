# 2026-Spectrum Documentation

Two ways to use this site. Pick the one that matches where you are.

---

## I'm new to programming

A sequential curriculum that takes you from "I've never written code" to "I can read and modify this robot's codebase." Each lesson assumes the previous one. Examples are pulled from this team's actual robot code, so you're learning Java *and* the codebase at the same time.

**Read in order.** Skipping ahead and getting stuck is the most common way to bounce off.

1. [Setup Guide](setup.md) — install Java, WPILib, and VSCode. You can't run anything without this.
2. [Variables & Arithmetic](frc-software-basics/variables-arithmetic.md) — types, math, and `final`.
3. [Logic-Based Operators & Strings](frc-software-basics/logic-operators.md) — `if`, `&&`, `==` vs `.equals()`.
4. [Arrays & Enums](frc-software-basics/arrays.md) — collections and named constants.
5. [Loops](frc-software-basics/loops.md) — `for`, `while`, and when not to use them on a robot.
6. [Classes, Methods, & Objects](frc-software-basics/classes-methods-objects.md) — the Java building blocks our subsystems are made of.
7. [Formatting Code & Comments](frc-software-basics/formatting-code.md) — how we write code so other people can read it.
8. [Applied to FRC](frc-software-basics/applied-to-frc.md) — the bridge. After this you can read the reference docs below.

By the end of step 8 you'll know enough Java and enough about command-based FRC to follow how a subsystem like `Launcher` actually works. Then graduate to the reference.

---

## I already know how to program — show me the reference

Self-contained pages, browse as the work demands. Each one assumes you can read Java and have a basic mental model of WPILib command-based robots.

### Start here

* [Setup Guide](setup.md) — environment, JDK, WPILib, clone the repo.
* [2026 Season Specific](other-guides/2026-season-specific.md) — what's actually in this codebase: subsystems, state machine, per-robot configs.
* [Programming Tips and Best Practices](other-guides/tips.md) — the small habits that keep this codebase maintainable.
* [Photon Guide to Programming](other-guides/photon-guide-to-programming.md) — how we think about FRC software design.

### Tools

* [Build Tools and Other Development Utilities](tools/build-tools.md) — Spotless, SpotBugs, Lombok, VSCode extensions.
* [Gradle](tools/gradle.md) — `./gradlew build`, deploy, sim, and the rest of the build commands.
* [Autonomous Programming (Auton)](tools/auton.md) — PathPlanner, the auto chooser, event triggers.
* [Vision Systems](tools/vision.md) — three Limelights, MegaTag fusion, pose-estimator integration.
* [Simulation](tools/simulation.md) — running the robot without a robot.
* [Logging and Data Analysis](tools/logging.md) — DogLog, `Telemetry`, `.wpilog` files.
* [Phoenix Tuner X](tools/phoenix-tuner-x.md) — motor configuration, swerve offsets, plotter.
* [PID Tuning](tools/pid-tuning.md) — gains, feedforward, the workflow.
* [Elastic Dashboard](tools/elastic.md) — driver-station UI, NetworkTables.
* [LEDs](tools/leds.md) — `SpectrumLEDs` patterns and CANdle plans.
* [Development Environment Shortcuts](other-guides/shortcuts.md) — keyboard and CLI shortcuts.

### Dependencies

* [Dependencies Overview](dependencies/overview.md) — what's on the classpath and why.
* [WPILib](dependencies/wpilib.md)
* [CTRE Phoenix 6](dependencies/phoenix6.md)
* [PathPlannerLib](dependencies/pathplanner.md)
* [DogLog](dependencies/doglog.md)
* [MapleSim](dependencies/maple-sim.md)
* [PhotonVision](dependencies/photonvision.md)

### Coding Conventions

* [Code Style](coding-conventions/code-style.md) — naming, formatting, AOSP.
* [Class Generation and Method Building](coding-conventions/class-generation.md) — subsystem layout, constructors, methods.
* [Documentation and Comments](coding-conventions/documentation-and-comments.md) — when to comment, when not to.
* [Exception Handling](coding-conventions/exception-handling.md) — what to catch, what to let crash.
* [Project Lombok](coding-conventions/project-lombok.md) — `@Getter`, `@Setter`, `@Accessors(chain = true)`.
* [Commits and Pull Requests](coding-conventions/commits-pull-requests.md) — git workflow.

---

## Each doc tells you what it expects

Every page starts with a one-line note about audience and prerequisites. If a reference page says "assumes you've read [Class Generation](coding-conventions/class-generation.md)", read that first — the reference docs don't repeat shared context.

When you find something unclear or wrong, fix it. Documentation is part of the codebase; PRs that improve docs are merged the same way as PRs that change code. See [Commits and Pull Requests](coding-conventions/commits-pull-requests.md) for the workflow.
