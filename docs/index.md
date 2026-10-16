# 2026-Spectrum documentation

Two ways to use this site. Pick the one that matches where you are.

**The source code is the source of truth for what the robot does.** These pages are written for
people, team students and mentors trying to get a job done, and they deliberately do not restate
what the code already says. A copy of the code inside a document goes stale the first time someone
edits the code, and then it quietly misleads. So if you want to know what a class does, what a state
means, or what a button is bound to, open the code and read it. What these pages give you is the
context you cannot get from a single file: why a decision was made, how a task is done, and which
page answers which question.

---

## I'm new to programming

A sequential curriculum that takes you from "I've never written code" to "I can read Java well
enough to work on this robot." Each lesson assumes the previous one. The examples are invented on
purpose, a bank account, a rectangle, a list of prices, so you can learn the language before you
need to understand a robot. Once you can read the language, the robot's own code is the best
textbook.

**Read in order.** Skipping ahead and getting stuck is the most common way to bounce off.

1. [Setup Guide](setup.md): install Java, WPILib, and VSCode. You can't run anything without this.
2. [Variables and Arithmetic](frc-software-basics/variables-arithmetic.md): types, math, and `final`.
3. [Logic Operators and Strings](frc-software-basics/logic-operators.md): `if`, `&&`, and `==` versus `.equals()`.
4. [Arrays and Enums](frc-software-basics/arrays.md): fixed lists, `ArrayList`, and named constants.
5. [Loops](frc-software-basics/loops.md): `for`, `while`, and how not to hang a robot with one.
6. [Classes, Methods, and Objects](frc-software-basics/classes-methods-objects.md): the building blocks everything is made of.
7. [Code Formatting and Comments](frc-software-basics/formatting-code.md): comment syntax, and where our style rules live.
8. [Applied to FRC](frc-software-basics/applied-to-frc.md): the bridge, and the one thing that changes how you think about loops.

By the end of step 8 you will know enough Java, and enough about command-based FRC, to start
reading this robot's code. Then graduate to the reference.

---

## I already know how to program: show me the reference

Self-contained pages, browse as the work demands. Each one assumes you can read Java and have a
basic mental model of WPILib command-based robots.

### Start here

* [Setup Guide](setup.md): environment, JDK, WPILib, and cloning the repo.
* [2026 Season Specific](other-guides/2026-season-specific.md): where things live in this codebase, and the rules for deciding where a new behavior goes.
* [Offseason Handoff 2026-09-05](other-guides/offseason-handoff-2026-09-05.md): a dated snapshot of the 2026-09-04 bench day, the operating procedure the code assumes, and the open items that came out of it.
* [Loop Time and CPU Handoff 2026-09-05](other-guides/loop-time-handoff-2026-09-05.md): a dated snapshot of the saturated-CPU diagnosis, what was cut and why, the pre-existing odometry errors, and how to measure it yourself.
* [Tuning and Calibration Handoff 2026-09-08](other-guides/tuning-calibration-handoff-2026-09-08.md): a dated snapshot of where the shot map, operator trims, and motor gains are still tuned by hand, plus the work list for closing each loop.
* [Programming tips](other-guides/tips.md): the small habits that keep this codebase maintainable.
* [Photon guide to programming](other-guides/photon-guide-to-programming.md): how we think about FRC software design.
* [Development environment shortcuts](other-guides/shortcuts.md): keyboard and command palette shortcuts.

### Tools

* [Build Tools and Other Development Utilities](tools/build-tools.md): Spotless, SpotBugs, Lombok, VSCode extensions.
* [Gradle](tools/gradle.md): `./gradlew build`, deploy, sim, and the rest of the build commands.
* [Autonomous Programming (Auton)](tools/auton.md): PathPlanner, the auto chooser, event triggers.
* [Vision Systems](tools/vision.md): the Limelights, MegaTag fusion, pose-estimator integration, and calibrating the cameras from the robot app.
* [Simulation](tools/simulation.md): running the robot without a robot.
* [Logging and Data Analysis](tools/logging.md): DogLog, `Telemetry`, `.wpilog` files, and the fast between-matches triage script.
* [Phoenix Tuner X](tools/phoenix-tuner-x.md): motor configuration, swerve offsets, the plotter.
* [Swerve Alignment](tools/swerve-alignment.md): the page that zeroes the modules and writes the offsets into the code.
* [Robot App](../tools/robot-app/README.md): the local web app it lives in, for control maps, log sync, and power and CAN-bus analysis.
* [PID Tuning](tools/pid-tuning.md): gains, feedforward, the workflow.
* [Shot Records and Trim Events](tools/shot-log.md): the one-row-per-burst log, and how the trims persist.
* [Elastic Dashboard](tools/elastic.md): driver station UI, NetworkTables.
* [LEDs](tools/leds.md): `SpectrumLEDs` patterns and CANdle plans.

### Dependencies

* [Dependencies Overview](dependencies/overview.md): what's on the classpath and why.
* [WPILib](dependencies/wpilib.md)
* [CTRE Phoenix 6](dependencies/phoenix6.md)
* [PathPlannerLib](dependencies/pathplanner.md)
* [DogLog](dependencies/doglog.md)
* [MapleSim](dependencies/maple-sim.md)

### Coding conventions

* [Code Style](coding-conventions/code-style.md): naming, formatting, AOSP.
* [Class Generation and Method Building](coding-conventions/class-generation.md): subsystem layout, constructors, methods.
* [Documentation and Comments](coding-conventions/documentation-and-comments.md): when to comment, when not to.
* [Exception Handling](coding-conventions/exception-handling.md): what to catch, what to let crash.
* [Project Lombok](coding-conventions/project-lombok.md): `@Getter`, `@Setter`, `@Accessors(chain = true)`.
* [Commits and Pull Requests](coding-conventions/commits-pull-requests.md): git workflow.
* [Writing an AGENTS.md for a Robot Repo](other-guides/agents-md-guidelines.md): what to tell an AI coding agent, commit identity on shared computers, one-feature-per-PR.

---

## Each doc tells you what it expects

Every page starts with a one-line note about who it is for and what you need to know first. If a
page says "assumes you've read [Class Generation](coding-conventions/class-generation.md)", read
that first. The reference pages do not repeat shared context.

When you find something unclear or wrong, fix it. Documentation is part of the codebase, and PRs
that improve docs are merged the same way as PRs that change code. See
[Commits and Pull Requests](coding-conventions/commits-pull-requests.md) for the workflow.
