# 2026 Spectrum documentation

The robot code is Rust, in `rust/`. The code is the source of truth for what the robot does. These pages hold what the code cannot say: how to work with it, and why it is built this way.

## New to the project

1. [Setup](setup.md): install Rust, download the libraries, run the simulation.
2. [2026 season specific](other-guides/2026-season-specific.md): where things live and how the robot is organized.
3. [Cargo xtask](tools/cargo-xtask.md): the build, sim and deploy commands.
4. [Code style](coding-conventions/code-style.md): the rules `cargo xtask gate` checks.

## New to programming

The pages in `frc-software-basics/` teach general ideas with Java examples. They are background from before the Rust port. The ideas carry over (variables, conditions, loops, classes), but the syntax differs in Rust. For Rust, read [The Rust Book](https://doc.rust-lang.org/book/).

1. [Variables and arithmetic](frc-software-basics/variables-arithmetic.md)
2. [Logic operators and strings](frc-software-basics/logic-operators.md)
3. [Arrays and enums](frc-software-basics/arrays.md)
4. [Loops](frc-software-basics/loops.md)
5. [Classes, methods and objects](frc-software-basics/classes-methods-objects.md)
6. [Formatting code and comments](frc-software-basics/formatting-code.md)
7. [Applied to FRC](frc-software-basics/applied-to-frc.md)

## Tools

* [Cargo xtask](tools/cargo-xtask.md): fetch-natives, sim, build-roborio, deploy.
* [Build tools](tools/build-tools.md): rustfmt, clippy, tests, editor setup.
* [Simulation](tools/simulation.md): running the robot without a robot.
* [Autonomous programming](tools/auton.md): PathPlanner, the auto chooser, event triggers.
* [Vision](tools/vision.md): Limelights and pose estimation.
* [Logging](tools/logging.md): `telemetry`, wpilog files.
* [Elastic dashboard](tools/elastic.md): driver station dashboard.
* [Phoenix Tuner X](tools/phoenix-tuner-x.md): motor setup and swerve offsets.
* [PID tuning](tools/pid-tuning.md): gains and feedforward.
* [LEDs](tools/leds.md): CANdle patterns.

## Dependencies

Rust crates in this repo replace the Java libraries. They are ports of pinned upstream versions.

* [Overview](dependencies/overview.md)
* [WPILib](dependencies/wpilib.md)
* [CTRE Phoenix 6](dependencies/phoenix6.md)
* [PathPlanner](dependencies/pathplanner.md)
* [DogLog](dependencies/doglog.md)
* [maple-sim](dependencies/maple-sim.md)

## Coding conventions

* [Code style](coding-conventions/code-style.md)
* [Adding a subsystem](coding-conventions/adding-a-subsystem.md)
* [Exception handling](coding-conventions/exception-handling.md): errors and panics.
* [Documentation and comments](coding-conventions/documentation-and-comments.md)
* [Commits and pull requests](coding-conventions/commits-pull-requests.md)

## Other guides

* [Programming tips](other-guides/tips.md)
* [Photon guide to programming](other-guides/photon-guide-to-programming.md)
* [Development shortcuts](other-guides/shortcuts.md)
* [Writing an AGENTS.md for a robot repo](other-guides/agents-md-guidelines.md)

## Fixing the docs

When a page is wrong or unclear, fix it. Docs are part of the code base and go through the same pull requests. See [Commits and pull requests](coding-conventions/commits-pull-requests.md).
