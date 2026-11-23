# 2026 Spectrum robot code

Robot code for FRC Team Spectrum 3847, 2026 season (REBUILT), written in Rust.

The robot is a swerve drive with a fuel launcher, hood, intake, two indexers, Limelight vision and LEDs. It runs on a roboRIO 2.

## Quick start

Run everything from the `rust/` folder. Full steps for Linux, Windows and macOS are in [docs/setup.md](docs/setup.md).

1. Install Rust with [rustup](https://rustup.rs). The right Rust version installs itself the first time you run `cargo` in `rust/`.
2. `cargo xtask fetch-natives` downloads the WPILib and CTRE libraries for your computer.
3. `cargo xtask sim` runs the robot in simulation and opens the Sim GUI. Add `--ds` for the driver station socket, or `--no-gui` for no window.
4. `cargo xtask gate` checks formatting, lints and tests. Run it before every commit.
5. `cargo xtask build-roborio` builds the program for the roboRIO.
6. `cargo xtask deploy` copies it to the roboRIO.

Deploying to a real roboRIO has not been tested yet. Neither has `xtask` on Windows or macOS. See [Setup](docs/setup.md#what-has-not-been-tested).

## Repository structure

```text
.
├── rust/                        Cargo workspace (all robot code)
│   ├── crates/
│   │   ├── wpilib/              HAL, driver station, robot loop, NetworkTables, dashboards
│   │   ├── commands/            command-based framework
│   │   ├── wpimath/             geometry, kinematics, pose estimator, PID, sims
│   │   ├── phoenix6/            TalonFX, CANcoder, Pigeon2, CANdle
│   │   ├── swerve/              swerve drivetrain over Phoenix 6
│   │   ├── pathplanner/         PathPlannerLib port
│   │   ├── telemetry/           wpilog writer and DogLog-style logging
│   │   ├── robot/               the robot program
│   │   └── golden/              test helper for fixtures
│   ├── xtask/                   fetch-natives, build-roborio, deploy, sim, gate
│   ├── fixtures/                frozen Java outputs used by tests
│   ├── tools/                   check-ctre-tables.py
│   ├── rust-toolchain.toml      pinned Rust version
│   └── THIRD_PARTY.md           ported code, versions and licenses
├── src/main/deploy/             files copied to the roboRIO
│   ├── pathplanner/             paths, autos, navgrid, settings
│   └── elastic-layout.json      Elastic dashboard layout
├── docs/                        team documentation
├── advantagescope-custom-assets/ robot models for AdvantageScope
├── images/                      two PNG tag images
├── AGENTS.md                    instructions for AI coding agents
└── WPILib-License.md
```

Inside `rust/crates/robot/src`:

```text
robot/            the 2026 robot: init, bindings, subsystems, auton, per-robot configs
rebuilt/          game logic (field, hub shift clock, shot calculator) and fuel and bump sims
spectrum_lib/     shared code: Mechanism, gamepads, LEDs, Limelight vision, sims
context.rs        robot-wide objects, set once at init
```

## Documentation

[docs/index.md](docs/index.md) lists every page.

Pages in `docs/frc-software-basics/` teach Java. They are background from before the Rust port.

## Dependencies

Rust crates in this repo replace the usual FRC libraries. They are ports of specific upstream versions, listed in [rust/THIRD_PARTY.md](rust/THIRD_PARTY.md).

| Replaces | Crate |
|---|---|
| WPILib | `wpilib`, `commands`, `wpimath` |
| CTRE Phoenix 6 | `phoenix6`, `swerve` |
| PathPlanner | `pathplanner` |
| DogLog | `telemetry` |
| maple-sim | `robot/src/spectrum_lib/swerve` |

Native code comes only through the WPILib and CTRE C libraries, which `cargo xtask fetch-natives` downloads.
