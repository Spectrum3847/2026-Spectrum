# Setup

*Audience: everyone. Start here.*

This page gets you from nothing to a robot simulation running on your computer.

## What you need

* [Git](https://git-scm.com/).
* Rust, installed with [rustup](https://rustup.rs).
* A code editor. [VS Code](https://code.visualstudio.com/) with the rust-analyzer extension works well. The repo's `.vscode/settings.json` points rust-analyzer at `rust/Cargo.toml` and uses clippy for its checks.
* The [PathPlanner](https://pathplanner.dev) app, to edit autos.
* [Elastic](https://github.com/Gold872/elastic-dashboard) or [AdvantageScope](https://docs.advantagescope.org), to watch the robot.
* To deploy: an `ssh` client. Windows needs the OpenSSH client feature turned on.

## Clone the repository

```text
git clone https://github.com/spectrum3847/2026-Spectrum.git
cd 2026-Spectrum
```

If you prefer a GUI, [GitHub Desktop](https://desktop.github.com/download/) works.

## Install Rust

Install rustup, then close and reopen your terminal.

* **Linux.** Run the install command from [rustup.rs](https://rustup.rs). You also need a C compiler and linker. On Debian or Ubuntu, `sudo apt install build-essential`. On Fedora, `sudo dnf install gcc`.
* **macOS.** Run `xcode-select --install`, then the install command from [rustup.rs](https://rustup.rs).
* **Windows.** Download and run `rustup-init.exe` from [rustup.rs](https://rustup.rs). It tells you if you need the Visual Studio C++ build tools, and links to them.

You do not pick a Rust version. `rust/rust-toolchain.toml` pins it. The first `cargo` command you run inside `rust/` downloads that version, with rustfmt, clippy and the roboRIO build target.

## Download the native libraries

The robot calls WPILib and CTRE code that is not written in Rust. Download those libraries once:

```text
cd rust
cargo xtask fetch-natives
```

This puts the files for your computer in `rust/native/<platform>/`. The folder is not committed. [Cargo xtask](tools/cargo-xtask.md) lists the platforms.

## Run the simulation

```text
cargo xtask sim
```

The first run compiles everything, so it takes a few minutes. The Sim GUI opens, and the robot's NetworkTables server starts too. Open Elastic or AdvantageScope and connect to `localhost` if you want a dashboard. [Simulation](tools/simulation.md) covers what you see.

On Linux the GUI needs a display and the `libX11` and `libGL` libraries. Over SSH or on a computer with no display, run `cargo xtask sim --no-gui`.

### Enabling the robot in simulation

The robot starts disabled. In the Sim GUI, pick a mode in the Robot State window and click Enable.

To use the FRC Driver Station program or another driver station client instead, run:

```text
cargo xtask sim --no-gui --ds
```

`--ds` loads WPILib's `halsim_ds_socket` extension, which opens the socket those clients talk to. `xtask` sets the `HALSIM_EXTENSIONS` variable for you, so do not set it by hand. [Cargo xtask](tools/cargo-xtask.md#sim) lists the flags. WPILib's [simulation docs](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/index.html) describe the protocol and clients.

## Check your work

```text
cargo xtask gate
```

This runs `cargo fmt --all --check`, clippy with warnings as errors, and all tests, on Linux, Windows and macOS. See [Code style](coding-conventions/code-style.md).

## Build and deploy

```text
cargo xtask build-roborio
cargo xtask deploy
```

The first `build-roborio` also downloads the roboRIO C toolchain into `rust/native/toolchain`. Run `cargo xtask fetch-natives linuxathena` before the first deploy. `deploy` needs your computer on the robot network and uses `ssh` as the `admin` user, whose password is empty. The default host is `10.38.47.2`. Over USB use `cargo xtask deploy 172.22.11.2`. See [Cargo xtask](tools/cargo-xtask.md).

## What has not been tested

* `cargo xtask deploy` has not run on a real roboRIO yet.
* `cargo xtask` has not run on Windows or macOS yet. Their code paths have unit tests only.
* The FRC Driver Station program has not been tried against the simulation. The socket was tested on Linux with a small test client.
* The Sim GUI was tested on Linux x86-64 under a virtual display only. Joysticks, Windows and macOS are untested.

If you try one of these, write down what happened in this page.

## Next steps

* [Table of contents](index.md)
* New to programming? The pages in [frc-software-basics](frc-software-basics/applied-to-frc.md) teach Java. They are background from before the Rust port. For Rust itself, read [The Rust Book](https://doc.rust-lang.org/book/).
