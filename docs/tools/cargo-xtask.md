# Cargo xtask

*Audience: reference. Assumes you have read [Setup](../setup.md).*

`xtask` is a small Rust program in `rust/xtask` that runs the tasks cargo does not have: downloading libraries, cross-compiling for the roboRIO, deploying and simulating. Run it from `rust/` as `cargo xtask <task>`. The alias is in `rust/.cargo/config.toml`.

Run `cargo xtask` with no task to print the usage text.

## Tasks

| Command | What it does |
|---|---|
| `cargo xtask fetch-natives [platform]` | Downloads the WPILib and CTRE libraries into `rust/native/<platform>/`. With no platform it uses your computer. |
| `cargo xtask sim [name] [-p package] [--ds] [--no-gui]` | Runs the robot in simulation with the Sim GUI. With a name, it runs that example instead. |
| `cargo xtask build-roborio [cargo args]` | Cross-compiles the robot for the roboRIO in release mode. |
| `cargo xtask deploy [host]` | Copies the program, libraries and `src/main/deploy/` to the roboRIO. |

### fetch-natives

Platforms: `linuxathena` (the roboRIO), `linuxarm64`, `linuxx86-64`, `windowsx86-64` and `osxuniversal`. Your computer is picked from your OS and CPU. A Windows ARM computer is not supported, so pass a platform.

Versions are constants at the top of `rust/xtask/src/fetch.rs`: WPILib `2026.2.1` and Phoenix `26.3.0`. For a desktop platform the task also downloads the WPILib `halsim_gui` and `halsim_ds_socket` extensions, `wpimath` (the GUI needs it) and the CTRE simulation libraries. For `linuxathena` it downloads only the libraries the roboRIO needs.

### sim

`sim` runs `cargo run -p robot` with `rust/native/<platform>/` added to the library search path (`LD_LIBRARY_PATH` on Linux, `DYLD_LIBRARY_PATH` on macOS, `PATH` on Windows). It stops with a message if you have not run `fetch-natives`.

`sim` also sets `HALSIM_EXTENSIONS`, the list of simulation extensions the HAL loads at startup:

| Flags | Extensions loaded |
|---|---|
| none | Sim GUI |
| `--ds` | Sim GUI and driver station socket |
| `--no-gui` | none |
| `--no-gui --ds` | driver station socket |

The Sim GUI needs a display and the system libraries `libX11` and `libGL`. On a computer with no display, use `--no-gui`. Any `HALSIM_EXTENSIONS` you set yourself is replaced.

* `cargo xtask sim hal_smoke -p wpilib` runs the example `hal_smoke` in the `wpilib` crate.
* `cargo xtask sim pathplanner_telemetry -p pathplanner` runs an example in another crate.

Examples are in each crate's `examples/` folder. See [Simulation](simulation.md) for enabling the robot.

### build-roborio

Downloads the roboRIO C toolchain (WPILib opensdk `v2025-2`, GCC `12.1.0`) into `rust/native/toolchain` on first use. It runs `rustup target add armv7-unknown-linux-gnueabi`, then builds with `cargo build --release --target armv7-unknown-linux-gnueabi -p robot`. Extra arguments go to cargo. The program lands in `rust/target/armv7-unknown-linux-gnueabi/release/robot`.

### deploy

Before you deploy, run `cargo xtask fetch-natives linuxathena` and `cargo xtask build-roborio`. Deploy then:

1. Packs the program, the libraries and `src/main/deploy/` into one archive.
2. Sends it over `ssh` as `admin@<host>`. The password is empty, so press Enter if ssh asks.
3. On the roboRIO, stops the robot program, installs the libraries in `/usr/local/frc/third-party/lib`, the program as `/home/lvuser/frcUserProgram` and the files in `/home/lvuser/deploy`, then starts the program again.

The default host is `10.38.47.2`. Over USB use `172.22.11.2`. If the roboRIO has "no app" mode on, deploy stops with a message. Turn it off with the imaging tool, or hold the User button for 5 seconds.

Deploy has not run on a real roboRIO yet. `xtask` has not run on Windows or macOS yet.

## Other commands you will use

| Command | What it does |
|---|---|
| `cargo xtask gate` | Format check, clippy with warnings as errors, all tests. |
| `cargo fmt --all` | Formats the code. |
| `cargo test --workspace` | Runs all tests. |
| `cargo test -p swerve` | Runs the tests of one crate. |

## When things go wrong

* **"missing native/..."**: run `cargo xtask fetch-natives`.
* **A download fails**: run the task again. It starts over for that platform.
* **A library will not load**: it is the wrong platform, or the version changed. Delete `rust/native/<platform>` and fetch again.
* **The first build takes long**: that is normal. Later builds only rebuild what changed.
