# Dependencies overview

*Audience: reference. No prerequisites.*

The robot program is Rust. It does not use the Java libraries any more. Crates in this repo replace them. Each one is a port of a specific upstream version, so behavior matches what the team used before the Rust port.

## Where the versions live

| What | Version | Where it is written |
|---|---|---|
| WPILib (HAL, NetworkTables, command framework, math) | 2026.2.1 | `rust/THIRD_PARTY.md`, `rust/xtask/src/fetch.rs` |
| CTRE Phoenix 6 | 26.3.0 | `rust/THIRD_PARTY.md`, `rust/xtask/src/fetch.rs` |
| PathPlannerLib | 2026.1.2 | `rust/THIRD_PARTY.md` |
| DogLog | 2026.5.0 | `rust/THIRD_PARTY.md` |
| maple-sim | v0.4.0-beta | `rust/THIRD_PARTY.md` |
| Limelight helper library | 1.14 | `rust/THIRD_PARTY.md` |
| roboRIO C toolchain | opensdk `v2025-2`, GCC 12.1.0 | `rust/xtask/src/fetch.rs` |
| Rust | pinned | `rust/rust-toolchain.toml` |

`rust/THIRD_PARTY.md` also lists the license of each ported source and the file that holds it. Ported code keeps its license.

External Rust crates (`nalgebra`, `libloading`, `serde`, `ureq` and others) are in each crate's `Cargo.toml`. Exact versions are in `rust/Cargo.lock`.

## Two kinds of dependency

**Natives.** The WPILib and CTRE shared libraries are real C code. `cargo xtask fetch-natives` downloads them into `rust/native/<platform>/`. The Rust crates load them at runtime with `libloading`, through the C function signatures in the vendors' headers. So the libraries need to be on the library path when the robot runs. `cargo xtask sim` and `cargo xtask deploy` handle that.

**Ports.** WPILib's Java command framework, WPILib math, PathPlannerLib, DogLog and maple-sim have no C API we can call. They were rewritten in Rust, with tests that compare against the Java output where they can. Frozen results are in `rust/fixtures/`.

## Updating a version

A version change is a behavior change. Do it in its own pull request.

1. Change the version constant in `rust/xtask/src/fetch.rs`, and fetch the natives again.
2. For a CTRE change, compare the new headers and Java API with `crates/phoenix6`. After you change the generated tables, run `python3 tools/check-ctre-tables.py` from `rust/`. The C layer is not a documented CTRE API, so keep the version pinned.
3. Update `rust/THIRD_PARTY.md`.
4. Run `cargo xtask gate` and the [simulation](../tools/simulation.md).

## Adding a library

Add only what the robot uses. Prefer pure Rust. Put the dependency in the crate that needs it. Add every ported source to `rust/THIRD_PARTY.md`, and keep its license.

For the surrounding subsystems, start with [2026 season specific](../other-guides/2026-season-specific.md).
