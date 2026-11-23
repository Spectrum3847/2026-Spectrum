# Code style

*Audience: reference. No prerequisites.*

Tools check the mechanical parts: rustfmt for layout, clippy for common mistakes. This page covers what they cannot check: naming, structure and judgment calls. The rules that govern the code are also in `AGENTS.md`, under "Rules for the code". If the two ever differ, `AGENTS.md` wins.

## The gate

Run this from `rust/` before every commit:

```text
cargo xtask gate
```

It runs `cargo fmt --all --check`, then `cargo clippy --workspace --all-targets -- -D warnings`, then `cargo test --workspace`. A clippy warning fails the gate. To fix formatting, run `cargo fmt --all`. See [Build tools](../tools/build-tools.md).

## Be consistent

When you edit a file, look at the code around yours and match it. If two files do the same job in different ways, fix that in its own change, so each diff is easy to review.

## Naming

Rust has standard names. clippy and rustc warn when you break them.

* **Types and traits:** `UpperCamelCase`. Example: `HoodConfig`, `SuperStructure`.
* **Functions, methods, variables, modules and files:** `snake_case`. Example: `set_wanted_state`, `super_structure.rs`.
* **Constants and statics:** `UPPER_SNAKE_CASE`. Example: `SECONDS_TO_SQUEEZE`.
* **Enum variants:** `UpperCamelCase`. Example: `WantedSuperState::IntakeFuel`. Enums that show up in logs implement `EnumName`, which gives the text the dashboard and logs show, such as `INTAKE_FUEL`.

Acronyms count as words: `PidConstants`, not `PIDConstants`.

Put the unit in the name when a number has one: `hood_angle_degrees`, `position_kp`, `velocity_rps`, `max_speed_mps`. Library crates use SI units (meters, radians, seconds). Phoenix values use rotations and rotations per second (`*_rot`, `*_rps`). Robot code keeps the units the old Java used (RPM, rotations, degrees). Convert at the edge between two kinds.

Tunable values, even ones that look like constants, go in a subsystem's config struct, not in a `const`. A `const` is for a value that never changes, such as a mathematical constant.

## Rules for the code

These come from `AGENTS.md`. Each one prevents a real bug.

* **Threads.** Robot setup, commands, the scheduler and dashboards run on the loop thread only. They use `Rc` and `RefCell`. Code on other threads (swerve odometry, notifiers, telemetry) uses only `Send + Sync` types: telemetry functions, Phoenix signals, atomics and `Mutex`. A dashboard call from another thread panics.
* **Borrows.** Subsystems are `Rc<T>` with `&self` methods. Hold a `RefCell` borrow only inside one method. Never hold it across a call into another subsystem, the scheduler, a command or a callback. That panics with "already borrowed".
* **Native calls.** Every `extern` signature must match the C header exactly. `#[repr(C)]` structs carry size checks for 64-bit and the 32-bit roboRIO. Put a `// SAFETY:` comment on every `unsafe` call. Register a stop function with `wpilib::shutdown` for any thread or notifier that touches native code.
* **CTRE.** The C layer is not a documented CTRE API, so the version stays pinned. After you change the generated tables in `crates/phoenix6`, run `python3 tools/check-ctre-tables.py`. Never disassemble CTRE binaries.
* **Licenses.** Ported code keeps its license. List every ported source in `rust/THIRD_PARTY.md`.
* **Behavior.** The robot must behave as the Java robot did. A behavior change gets its own commit, never a side effect of a refactor.
* **Tests.** Every calculation has a test. Ported math is checked against `rust/fixtures/`.
* **Scope.** Port or add only what the robot uses.

## `Java:` comments

A comment that starts with `Java:` marks behavior kept from the Java robot that looks wrong. Do not fix it as part of another change. Fix each one in its own commit, and remove the comment.

## Pasting code

When you copy from WPILib examples or another team, run `cargo fmt --all` right away. Read the code before you keep it. Use plain `use` lines for what you need. Avoid `use something::*`, which hides where a name comes from.

## File order

Subsystem files follow one order, so a reader knows where to look. See [Adding a subsystem](adding-a-subsystem.md).

## Line length

rustfmt wraps at 100 columns. If a call wraps in an ugly way, the code usually wants a local variable, or a split into smaller functions. Do not fight the formatter.

## When to diverge

Use `#[rustfmt::skip]` on one item when hand-aligned numbers are clearer, and use `#[allow(clippy::...)]` on one item when a lint is wrong for that code. Add a one-line comment that says why. A reader should not have to guess.
