# Documentation and comments

*Audience: reference. Assumes you have read [Code style](code-style.md).*

Someone who joins next month should be able to read this code base and understand it without you on a call. Comments help most when the code shows what happens, but not why.

The code is the record of what the robot does. A comment never replaces it. A comment that says something the code no longer does is worse than no comment. A comment earns its place by saying what the code cannot: the reason, the constraint, the cause of a choice.

## When to comment

Write no comment by default. Add one when the why is not obvious. Keep it to a few words or a few lines.

* A hidden constraint: `// PathPlanner expects radians`.
* A workaround: `// Java: the staleness check reads the MT1 capture time, not the MT2 one`.
* An order that matters: `// Set the context item before the sims are built`.
* Behavior that would surprise a reader.

Do not restate the code. `// loop over modules` above a `for` loop is noise. A clear name does that job.

Do not describe the old code, an issue number or the current change. "Used to" and "no longer" comments go stale. Put that in the commit message. When behavior changes, update or delete the comment next to it.

## Doc comments

Rust has two doc comment forms:

* `///` documents the item below it.
* `//!` documents the module or crate it sits in. It goes at the top of the file.

Public items that other code uses should have at least one line. The top of each file has a `//!` line that says what the file is for, as in `hood.rs`: "Sets the hood angle that aims the fuel shot."

Two habits that help:

* **Say the unit.** `meters`, `radians`, `volts`, `RPM`, `rotations`. Wrong-unit errors are common. Config fields get their unit in the doc comment or the name, such as `hood_x` in meters.
* **Say the thread or loop rule when it matters.** For example "Call on the robot loop thread" or "Safe to call from any thread".

A shape to aim for:

```rust
/// Position in rotations, via Motion Magic in torque current FOC (requires Phoenix Pro).
pub fn set_mm_position_foc(&self, rotations: impl Fn() -> f64) { ... }
```

Link to other items with `[`name`]`. `cargo doc` turns them into links, and warns if they break.

Run `cargo doc --workspace --no-deps --open` to see the result.

## Safety comments

Every `unsafe` call needs a `// SAFETY:` comment that says why the call is sound. For example, `// SAFETY: no arguments, no preconditions.` See [Code style](code-style.md).

## TODO comments

Use `// TODO:` for code that works but needs follow-up. Give enough detail that someone can pick it up cold. For anything bigger than one line, open a GitHub issue and put its number in the TODO: `// TODO(#142): add the game piece detection pipeline`. Do not use a TODO for known broken code. Fix it, or open an issue.

## What not to write

* **Banners.** `// === Helpers ===` adds nothing.
* **Long doc comments** unless the item is that subtle. If the comment is longer than the code, split the code.
* **Commented-out code.** Delete it. Git has the history.

## Linking docs to code

The pages in `docs/` link to source files with relative paths. Keep them working:

* When you rename a file or type, search `docs/` for the old name (`grep -r "OldName" docs/`) and fix it.
* When you delete a file, remove the reference or point to its replacement.
* When you move a file, update the relative path.

Nothing checks the links for you. If you break one, nobody notices until someone clicks it.

## External resources

Do not repeat the [WPILib docs](https://docs.wpilib.org), CTRE, Limelight or PathPlanner docs. These pages cover how we put those pieces together. If the answer is "read the WPILib docs on commands", link to them.

## See also

* [Code style](code-style.md) for naming and layout.
* [Adding a subsystem](adding-a-subsystem.md) for the structure the docs sit around.
* [Build tools](../tools/build-tools.md) for `cargo doc`.
