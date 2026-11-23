# Build tools and editor setup

*Audience: reference. Assumes you have read [Setup](../setup.md).*

The build is plain Cargo. There is one workspace file, `rust/Cargo.toml`, and one pinned toolchain, `rust/rust-toolchain.toml`. [Cargo xtask](cargo-xtask.md) covers the extra tasks.

## rustfmt

rustfmt formats the code. There is no config file, so it uses the defaults. `cargo xtask gate` fails on unformatted code. Fix it with:

```text
cargo fmt --all
```

Run that before you save your work to git. Format-only changes belong in the same change as the code they touch.

## clippy

clippy finds common mistakes. The workspace turns on the `clippy::all` group in `rust/Cargo.toml`, and `cargo xtask gate` runs `cargo clippy --workspace --all-targets -- -D warnings`, so any warning fails the check. Read the message. It usually says what to write instead. If you must silence a lint, use a narrow `#[allow(...)]` on one item, with a short comment that says why. `unsafe_op_in_unsafe_fn` is also set to deny for the whole workspace.

## Tests

`cargo test --workspace` runs every test. Each crate has unit tests in its source files, and some have a `tests/` folder.

Ported math is checked against frozen Java outputs. The `golden` crate loads `rust/fixtures/<name>.json`, which holds inputs and the Java results for them. The Java generator is gone, so do not regenerate fixtures. If a golden test fails, either the Rust code is wrong, or the behavior is changing on purpose and needs its own change.

## The gate

`cargo xtask gate` runs the three checks above in order and stops at the first failure. Run it before every commit. See [Code style](../coding-conventions/code-style.md).

## BuildConstants

`rust/crates/robot/build.rs` writes `build_constants.rs` on each build, with the build time and the git branch, commit and dirty flag. The robot publishes them as `BuildConstants/*` at startup. They tell you which build is on the robot, and which build wrote a log. Without git the git fields read "unknown". Do not write these values by hand.

## Documentation of the code

`cargo doc --workspace --no-deps --open` builds HTML from the doc comments and opens it. See [Documentation and comments](../coding-conventions/documentation-and-comments.md).

## Generated tables

The tables in `crates/phoenix6` (SPN numbers, enums, config defaults) are generated from CTRE's API. After you change them, run `python3 tools/check-ctre-tables.py` from `rust/`.

## Editor setup

Install these in VS Code:

* **rust-analyzer** (`rust-lang.rust-analyzer`): completion, go to definition, inline errors. The repo's `.vscode/settings.json` points it at `rust/Cargo.toml` and uses clippy for its checks.
* **Error Lens** (`usernamehw.errorlens`): shows errors next to the code.
* **GitLens** (`eamodio.gitlens`): shows who last changed each line.
* **GitHub Pull Requests and Issues** (`github.vscode-pull-request-github`): review in the editor.

If rust-analyzer shows errors on code that builds, run **rust-analyzer: Restart server** from the Command Palette. A stale cache after a branch switch looks like a compile error that is not there.

## See also

[Setup](../setup.md), [Cargo xtask](cargo-xtask.md), [Code style](../coding-conventions/code-style.md).
