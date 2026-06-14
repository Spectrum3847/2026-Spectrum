# Code Style

*Audience: Reference. No prerequisites.*

We use the [Android Open Source Project (AOSP) coding standards](https://source.android.com/docs/setup/contribute/code-style) as our baseline. Spotless enforces formatting at every `./gradlew build`, so this page is mostly about the parts a formatter can't enforce — naming, structure, judgment calls.

## Spotless Does the Mechanical Work

[`build.gradle`](../../build.gradle) wires `compileJava` to `spotlessApply`, so every local build reformats your `.java`, `.gradle`, `.xml`, and `.md` files using `googleJavaFormat("1.15.0").aosp()` and friends. There's no need to manually format anything — just `./gradlew build` and Spotless tidies up. See [Build Tools](../tools/build-tools.md) for the full Spotless story.

## The Spirit, Borrowed from AOSP

> "One of the simplest rules is BE CONSISTENT. If you're editing code, take a few minutes to look at the surrounding code and determine its style. … The point of having style guidelines is to have a common vocabulary of coding, so readers can concentrate on what you're saying, rather than on how you're saying it."

If you're touching `Launcher.java`, look at how the rest of that file is organized and match it. If `Hood.java` uses a slightly different pattern for the same job, fix the inconsistency in a separate commit so the diff is reviewable on its own.

## Naming

* **Classes / Interfaces:** `UpperCamelCase` — `Launcher`, `LauncherConfig`, `RobotStates`.
* **Methods / variables:** `lowerCamelCase` — `getVelocityRPM()`, `kPSlot0`.
* **Constants:** `UPPER_SNAKE_CASE` *only* for true compile-time constants that can never change (`Math.PI`, the `MAX_JAVA_HEAP_SIZE_MB` in `build.gradle`). Anything tunable — even something like `WHEEL_BASE_INCHES` that varies between robots — goes in a `*Config` class as a regular field, not a constant.
* **Enums:** enum *names* are `UpperCamelCase`; their *values* are `UPPER_SNAKE_CASE`. `State.LAUNCH_WITH_SQUEEZE`, `Telemetry.Fault.CAMERA_OFFLINE`.

Acronyms get treated as words. `PidConfig`, not `PIDConfig`. `Rpm` in compound names, not `RPM`. The one exception is when the acronym *is* the whole identifier (a constant `RPM`).

## No `m_` or `_` Prefixes

If you see `m_someField`, it's from a library we imported and didn't rewrite. Don't add new ones, and feel free to rename them away when touching a file. Field-vs-local disambiguation belongs in `this.field = field` if needed, not in the name.

## Indentation

4 spaces, never tabs. Spotless reformats anything that drifts. If you're pasting code from WPILib examples or another team's repo, run `./gradlew spotlessApply` immediately — it will fix indentation and trailing whitespace in one pass.

## Imports

Spotless's `removeUnusedImports()` strips dead imports on every build. Don't worry about cleaning these by hand. Don't use star imports (`import com.ctre.phoenix6.*`) — the formatter expands them.

## File Organization

For a subsystem file like `Launcher.java`, the conventional order is:

1. Inner `Config` class (with `@Getter`/`@Setter` fields).
2. Fields (motors, sensors, suppliers, triggers).
3. Constructor.
4. `setupStates()`, `setupDefaultCommand()`, `periodic()`.
5. Public API used by `*States` (getters, setpoint setters, `At/Above/Below` trigger helpers).
6. Private helpers.

This isn't a hard rule, but every existing subsystem follows it, and a reader skimming the file knows where to look. See [Class Generation](class-generation.md) for the why behind this layout.

## Line Length

`googleJavaFormat` wraps at 100 columns. If a method signature or chained call ends up wrapping in an ugly way, that's usually a sign the code wants to be restructured — pull out a local, split a chained `Commands.sequence(...)` across multiple lines with one command per line, etc. Don't fight the formatter; let it tell you where things are too dense.

## When to Diverge

Spotless honors `// spotless:off` / `// spotless:on` markers. Use them sparingly — typically for a hand-aligned constant table or a multi-line math expression where the alignment is the readability. Document *why* you're disabling Spotless in a one-line comment above the `off` marker. If a future reader can't tell why the section is special, they'll either re-enable it (and lose the alignment) or worse, copy the suppression elsewhere.
