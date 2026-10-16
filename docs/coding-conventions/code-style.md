# Code Style

*Audience: Reference. No prerequisites.*

We use the [Android Open Source Project (AOSP) coding standards](https://source.android.com/docs/setup/contribute/code-style) as our baseline. Spotless enforces formatting on every `./gradlew build`, so this page is about the parts a formatter cannot enforce: naming, structure, and judgment calls. The formatter's own settings live in [`build.gradle`](../../build.gradle). Read them there, not here.

## Spotless does the mechanical work

`build.gradle` wires `compileJava` to `spotlessApply`, so every build reformats your source for you. There is never a reason to format by hand. If you paste code from a WPILib example or another team's repo, run `./gradlew build` (or `./gradlew spotlessApply`) and let it settle before you commit. See [Build Tools](../tools/build-tools.md) for the full Spotless story.

## The spirit, borrowed from AOSP

> "One of the simplest rules is BE CONSISTENT. If you're editing code, take a few minutes to look at the surrounding code and determine its style. … The point of having style guidelines is to have a common vocabulary of coding, so readers can concentrate on what you're saying, rather than on how you're saying it."

If you're touching `Launcher.java`, look at how the rest of that file is organized and match it. If `Hood.java` uses a slightly different pattern for the same job, fix the inconsistency in a separate commit so the diff is reviewable on its own.

## Naming

* **Classes / interfaces:** `UpperCamelCase`, such as `Launcher`, `LauncherConfig`, `SuperStructure`.
* **Methods and variables:** `lowerCamelCase`, such as `getVelocityRPM()`.
* **Constants:** `UPPER_SNAKE_CASE` *only* for true compile-time constants that can never change (`Math.PI`, `MAX_JAVA_HEAP_SIZE_MB` in `build.gradle`). Anything that differs between robots, or that we might want to change without recompiling the world, goes in a `*Config` class as an ordinary field. `SwerveConfig` is the clearest example: every one of its values is a lowercase field with a `@Getter`, not a constant.
* **Enums:** enum *names* are `UpperCamelCase`; their *values* are `UPPER_SNAKE_CASE`. `WantedSuperState.LAUNCH_WITH_SQUEEZE` is a value, so it is shouty, and `WantedSuperState` is a type, so it is not.

Acronyms get treated as words in type names, so `RpmLog` rather than `RPMLog`. Unit suffixes on getters keep their engineering spelling, so `getVelocityRPM()` rather than `getVelocityRpm()`. Pick whichever of the two your identifier actually is and stay consistent; the AOSP rule of matching the surrounding code settles the rest.

## No `m_` or `_` prefixes

If you see `m_someField`, it is from a library we imported and did not rewrite. Do not add new ones, and feel free to rename them away when you are already in the file. If a field and a local share a name, disambiguate with `this.field = field`, not with a prefix in the name.

## Imports

Do not write star imports. The formatter expands them, so a star import becomes a wall of single imports in the next diff, which is exactly the noise a reviewer did not ask for.

## File organization

For a subsystem file, the conventional order is:

1. Inner `Config` class, with its fields.
2. Fields: motors, sensors, suppliers, triggers.
3. Constructor.
4. The state machine: the `WantedState` and `SystemState` enums, `setWantedState(...)`, `handleStateTransition()`, `applyStates()`, `periodic()`.
5. Public API the rest of the robot calls: getters, setpoint setters, and the at/above/below trigger helpers.
6. Private helpers.

This is not a hard rule, but every existing subsystem follows it, so a reader skimming the file knows where to look before reading a word. See [Class Generation](class-generation.md) for the reasoning behind the layout.

## When the formatter wraps something ugly

`googleJavaFormat` wraps long lines wherever it likes, and a signature or a chained call that wraps badly is usually a sign the code wants restructuring rather than a sign the formatter is wrong. Pull out a local, or split a long `Commands.sequence(...)` so one command sits per line. Do not fight the formatter; read the wrapping as feedback.

## When to diverge

Spotless honors `// spotless:off` / `// spotless:on` markers. Use them sparingly, typically for a hand-aligned constant table or a multi-line math expression where the alignment is the readability. Document *why* you're disabling Spotless in a one-line comment above the `off` marker. If a future reader can't tell why the section is special, they'll either re-enable it (and lose the alignment) or worse, copy the suppression elsewhere.
