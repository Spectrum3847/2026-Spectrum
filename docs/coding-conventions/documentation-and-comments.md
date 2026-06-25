# Documentation and Comments

*Audience: Reference. Assumes you've read [Code Style](code-style.md).*

The goal: someone joining the team next month should be able to read this codebase top-to-bottom and form an accurate mental model without you on a call to translate. Comments and JavaDoc are the most useful when the code's *what* is clear from the names but the *why* isn't.

## When to Comment

Default to writing no comment. Add one when the *why* is non-obvious:

* A hidden constraint (`// PathPlanner expects angles in radians, not degrees`)
* A workaround for a specific bug (`// Phoenix 6 < 26.1.3 returns NaN for unconfigured slots — see CTRE issue #471`)
* A subtle invariant (`// must be called before configurePID() — order matters`)
* Behavior that would surprise a reader (`// LED priority of -1 is the lowest — it only runs when no higher-priority pattern is active`)

Don't write comments that just restate the code. `// loop over modules` above `for (var module : modules)` is noise. Names like `tightenMagnetOffset` or `rejectAmbiguousTagEstimate` already say what the code does.

## JavaDoc

Public methods on `*States` classes and `*Config` inner classes are the API the rest of the robot depends on. Those deserve at least a one-line JavaDoc, especially when units or ranges are non-obvious:

```java
/**
 * Sets the launcher target speed.
 *
 * @param rpm flywheel revolutions per minute, 0-6000
 */
public Command setSpeed(double rpm) { ... }
```

Two specific habits that pay off:

* **Document units in `@param`/`@return`** — `meters`, `radians`, `volts`, `RPM`. Off-by-π errors happen because someone assumed the wrong unit.
* **Note thread/loop expectations when they matter** — "must be called from `periodic()`", "safe to call while disabled", etc.

`./gradlew javadoc` renders HTML to `build/docs/javadoc/`. `Xdoclint:none` is set, so missing JavaDocs don't fail the build — but they do fail readers later. See [Build Tools](../tools/build-tools.md) for the JavaDoc task config and cross-link setup.

## TODO Comments

Use `// TODO:` for code that works but you know wants follow-up. Include enough context that future-you can pick it up cold:

```java
// TODO: replace hardcoded 12V with battery-compensated supply voltage
```

VSCode surfaces TODOs in the Problems panel, which is the closest thing to a built-in tracker. For anything bigger than a one-line follow-up, open a GitHub Issue and reference it from the TODO (`// TODO(#142): integrate PhotonVision pipeline`). The Issue persists; the TODO might get refactored away.

Don't use `// TODO` for known broken code — fix it or open an Issue and `@SuppressWarnings` it intentionally. A TODO on a bug becomes a landmine for the next person.

## What Not to Write

* **Don't reference current context.** `// added for the launching state` or `// fix for issue #99` rots — the issue gets closed, the state gets renamed, and the comment is wrong forever. Put that in the PR description and commit message.
* **Don't write block-comment banners.** `/* === Helpers === */` looks tidy but adds nothing.
* **Don't write multi-paragraph docstrings** unless the method really is that subtle. If your comment is longer than the method, the method probably wants to be split.
* **Don't commit commented-out code.** Delete it; git has the history. The only exception is when explicitly marking a WIP region with a clear `// TODO: re-enable when X` — and even then, prefer a stub or `throw new UnsupportedOperationException`.

## Cross-Referencing Code From Docs

This documentation set links into source via relative paths from `docs/` — e.g., `[Launcher.java](../../src/main/java/frc/robot/launcher/Launcher.java)`. Keep those alive:

* When you rename a class, grep the `docs/` tree for the old name (`grep -r "OldClassName" docs/`) and fix references.
* When you delete a class, decide whether the doc reference is still useful (might point to a successor) or should be removed.
* When you move a file, the relative path needs updating.

There's no automated link checker on this repo — if you break a link, nobody notices until someone clicks. Periodic grep-and-fix is the only enforcement.

## External Resources

Don't reinvent. Chief Delphi, the [WPILib docs](https://docs.wpilib.org), and CTRE / Limelight / PathPlanner docs cover everything except the team-specific patterns. This doc set is specifically *not* trying to replace those — its job is to capture how *we* assemble those pieces. If the right answer is "go read the WPILib docs on `SubsystemBase`," that's the right answer; link to it.

## See Also

* [Code Style](code-style.md) for naming and formatting.
* [Class Generation](class-generation.md) for the structural conventions JavaDocs live around.
* [Build Tools](../tools/build-tools.md) for the JavaDoc Gradle task.
