# Dependencies overview

*Audience: Reference. No prerequisites.*

Each page here covers one third party library from our side of it: the conventions we settled on, the recipes, and the traps we have already hit. It is not a summary of what the library does. For that, read the library's own docs, linked at the bottom of each page.

## Where the versions live

`vendordeps/` is the only place a version number is written down. Each JSON there pins one dependency, and GradleRIO reads them directly, so there is no WPILib-core JSON to go looking for: WPILib itself arrives from `build.gradle`.

To change a version, swap the JSON through WPILib VSCode's `Manage Vendor Libraries`, then run `./gradlew build`.

## Adding a new library

1. Use `Manage Vendor Libraries → Install new library (online)` in WPILib VSCode and paste the vendor's URL. The JSON lands in `vendordeps/`.
2. Run `./gradlew build` so GradleRIO fetches the jar and the native bits.
3. Add the JavaDoc link base to `javadoc.options.setLinks([...])` in `build.gradle`. This is the step people forget, and without it every `{@link}` to that library comes out as plain text in our generated docs. Check for an existing base first, because `build.gradle` carries entries for libraries we do not depend on, including REV Robotics and Phoenix v5. If you add a REV or Phoenix v5 import, you also need the matching vendordep JSON or the code will not compile.
4. Write a page in this directory covering how we use it, and link it from [`index.md`](../index.md).

For the surrounding subsystems before any one library, start with [2026 season specific](../other-guides/2026-season-specific.md).
