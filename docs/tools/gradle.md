# Gradle

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

[Gradle](https://gradle.org) is the build tool that runs everything: compiling Java, formatting, static analysis, deploys, and the simulator. WPILib's [GradleRIO](https://github.com/wpilibsuite/GradleRIO) plugin layers FRC-specific tasks on top.

The version is pinned in `gradle/wrapper/gradle-wrapper.properties`, and `./gradlew` (or `gradlew.bat` on Windows) is the wrapper script that downloads it. **Always use `./gradlew`**, not a globally installed `gradle` — the wrapper guarantees the whole team builds with the same version, which matters more than you'd think.

## The Commands You'll Actually Run

| Command | What it does |
| --- | --- |
| `./gradlew build` | Compile, run Spotless, run SpotBugs, run tests, produce the deployable jar. |
| `./gradlew clean` | Delete `build/` so the next build is from scratch. |
| `./gradlew clean build` | Both of the above, useful when something stale is causing weird errors. |
| `./gradlew deploy` | Build and deploy to the connected roboRIO. Team number comes from `.wpilib/wpilib_preferences.json`. |
| `./gradlew simulateJava` | Launch the WPILib GUI simulator. (The VSCode `WPILib: Simulate Robot Code` command is usually faster.) |
| `./gradlew javadoc` | Generate JavaDoc HTML into `build/docs/javadoc/`. |
| `./gradlew spotlessApply` | Apply the AOSP code style across every `.java`, `.gradle`, `.xml`, and `.md` file. Runs automatically on `compileJava`. |
| `./gradlew spotlessCheck` | Verify formatting without rewriting — what CI runs. |
| `./gradlew spotbugsMain` | Run SpotBugs static analysis. The HTML report lands at `build/reports/spotbugs.html`. |
| `./gradlew tasks` | List every task, including ones not documented here. |

You can chain them. `./gradlew clean build deploy` will clean, build, and deploy in one go.

## Things `build.gradle` Sets Up

A few specifics that are easy to miss if you don't open `build.gradle`:

GradleRIO is at `2026.2.1` and targets the 2026 WPILib release. Bumping it means you also need to re-run `Manage Vendor Libraries → Check for Updates` in WPILib VSCode so the vendordeps line up.

Java 17 is enforced via `sourceCompatibility`/`targetCompatibility`. Anything else will fail at compile, with a not-always-obvious error message. See [Setup](../setup.md) for how to install Temurin 17.

Spotless is wired to `compileJava`, so `./gradlew build` reformats your code in place using `googleJavaFormat("1.15.0").aosp()`. If you need to opt a region out (say, a hand-aligned matrix), wrap it in `// spotless:off` / `// spotless:on`.

SpotBugs runs against the main source set with the exclude list in `excludeFilter-spotbugs.xml`. The HTML report (`build/reports/spotbugs.html`) is the easiest way to triage findings.

`gversion` regenerates `frc/robot/BuildConstants.java` on every `compileJava`, baking in build time and Git state so the deployed code can log which build it is.

Lombok annotation processing comes through `io.freefair.lombok`. Generated getters and setters exist at compile time only — don't try to commit them.

The JavaDoc task is configured with external `setLinks(...)` for WPILib, Phoenix 6, PathPlanner, DogLog, MapleSim, PhotonVision, and the Java 17 stdlib. That's why our generated docs cross-link cleanly. The list of dependencies on each per-library page in [Dependencies](../dependencies/overview.md) lines up with this.

## When Things Go Wrong

The first instinct, more often than not, is `./gradlew clean build`. If that doesn't fix it:

A Java-version error from `./gradlew` means your active JDK isn't 17. Run `java -version` to confirm, then switch via SDKMAN (`sdk list java | grep tem` then `sdk use java <latest-17.x.x-tem>`) or your IDE's Java runtime settings.

Spotless complaining about format violations: run `./gradlew spotlessApply` and commit the result. CI runs `spotlessCheck`, which doesn't write — `spotlessApply` does.

A vendor jar failing to download usually means a transient network thing. `--refresh-dependencies` retries; if that doesn't work, blow away `~/.gradle/caches/modules-2/files-2.1/<vendor>` and try again.

Deploy works for you but fails for a teammate: check that you're both on the same wrapper version with `./gradlew --version`. A globally installed Gradle will sometimes mask the wrapper.

## See Also

[Build Tools and Other Development Utilities](build-tools.md) for the broader Spotless/SpotBugs/Lombok overview. [Setup](../setup.md) for one-time environment work.
