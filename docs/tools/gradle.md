# Gradle

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

[Gradle](https://gradle.org) is the build tool that runs everything: compiling Java, formatting, static analysis, deploys, and the simulator. WPILib's [GradleRIO](https://github.com/wpilibsuite/GradleRIO) plugin layers the FRC specific tasks on top.

Always run `./gradlew` (or `gradlew.bat` on Windows), never a globally installed `gradle`. The wrapper downloads the version pinned in `gradle/wrapper/gradle-wrapper.properties`, which is what keeps every student on the same build. A global Gradle silently overrides the wrapper and is the reason a change that builds for you fails for the next person.

## The commands you will actually run

|          Command          |                                               What it does                                               |
|---------------------------|----------------------------------------------------------------------------------------------------------|
| `./gradlew build`         | Compile, format, test, and analyze, then produce the deployable jar.                                     |
| `./gradlew clean`         | Delete `build/` so the next build starts from scratch.                                                   |
| `./gradlew clean build`   | Both, for when something stale is causing errors that make no sense.                                     |
| `./gradlew deploy`        | Build and deploy to the connected roboRIO. The team number comes from `.wpilib/wpilib_preferences.json`. |
| `./gradlew simulateJava`  | Launch the sim. The VSCode `WPILib: Simulate Robot Code` command is usually faster.                      |
| `./gradlew spotlessApply` | Rewrite the source in place to the project style.                                                        |
| `./gradlew spotlessCheck` | Report formatting problems without writing. This is what CI runs.                                        |
| `./gradlew spotbugsMain`  | Run static analysis. HTML report at `build/reports/spotbugs.html`.                                       |
| `./gradlew javadoc`       | Generate JavaDoc HTML into `build/docs/javadoc/`.                                                        |
| `./gradlew tasks`         | List every task, including the ones not documented here.                                                 |

You can chain them. `./gradlew clean build deploy` cleans, builds, and deploys in one go.

## Formatting happens during the build

`./gradlew build` rewrites your files before it compiles them, Java and Markdown both. Two consequences worth internalizing:

- If a build fails on a format violation, run `./gradlew spotlessApply` and commit the result. That is usually the entire fix. CI runs `spotlessCheck`, which reports and does not write, so an unformatted file is a red check rather than a silent local change.
- Because Markdown is formatted too, never hand-align a table in `docs/`. The next build reflows it and the reflow lands in your diff next to your real change.

To exempt a region, such as a hand-aligned matrix or a generated table, wrap it in `// spotless:off` and `// spotless:on`.

## When things go wrong

A Java version error from `./gradlew` means the active JDK is not 17, and the message does not always say so. Confirm with `java -version`, then switch with SDKMAN (`sdk list java | grep tem`, then `sdk use java <latest-17.x.x-tem>`) or through your IDE's Java runtime settings. [Setup](../setup.md) covers the one-time install.

A vendor jar that will not download is usually a transient network problem. `./gradlew --refresh-dependencies` retries. If that does not work, delete that vendor's directory under `~/.gradle/caches/modules-2/files-2.1/` and try again.

`src/main/java/frc/robot/BuildConstants.java` is generated. gversion rewrites it on every `compileJava` with the build time and the Git state, so never edit it and never commit a hand change.

After bumping GradleRIO or a vendor library in `build.gradle`, re-run **Manage Vendor Libraries, then Check for Updates** in the WPILib VSCode extension. The `vendordeps/` JSON files have to match what the build declares or you get a version conflict that reads like a corrupt jar.

## Static analysis

`./gradlew build` runs SpotBugs and fails on any finding, which is intentional. Read `build/reports/spotbugs.html`. The suppressions live in [`excludeFilter-spotbugs.xml`](../../excludeFilter-spotbugs.xml), including several for the vendored Limelight glue code we do not own, so read that file before adding to it.

## See also

[Build Tools and Other Development Utilities](build-tools.md) for what Spotless, SpotBugs, and Lombok actually do. [Setup](../setup.md) for one-time environment work.
