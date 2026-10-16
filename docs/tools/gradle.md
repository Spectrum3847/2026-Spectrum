# Gradle

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

[Gradle](https://gradle.org) is the build tool that runs everything: compiling Java, formatting, static analysis, deploys, and the simulator. WPILib's [GradleRIO](https://github.com/wpilibsuite/GradleRIO) plugin layers FRC-specific tasks on top, and `build.gradle` is where this project's configuration lives. Read that file for what is configured; this page is about running the build and what to do when it misbehaves.

## The commands you'll actually run

|          Command          |                                                      What it does                                                       |
|---------------------------|-------------------------------------------------------------------------------------------------------------------------|
| `./gradlew build`         | Compile, run Spotless, run SpotBugs, run tests, produce the deployable jar.                                             |
| `./gradlew clean`         | Delete `build/` so the next build is from scratch.                                                                      |
| `./gradlew clean build`   | Both of the above, useful when something stale is causing weird errors.                                                 |
| `./gradlew deploy`        | Build and deploy to the connected roboRIO. Team number comes from `.wpilib/wpilib_preferences.json`.                    |
| `./gradlew simulateJava`  | Launch the WPILib GUI simulator. (The VSCode `WPILib: Simulate Robot Code` command is usually faster.)                  |
| `./gradlew javadoc`       | Generate JavaDoc HTML into `build/docs/javadoc/`.                                                                       |
| `./gradlew spotlessApply` | Apply the AOSP code style across every `.java`, `.gradle`, `.xml`, and `.md` file. Runs automatically on `compileJava`. |
| `./gradlew spotlessCheck` | Verify formatting without rewriting: what CI runs.                                                                      |
| `./gradlew spotbugsMain`  | Run SpotBugs static analysis. The HTML report lands at `build/reports/spotbugs.html`.                                   |
| `./gradlew robotApp`      | Open the local robot app web UI on its home page.                                                                       |
| `./gradlew alignSwerve`   | Open the robot app on the swerve alignment page. See [Swerve Alignment](swerve-alignment.md).                           |
| `./gradlew tasks`         | List every task, including ones not documented here.                                                                    |

You can chain them. `./gradlew clean build deploy` will clean, build, and deploy in one go.

## Use the wrapper, not a global gradle

`./gradlew` (or `gradlew.bat` on Windows) is a wrapper script that downloads the exact Gradle version pinned in `gradle/wrapper/gradle-wrapper.properties`. Always use it. A globally installed `gradle` on your PATH will happily run a different version and produce failures that only the person with that version sees. Check what the wrapper resolves with `./gradlew --version` before you go blaming the code.

## Bumping a dependency

GradleRIO, the WPILib version, and the vendordeps are three separate pins and they have to move together. When you bump GradleRIO in `build.gradle`, you also need to re-run `Manage Vendor Libraries → Check for Updates` in WPILib VSCode so the vendordep line up. Mixing a new WPILib with old vendordeps produces compile errors that look nothing like a version problem.

## When things go wrong

The first instinct, more often than not, is `./gradlew clean build`. If that doesn't fix it:

* **"invalid source release" or any message that names a Java version.** Your active JDK is not 17. Run `java -version` to confirm, then switch with SDKMAN (`sdk list java | grep tem`, then `sdk use java <latest-17.x.x-tem>`) or your IDE's Java runtime settings. See [Setup](../setup.md).

* **Spotless complaining about format violations.** Run `./gradlew spotlessApply` and commit the result. CI runs `spotlessCheck`, which does not write. Note that a build reformatting your files in place is normal, not a failure; if the first build fails on formatting, run it again.

* **`BuildConstants.java` shows up in `git status` after a build.** It is generated. Do not edit it and do not commit it. See [Build Tools](build-tools.md).

* **A vendor jar failing to download.** Usually a transient network thing. `--refresh-dependencies` retries. If that doesn't work, delete `~/.gradle/caches/modules-2/files-2.1/<vendor>` and try again.

* **Deploy works for you but fails for a teammate.** Check that you are both on the same wrapper version with `./gradlew --version`. A globally installed Gradle will sometimes mask the wrapper.

* **The roboRIO is not found.** `./gradlew deploy` needs the roboRIO on the same network or plugged in by USB. The team number comes from `.wpilib/wpilib_preferences.json`; if the robot is on a rival's number the deploy goes to the wrong place or nowhere.

## See also

[Build Tools and Other Development Utilities](build-tools.md) for the formatter, static analyzer, and annotation processor. [Setup](../setup.md) for one-time environment work.
