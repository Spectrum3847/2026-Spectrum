# Build tools and other development utilities

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

The non-Gradle tooling that runs as part of a build: the formatter, the static analyzer, the annotation processor, and the JavaDoc generator. Gradle itself gets its own page; see [Gradle](gradle.md). Their configuration is all in `build.gradle` and `excludeFilter-spotbugs.xml`, so this page covers what each one will do to you and when it fails rather than what it is configured with.

## Spotless

[Spotless](https://github.com/diffplug/spotless) keeps the codebase formatted to a single style so nobody's PR is half-diff because of whitespace. It is wired to `compileJava`, so every local build rewrites your files in place. That is normal. The style is Google Java Format's AOSP variant, which is 4-space indent and covers `.java`, `.gradle`, `.xml` and `.md`, so a build can also rewrite this very page.

CI runs `spotlessCheck`, which is read-only. If it fires, run `./gradlew spotlessApply` locally, commit, and push again.

To keep a region as you wrote it, wrap it in `// spotless:off` and `// spotless:on`. Those markers are honored, so a hand-aligned matrix or a generated table survives a build.

## SpotBugs

[SpotBugs](https://spotbugs.github.io/) is the static analyzer. It runs as part of `./gradlew build` through the `spotbugsMain` task and writes an HTML report to `build/reports/spotbugs.html`. Open that report locally when CI goes red; the report is far more readable than the console output.

A finding fails the build. That is intentional, and it means a red build after a pull is not something to work around. Try to fix the finding. If a fix is genuinely impractical, add a narrowly scoped `<Match>` block to `excludeFilter-spotbugs.xml` with a comment saying why, rather than widening an existing rule. The existing exclusions are broad categories, added because the classes of warning they suppress are not ones this codebase can act on: the whole `PERFORMANCE` category, the mutable-array-exposure patterns that WPILib's own APIs trigger everywhere, and the third-party Limelight glue code we do not own.

## Project Lombok

[Lombok](https://projectlombok.org) generates getters, setters and value-object constructors at compile time. It is wired in through the `io.freefair.lombok` Gradle plugin, so there is no annotation-processor setup to do by hand. The conventions and the gotchas are on their own page: [Project Lombok](../coding-conventions/project-lombok.md). Read it before adding a new annotation.

The thing that wastes the most time: generated methods exist only at compile time, so if your IDE cannot see them it is missing the Lombok plugin, not looking at broken code. That is the first thing to check when `@Getter` appears to do nothing.

## BuildConstants

`src/main/java/frc/robot/BuildConstants.java` is generated on every `compileJava` and must not be edited or committed. It bakes the build timestamp, git branch, commit and dirty flag into the jar, which lets a log or a Driver Station screen identify exactly which build is running. `Robot.java` reads it at init and publishes the values to NetworkTables under `BuildConstants/*`; there is no Elastic widget for them, so read them from AdvantageScope or the log. Seeing it appear in `git status` after a build is expected.

## JavaDoc

`./gradlew javadoc` writes HTML to `build/docs/javadoc/`. Two things to know. The `javadoc` block in `build.gradle` sets `Xdoclint:none` and `failOnError = false`, so a missing or malformed JavaDoc never fails a build. That is a deliberate trade-off, we prefer encouragement over enforcement, which means the JavaDoc is only as good as the person who wrote it.

The same block configures external link bases, so a reference to a class from a vendored library turns into a link to that library's hosted docs. If you add a new vendor library, add its base URL there or every cross-link into it silently degrades to plain text.

## VSCode extensions

The bare minimum for productive Java work in this repo:

* **Language Support for Java by Red Hat** (`redhat.java`), the language server, for IntelliSense and navigation. If Java looks broken in a way the build disagrees with, run `Java: Clean Java Language Server Workspace` from the Command Palette first. A stale language server cache is the usual cause and the clean command fixes it.
* **Lombok Annotations Support**, without which the IDE cannot see the generated members.
* **Error Lens** (`usernamehw.errorlens`), for compile errors and warnings inline.
* **GitLens** (`eamodio.gitlens`), for `git blame` on every line. Worth it for archaeology.
* **GitHub Pull Requests and Issues** (`github.vscode-pull-request-github`), for review without leaving the editor.

Optional but useful:

* **Git Config User Profiles**, for shared laptops or pair programming, lets you swap committer identity without `git config` gymnastics.
* **Live Share**, real-time pair programming. We do not reach for it often, but when a mentor is helping debug remotely it is the fastest path.
* **Open in Browser**, one click for the SpotBugs HTML report.

Skip GitHub Copilot for code completion if you are trying to learn this codebase's patterns. It will confidently autocomplete the wrong subsystem-wiring convention. It is fine for tests and boilerplate.

## See also

[Setup](../setup.md) for one-time environment work, [Gradle](gradle.md) for the build commands themselves, and [Project Lombok](../coding-conventions/project-lombok.md) for annotation conventions.
