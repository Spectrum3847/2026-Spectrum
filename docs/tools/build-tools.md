# Build Tools and Other Development Utilities

*Audience: Reference. Assumes you've read [Setup](../setup.md).*

A walkthrough of the non-Gradle tooling that ships in this repo. Gradle itself gets its own page — see [Gradle](gradle.md). Everything else lives here: the formatter, the static analyzer, the annotation processor, and the VSCode extensions worth installing on day one.

## Spotless

[Spotless](https://github.com/diffplug/spotless) keeps the codebase formatted to a single style so nobody's PR is half-diff because of whitespace. The configuration sits at the bottom of [`build.gradle`](../../build.gradle):

* **Java** → `googleJavaFormat("1.15.0").aosp()` plus `removeUnusedImports`, `trimTrailingWhitespace`, `endWithNewline`. The AOSP variant uses 4-space indents, which matches what WPILib examples use.
* **Gradle**, **XML**, and **Markdown** all get their own formatters too — that's why a `./gradlew build` will sometimes rewrite this very file.

`compileJava` depends on `spotlessApply`, so every local build reformats your code. CI runs `spotlessCheck` instead, which is read-only — if it fires, run `./gradlew spotlessApply` locally, commit, and push again.

To opt a region out (a hand-aligned matrix, a generated table), wrap it in `// spotless:off` / `// spotless:on`. `toggleOffOn()` is enabled, so those markers are honored.

## SpotBugs

[SpotBugs](https://spotbugs.github.io/) is the static analyzer. It runs as part of `./gradlew build` (via `spotbugsMain`) and writes an HTML report to `build/reports/spotbugs.html`. The repo is configured with `Effort.DEFAULT` and the highest confidence threshold — only findings SpotBugs is fairly sure about make it into the report.

The exclude list lives in [`excludeFilter-spotbugs.xml`](../../excludeFilter-spotbugs.xml). Notable carve-outs:

* The entire `PERFORMANCE` category is suppressed — micro-perf is rarely the right thing to chase on robot code.
* `EI_EXPOSE_REP` / `EI_EXPOSE_REP2` (returning mutable internal arrays) are suppressed — WPILib APIs leak mutable buffers everywhere, and the warnings drown out signal.
* `LimelightHelpers` and `RobotLogger` are excluded wholesale because they're third-party / glue code we don't own.

If you legitimately want to silence a finding, prefer fixing it. If a fix is impractical, add a narrowly-scoped `<Match>` block to the exclude file with a comment explaining why, rather than disabling the rule everywhere.

`ignoreFailures = false` — a SpotBugs finding fails the build. That's intentional. If CI breaks on a SpotBugs report you can't explain, open `build/reports/spotbugs.html` locally; the "fancy-hist" stylesheet makes it readable.

## Lombok

[Project Lombok](https://projectlombok.org) is wired in through `io.freefair.lombok` (a Gradle plugin) so there's no extra annotation-processor setup. We use a small handful of annotations across the codebase:

* `@Getter` / `@Setter` on `*Config` inner classes so the chainable per-robot overrides don't need hand-written boilerplate.
* `@Accessors(chain = true)` to make setters return `this`, so configs read like builders.
* `@RequiredArgsConstructor` here and there for value-object constructors.

There's a dedicated page on conventions and gotchas: [Project Lombok](../coding-conventions/project-lombok.md). Read it before adding new annotations — a few combinations (`@Builder` + inheritance, `@Data` + JPA-style equals) cause subtle bugs.

The big thing to remember: generated methods only exist at compile time. If your IDE doesn't see them, it doesn't have the Lombok plugin installed. For VSCode that's `Project Lombok Extension Pack` (`pleiades.java-extension-pack-jdk`-style bundles also include it).

## gversion / `BuildConstants`

The `com.peterabeles.gversion` plugin regenerates [`src/main/java/frc/robot/BuildConstants.java`](../../src/main/java/frc/robot/BuildConstants.java) on every `compileJava`. It bakes in:

* Build timestamp (Eastern Time).
* Git branch, commit, and dirty flag.
* Maven/Gradle project version.

[`Robot.java`](../../src/main/java/frc/robot/Robot.java) reads `BuildConstants` directly in its constructor and publishes git/build info to NetworkTables — that's what the **Git Status** tab in [Elastic](elastic.md) renders. Don't hand-edit `BuildConstants.java`; it gets overwritten next build.

## JavaDoc

`./gradlew javadoc` writes HTML to `build/docs/javadoc/`. The interesting bit is the cross-link configuration in `build.gradle`'s `javadoc.options.setLinks(...)`:

```
WPILib, REV, Phoenix v5, Phoenix 6, PathPlanner, DogLog, MapleSim, PhotonVision, Java 17
```

When our JavaDoc references a class from any of those, the generated HTML links straight to the vendor's hosted docs. If you add a new vendor library, add its JavaDoc base URL here so cross-links keep working.

`Xdoclint:none` is set, so missing JavaDocs don't fail the build. That's a deliberate trade-off — we prefer encouragement over enforcement.

## VSCode Extensions

The bare minimum for productive Java work in this repo:

* **Language Support for Java™ by Red Hat** (`redhat.java`) — language server, IntelliSense, navigation. If Java gets weirdly broken, the `Java: Clean Java Language Server Workspace` command is the first thing to try.
* **Lombok Annotations Support** — without this the IDE doesn't see `@Getter`/`@Setter` and your code looks broken even though it builds.
* **Error Lens** (`usernamehw.errorlens`) — surfaces compile errors and warnings inline.
* **GitLens** (`eamodio.gitlens`) — `git blame` annotations on every line. Worth it for archaeology.
* **GitHub Pull Requests and Issues** (`github.vscode-pull-request-github`) — review and comment on PRs without leaving the editor.

Optional but useful:

* **Git Config User Profiles** — for shared laptops or pair programming, lets you swap committer identity without `git config` gymnastics.
* **Live Share** — real-time pair programming. We don't reach for it often, but when a mentor is helping debug remotely it's the fastest path.
* **Open in Browser** — one-click for the SpotBugs HTML report.

Skip GitHub Copilot for code completion if you're trying to learn the patterns in this codebase — it'll happily autocomplete the wrong subsystem-wiring convention. Use it for tests or boilerplate where the patterns are obvious.

## See Also

[Setup](../setup.md) for one-time environment work, [Gradle](gradle.md) for the build commands themselves, and [Project Lombok](../coding-conventions/project-lombok.md) for annotation conventions.
