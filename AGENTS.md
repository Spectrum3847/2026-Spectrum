# Agent Instructions

FRC Team Spectrum 3847 robot code for the 2026 season (REBUILT game): **Java 17**, GradleRIO 2026.2.1, WPILib 2026. (Verified 2026-08-03: `gradle/wrapper/gradle-wrapper.properties` → Gradle 8.x; `build.gradle` → `wpilibVersion = "2026.x"`.)
Swerve drive with fuel launcher, turret, indexer, intake, vision, LEDs, and climb. (Verified 2026-08-03: subsystem list matches `src/main/java/frc/robot/subsystems`.)

## Documentation (read this first)

`docs/` is the source of truth for this codebase; read the relevant page instead of guessing, and update the docs when your changes affect behavior. (Verified 2026-08-03: `docs/index.md` exists and cross-links the pages below.)

- `docs/index.md`: full doc map
- `docs/other-guides/2026-season-specific.md`: what's in the codebase: subsystems, state machine, controls, per-robot configs
- `docs/tools/*.md`: build tools, gradle commands, auton, vision, simulation, logging, PID tuning, Elastic, Phoenix Tuner X
- `docs/dependencies/*.md`: how we use Phoenix 6, PathPlanner, DogLog, MapleSim, PhotonVision, WPILib
- `docs/coding-conventions/*.md`: code style (AOSP), class generation, exceptions, Lombok, git workflow
- `README.md`: repository structure tree

## Build & Development

- **Java 17** is required. If it's not the default JDK, use sdkman; fallback: download a Temurin 17 archive to a temp directory, extract it there, and point `JAVA_HOME` at the extracted JDK (or prepend its `bin` directory to `PATH`) before invoking `./gradlew`. Source: Eclipse Temurin releases at <https://adoptium.net/temurin/releases/> (recorded 2026-08-02). Full setup: `docs/setup.md`.
- `./gradlew build`: compile + Spotless auto-format + tests + SpotBugs. Also: `./gradlew simulateJava`, `./gradlew deploy`, `./gradlew test`, `./gradlew spotlessApply`.
- **Spotless auto-formats on every build** (Google Java Format **AOSP**, 4-space indent, LF line endings; covers `.java`, `.gradle`, `.xml`, `.md`, `.gitignore`). Re-run `./gradlew build` if the first pass fails on formatting. CI runs `spotlessCheck`.
- Known issues: `src/main/java/frc/robot/BuildConstants.java` is auto-generated (never edit or commit it); SpotBugs report at `build/reports/spotbugs.html` (exclude filter `excludeFilter-spotbugs.xml`); ErrorProne may elevate warnings to errors.
- **WPILib's `Trigger` is vendored** (patched copy at `src/main/java/edu/wpi/first/wpilibj2/command/button/Trigger.java`, default start condition `false`). Do not replace with the upstream version.

## Architecture in Brief

- **State machine**: `State.java` (17 states) → `RobotStates.java` (Triggers) → `Coordinator.java` (state → subsystem command mappings). Edit `Coordinator.java` for new behaviors. (A new state-machine architecture is under test on branch `2026-offseason-bot`.)
- **Subsystems**: `SubsystemName.java` + `SubsystemNameStates.java`; motor-based ones extend `frc.spectrumLib.Mechanism` and implement `frc.spectrumLib.SpectrumSubsystem`; hardware config lives in each subsystem's inner `Config` class.
- **Robot configs**: `FM2026` / `XM2026` / `PM2026` / `AM2026` / `PHOTON2026` in `src/main/java/frc/robot/configs/`, auto-selected by RoboRIO serial via `frc.spectrumLib.Rio`. CAN IDs and encoder offsets go here, not in subsystem files.
- **Autos**: PathPlanner (paths/autos in `src/main/deploy/pathplanner/`); named commands registered in `Auton.java`.
- **Telemetry**: DogLog via `frc.spectrumLib.telemetry.Telemetry`; `TuneValue` for live tuning.
- **Gamepads**: `Pilot.java` / `Operator.java`; bindings in `PilotStates.java` / `OperatorStates.java`.

## Git, Commits, and Pull Requests

Full workflow: `docs/coding-conventions/commits-pull-requests.md`. How to write these rules into a new robot repo's `AGENTS.md`: `docs/other-guides/agents-md-guidelines.md`. The three rules that govern what an agent session does, rather than what it writes:

- **Ask who is committing, once per session.** Team computers are shared by several students a day, so the machine's `git config user.name` is not to be trusted. Before the *first* commit of a session, read `.claude/committers.json`, ask the person at the keyboard which entry is theirs, and pass the identity inline: `git -c user.name="..." -c user.email="..." commit -m "..."`. Reuse that answer for the rest of the session rather than asking again; a new session asks again from scratch. Never set `git config user.name`/`user.email` (that changes the machine for the next student), and never commit under an agent or bot identity. `.claude/hooks/require-committer.sh` enforces this and will deny a commit that skips it.
- **No agent attribution.** No `Co-Authored-By` trailer, no "generated with" line, no session URL, in commits or PR descriptions. Commits belong to the student's GitHub account. `.claude/settings.json` turns this off for Claude Code; check the equivalent setting when adding another agent.
- **One feature per PR.** One branch, one feature, one PR, named for the feature, so a single `git revert` backs it out cleanly at an event. An unrelated fix found mid-branch gets its own branch off `main`, even a one-liner. Refactors ship separately from behavior changes. Don't open a PR unless a human asked for one.

## Skills Policy

`./.agents/skills/` holds agent skills for this team. Keep them tailored to **Spectrum 3847** (team 3847, IP `10.38.47.2`) and to our actual stack: DogLog (via `frc.spectrumLib.telemetry.Telemetry`), MapleSim (`MapleSimSwerveDrivetrain`), `FuelPhysicsSim`, PhotonVision, PathPlanner, CTRE Phoenix 6. We do **not** use AdvantageKit, so: (Aligns to Spectrum 3847 stack; PR #132, reviewed 2026-08-07.)
- New skills must describe our real classes/topics, never generic donor code. Verify names against `src/main/java` and `docs/`.
- To keep a skill from loading across all agents (opencode, Claude Code, etc.), rename its `SKILL.md` → `SKILL.md.disabled` instead of editing frontmatter. Leave the directory's other files in place.

## Important Notes

1. Run `./gradlew build` after any Java change; it auto-formats and runs SpotBugs/tests. Re-run if it fails on formatting.
2. Never edit `BuildConstants.java`.
3. New subsystem → register in `Robot.java` + `Coordinator.java`. New state → `State.java` + `Coordinator.java` + `RobotStates.java`. New paths/autos → `src/main/deploy/pathplanner/` + register in `Auton.java`. Hardware config → the `*2026.java` config class.
4. Line endings must be **LF (UNIX)**; `.gitattributes` enforces `eol=lf`.
5. When you learn new repo facts, append a concise note here (source + date), and keep `docs/` updated too. No secrets or credentials.
