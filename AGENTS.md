# Agent Instructions

FRC Team Spectrum 3847 robot code for the 2026 season (REBUILT game): **Java 17**, GradleRIO 2026.2.1, WPILib 2026. (Verified 2026-09-25: `gradle/wrapper/gradle-wrapper.properties` → Gradle 9.7.1; `build.gradle` → GradleRIO `2026.2.1`, Java 17.)
Swerve drive with fuel intake + intake extension, dye rotor (indexer), launcher + launcher tower, hood, turret and vision. LEDs exist (`subsystems/leds/Leds.java`) but are commented out; there is no climb. (Verified 2026-09-25 against `src/main/java/frc/robot/subsystems`.)

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
- Known issues: `src/main/java/frc/robot/BuildConstants.java` is auto-generated (never edit or commit it); SpotBugs report at `build/reports/spotbugs.html` (exclude filter `excludeFilter-spotbugs.xml`); ErrorProne may elevate warnings to errors. **Open (2026-09-19):** after a roboRIO restart with the CANivore bus dead, CPU pegs at 95-100% exactly 30 s after robot init and stays there; not the robot loop. Repro and suspects in the `CanConfigBudget` class javadoc; log `logs/matches/FRC_20260919_034444.wpilog`.
- **WPILib's `Trigger` is not vendored**: the stock WPILib `Trigger` is used (no copy under `src/main/java/edu/`). (Verified 2026-09-25.)

## Architecture in Brief

- **State machine**: `SuperStructure.java` maps a `WantedSuperState` to a `CurrentSuperState` each loop (`handleStateTransitions()`) and sets every subsystem's wanted state from it. Add new robot behaviors there. Teleop states reuse their auton twins plus `teleopDrive(...)`.
- **Subsystems**: one file each under `frc.robot.subsystems.<name>`, with its own `WantedState`/`SystemState` enums; motor-based ones extend `frc.spectrumLib.mechanism.Mechanism` and log through `logStandard(...)`. Hardware config lives in each subsystem's inner `Config` class (IntakeExtension's two sides share `Axis`/`AxisConfig.left()`/`right()`).
- **Robot configs**: `OM2026` is what the robot runs: `Robot.java` always constructs it whatever `frc.spectrumLib.hardware.Rio.id` says. `FM2026` / `XM2026` / `PM2026` / `AM2026` / `PHOTON2026` also exist in `src/main/java/frc/robot/configs/` but are not selected. CAN IDs and encoder offsets go in the config, not in subsystem files.
- **Autos**: PathPlanner (paths/autos in `src/main/deploy/pathplanner/`). `Auton.java` builds the chooser from `.auto` files with `routine(...)`/`single(...)`; there are no `NamedCommands`: path event markers fire `EventTrigger`s declared in `Auton.java` and bound in `Robot.configureBindings()`.
- **Telemetry**: DogLog via `frc.spectrumLib.telemetry.Telemetry`; `Telemetry.logState(...)` for state enums (written on change); `Telemetry.tunable(...)` for live tuning (`TuneValue` is unused).
- **Gamepads**: triggers are defined in `Pilot.java` / `Operator.java`; bindings live in `Robot.configureBindings()` (and `configureSimBindings()`). Keep `tools/robot-app/data/controls.json` in step; `node tools/robot-app/scripts/check-drift.mjs` reports drift.

## Skills Policy

`./.agents/skills/` holds agent skills for this team. Keep them tailored to **Spectrum 3847** (team 3847, IP `10.38.47.2`) and to our actual stack: DogLog (via `frc.spectrumLib.telemetry.Telemetry`), MapleSim (`MapleSimSwerveDrivetrain`), `FuelPhysicsSim`, PhotonVision, PathPlanner, CTRE Phoenix 6. We do **not** use AdvantageKit, so: (Aligns to Spectrum 3847 stack; PR #132, reviewed 2026-08-07.)
- New skills must describe our real classes/topics, never generic donor code. Verify names against `src/main/java` and `docs/`.
- To keep a skill from loading across all agents (opencode, Claude Code, etc.), rename its `SKILL.md` → `SKILL.md.disabled` instead of editing frontmatter. Leave the directory's other files in place.
- **Log problems between matches**: run `.agents/skills/fast-log-triage/scripts/triage_wpilog.py <log>` first (pure Python, seconds, ranked suspects), then go deeper with `wpilog-decode` only on the top finding. (Added 2026-09-19; reader cross-checked against WPILib `DataLogReader`.)
- **Sync a branch with GitHub**: run `bash .agents/skills/git-sync-upstream/scripts/sync_upstream.sh` (fetch, fast-forward, merge `origin/main`, autostash, conflict list, summary; never pushes). `--dry-run` shows what would come in. (Added 2026-09-19.)

## Important Notes

1. Run `./gradlew build` after any Java change; it auto-formats and runs SpotBugs/tests. Re-run if it fails on formatting.
2. Never edit `BuildConstants.java`.
3. New subsystem → construct it in `Robot.java` and pass it to `SuperStructure`. New state → add it to `WantedSuperState`/`CurrentSuperState` and `handleStateTransitions()`/`applyStates()` in `SuperStructure.java`. New paths/autos → `src/main/deploy/pathplanner/` + a chooser entry in `Auton.setupSelectors()`. Hardware config → the `*2026.java` config class (`OM2026` on the robot).
4. Line endings must be **LF (UNIX)**; `.gitattributes` enforces `eol=lf`.
5. When you learn new repo facts, append a concise note here (source + date), and keep `docs/` updated too. No secrets or credentials.
6. Auto names in `Auton.java` must match the `.auto` file names **exactly, including case**: the rio's filesystem is case-sensitive and the Windows sim is not, so a mismatch only fails on the robot (Chezy QM4, 2026-09-19: "OSCENT Full" vs `OSCENT FULL.auto`, auto sat still). `Auton.verifyAutoFile` alerts at boot and `Robot.logAutoSelection` logs `Auton/SelectedAuto` and `Auton/AutoFileFound` to the wpilog.
7. Real match logs are committed in `logs/matches/` (the rest of `logs/` is gitignored). Add one with `python tools/copy-match-logs.py`, describe it in `logs/matches/README.md`, and keep them under 50 MB each. The robot app's `npm test` and the fast-log-triage script both run against them. (Added 2026-09-19.)
