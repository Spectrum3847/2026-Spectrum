# Agent Instructions

FRC Team Spectrum 3847 robot code for the 2026 season (REBUILT game), written in
Rust. Swerve drive with a fuel launcher, hood, intake, indexers, vision and LEDs.
The Cargo workspace is in `rust/`. The Java version it was ported from is the
parent of the commit `feat: port the robot program to Rust`.

## Direction

- The robot must behave like the Java robot did. Change behavior on purpose, in
  its own commit, never as a side effect of a refactor.
- Small memory and CPU use: the program runs on a roboRIO 2.
- Pure Rust where possible. Native code only through the WPILib and CTRE C APIs,
  loaded at runtime.
- Every calculation has a test. Ported math is checked against frozen Java
  outputs in `rust/fixtures/`.
- Port or add only what the robot uses.

## Glossary

- **Natives**: WPILib 2026.2.1 and CTRE Phoenix 6 26.3.0 shared libraries.
  `cargo xtask fetch-natives` puts them in `rust/native/<platform>/` (gitignored).
- **Golden fixture**: JSON in `rust/fixtures/` with inputs and the Java outputs
  for them. Frozen: the Java generator is gone. `golden::check` compares them.
- **SPN**: CTRE signal or config parameter number, used by the CTRE C API.
- **Context item**: a robot-wide object (`swerve`, `super_structure` and others)
  set once at init in `crates/robot/src/context.rs` and read through a function.
- **`Java:` comment**: marks behavior kept from the Java robot that looks wrong.
  Fix each in its own change.

## Layout

| Crate (`rust/crates/`) | What |
|---|---|
| `wpilib` | HAL, driver station, timed robot loop, NetworkTables, dashboards (SmartDashboard, chooser, Field2d, Mechanism2d, Alert), WebServer, shutdown hooks |
| `commands` | Command-based framework: scheduler, subsystems, triggers, SysIdRoutine |
| `wpimath` | Geometry, kinematics, pose estimator, PID, DCMotor and mechanism sims |
| `phoenix6` | TalonFX, CANcoder, Pigeon2, CANdle over CTRE's C layer |
| `swerve` | Swerve drivetrain with CTRE's API and behavior |
| `pathplanner` | PathPlannerLib port: paths, autos, follower, pathfinding |
| `telemetry` | wpilog writer and DogLog-style logging |
| `robot` | The robot program |
| `golden` | Test helper for fixtures |

`rust/xtask` holds the build, deploy and sim tasks. `src/main/deploy/` holds
PathPlanner paths and autos, the navgrid and the Elastic layout; the robot reads
them from there in sim, and `cargo xtask deploy` copies them to the roboRIO.

Inside `crates/robot/src`:

- `robot/robot.rs`: init order, config selection, controller bindings, mode
  methods. Config by roboRIO serial (`spectrum_lib/hardware/rio.rs`):
  `PHOTON2026`, `PM2026`, else `FM2026`. Configs set swerve encoder offsets and
  which mechanisms are attached. CAN IDs live in each subsystem's config struct.
- `robot/subsystems/super_structure.rs`: the robot state machine
  (`WantedSuperState`). Mechanisms are `Mechanism` subsystems in
  `robot/subsystems/`.
- `robot/auton/`: auto chooser and PathPlanner event triggers (`intake`,
  `shotPrep`, `clearState`, `unjam`, `poseUpdate`).
- `rebuilt/`: game logic (field, hub shift clock, shot calculator) and the fuel
  and bump sims. `spectrum_lib/`: shared code (Mechanism, gamepads, LEDs,
  Limelight vision, mechanism sims, maple-sim swerve sim).

## Build and run

Rust toolchain is pinned in `rust/rust-toolchain.toml`. From `rust/`:

- `cargo xtask gate`: rustfmt check, clippy with warnings as errors, all tests. Run it
  before every commit.
- `cargo xtask fetch-natives [platform]`: download natives (host default;
  `linuxathena` is the roboRIO).
- `cargo xtask sim`: run the robot in simulation. `cargo xtask sim <example> -p <crate>`
  runs an example.
- `cargo xtask build-roborio`, then `cargo xtask deploy [host]` (default
  `10.38.47.2`, USB `172.22.11.2`). The roboRIO `admin` password is empty.
- Unverified: `deploy` has not run on a real roboRIO yet, and `xtask` has not
  run on Windows or macOS yet (their code paths are unit-tested only).

## Rules for the code

- **Threads**: robot setup, commands, the scheduler and dashboards run on the
  loop thread only and use `Rc`/`RefCell`. Code on other threads (swerve
  odometry, notifiers, telemetry) uses only `Send + Sync` APIs: telemetry,
  Phoenix signals, atomics, `Mutex`. Dashboard calls from another thread panic.
- **Borrows**: subsystems are `Rc<T>` with `&self` methods. Hold a `RefCell`
  borrow only inside one method body, never across a call into another
  subsystem, the scheduler, a command or a callback.
- **Units**: library crates use SI `f64`, with rotations only at the Phoenix
  boundary (`*_rot`, `*_rps`). Robot code keeps the units the Java used (RPM,
  rotations, degrees), with the unit in the name.
- **Native calls**: every `extern` signature matches the C header exactly.
  `#[repr(C)]` structs carry size checks for 64-bit and the 32-bit roboRIO.
  Wrap each unsafe call with a `// SAFETY:` comment. Register a stop function
  with `wpilib::shutdown` for any thread or notifier that touches natives.
- **CTRE**: the C layer is not a documented CTRE API, so keep the version
  pinned. After changing the generated tables in `crates/phoenix6`, run
  `python3 tools/check-ctre-tables.py`. Never disassemble CTRE binaries
  (CTRE's license forbids it); learn behavior from headers, Java sources, docs
  or by measuring.
- **Swerve facts measured from CTRE's sim** (CTRE ships this code compiled
  only): the coupling correction low-pass filters steer velocity with a 50 ms
  time constant (measured at 250 Hz only); wheel force feedforward is projected
  on the measured steer angle; CANcoder position and velocity frames stay at
  100 Hz after bus optimization, because the steer Talons' fused feedback reads
  them.
- **Licenses**: ported code keeps its license (WPILib BSD-3 in
  `WPILib-License.md`). List every ported source in `rust/THIRD_PARTY.md`.
- Short comments that explain why. No emojis.

## Git, Commits, and Pull Requests

Full workflow: `docs/coding-conventions/commits-pull-requests.md`. How to write these rules into a new robot repo's `AGENTS.md`: `docs/other-guides/agents-md-guidelines.md`. The three rules that govern what an agent session does, rather than what it writes:

- **Ask who is committing, once per session.** Team computers are shared by several students a day, so the machine's `git config user.name` is not to be trusted. Before the *first* commit of a session, read `.claude/committers.json`, ask the person at the keyboard which entry is theirs, and pass the identity inline: `git -c user.name="..." -c user.email="..." commit -m "..."`. Reuse that answer for the rest of the session rather than asking again; a new session asks again from scratch. Never set `git config user.name`/`user.email` (that changes the machine for the next student), and never commit under an agent or bot identity. `.claude/hooks/require-committer.sh` enforces this and will deny a commit that skips it.
- **No agent attribution.** No `Co-Authored-By` trailer, no "generated with" line, no session URL, in commits or PR descriptions. Commits belong to the student's GitHub account. `.claude/settings.json` turns this off for Claude Code; check the equivalent setting when adding another agent.
- **One feature per PR.** One branch, one feature, one PR, named for the feature, so a single `git revert` backs it out cleanly at an event. An unrelated fix found mid-branch gets its own branch off `main`, even a one-liner. Refactors ship separately from behavior changes. Don't open a PR unless a human asked for one.

## Skills Policy

`./.agents/skills/` holds agent skills for this team. Keep them tailored to **Spectrum 3847** (team 3847, IP `10.38.47.2`) and to our actual stack: the Rust crates above, Limelight vision, PathPlanner, CTRE Phoenix 6, DogLog-style logs. We do **not** use AdvantageKit or PhotonVision.
- New skills must describe our real code and topics, never generic donor code. Verify names against `rust/crates/` and `docs/`.
- To keep a skill from loading across all agents (opencode, Claude Code, etc.), rename its `SKILL.md` → `SKILL.md.disabled` instead of editing frontmatter. Leave the directory's other files in place.

## Documentation

`docs/index.md` maps the docs. Update the page that covers your change.
`docs/frc-software-basics/` is Java background material from before the Rust port.

## Important Notes

1. Run `cargo xtask gate` from `rust/` after any change.
2. New subsystem: build it in `robot/robot.rs` init and set a context item if
   others read it. New robot state: `WantedSuperState` in `super_structure.rs`.
   New paths or autos: `src/main/deploy/pathplanner/`, registered in
   `robot/auton/`.
3. Line endings must be LF; `.gitattributes` enforces it.
4. When you learn a new repo fact, add a short note here (what and why) and
   update `docs/`. No secrets or credentials.
