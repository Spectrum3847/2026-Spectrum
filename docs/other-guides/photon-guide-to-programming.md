# Photon Guide to Programming

*Audience: Reference. No prerequisites.*

This is less a tutorial and more a mindset guide — the thinking patterns that make programming on a competition robot less frustrating and more effective.

## Programming is Empirical

You're not going to read your way to a working robot. At some point you have to run code, watch what happens, decide what it tells you, and change something. The teams that win programming are the ones who can cycle through that loop fast — not the ones who planned longest before touching a keyboard.

That said, a few minutes of design before writing saves hours of debugging. If you can't explain in one sentence what a piece of code is supposed to do, you probably aren't ready to write it yet.

## Draw Before You Code

If a system isn't clicking, draw it. A state machine for the intake sequence on a whiteboard is faster to get right than a `Coordinator` method you'll rewrite three times. Flowcharts, block diagrams, rough sketches — whatever gets the structure out of your head and onto paper so you can see if it makes sense.

The [`Coordinator.java`](../../src/main/java/frc/robot/Coordinator.java) class in this repo coordinates multi-subsystem behaviors by picking a `State` enum value and dispatching to each subsystem. Drawing the state graph before building it in code — which states transition to which, what triggers each transition — makes the code almost write itself.

## Shadow a Programmer

Watching someone who knows the codebase navigate it is one of the fastest ways to learn it. You'll pick up keyboard shortcuts, patterns for where things live, and how to debug CAN issues in about half the time it would take to discover them yourself.

When you're the one being shadowed: explain what you're doing out loud as you do it. The "rubber duck" effect is real — articulating your logic to a person (or a rubber duck on your desk) forces precision that silent coding doesn't.

## Read the Existing Code Before Writing New Code

Before adding a new mechanism, look at how an existing one is built. The [`Launcher`](../../src/main/java/frc/robot/launcher/Launcher.java) / [`LauncherStates`](../../src/main/java/frc/robot/launcher/LauncherStates.java) pair is a good template. You'll see the three-file layout (subsystem, config as inner class, states class), how gain slots are set up, how `DoubleSupplier` is used for setpoints, and how `scheduleIfNotRunning` guards command factories from being scheduled over each other.

The coding conventions in [Class Generation](../coding-conventions/class-generation.md) are based on that same pattern. Read that page before building something new — it explains the *why*, not just the *what*.

## Start in Simulation

Almost everything testable is testable in simulation before the robot is built. State logic, PathPlanner paths, subsystem interactions — simulation catches the category of bugs that would otherwise eat real-robot time. See [Simulation](../tools/simulation.md) for how to launch and what you get.

The corollary: if you can't tell whether your code works without a robot, that's usually a sign the code is harder to reason about than it needs to be.

## The 5-Second Rule

When something breaks, resist the impulse to start changing things. Five seconds of thinking about what the failure tells you is worth more than five minutes of random edits. What exactly happened? What did you expect? What's the smallest change that would distinguish between your hypotheses?

This is especially true on competition day, when the instinct is to panic-edit. One targeted change you can verify beats three simultaneous changes that leave you with no idea what fixed it.

## Mistakes Are Useful

When something doesn't work, that's data. A CAN error, a command that ends immediately, a mechanism that overshoots — each of those tells you something specific if you look at the logs. See [Logging](../tools/logging.md) for how to pull information out of the `.wpilog` files and AdvantageScope.

Don't delete broken code immediately. Comment it out, note what it did, and understand why it failed before replacing it. The codebase's commit history is also a record of what's been tried — `git log` with a file path is useful when you want to know why something was written the way it was.

## Ask Early

If you're stuck for more than about 20 minutes, ask. Ask a teammate, a mentor, or post to Chief Delphi. The FRC programming community is unusually open — teams share code, mechanisms, and hard-won tuning data. The answers to most questions you'll have are already in someone's documentation or a CD thread.

A few starting points:

- [Chief Delphi](https://www.chiefdelphi.com) — the main FRC forum. Search before posting; many questions have been asked before.
- [WPILib docs](https://docs.wpilib.org) — covers subsystems, commands, the scheduler, and simulation in depth.
- [CTRE Phoenix 6 docs](https://v6.docs.ctr-electronics.com) — TalonFX gain configuration, closed-loop control modes, and fault handling.
- The `#programming` channel in the team Slack — the fastest way to reach teammates who've already solved the problem.

## Resources Used on This Robot

The [Dependencies Overview](../dependencies/overview.md) lists every library the 2026 robot uses. The ones you'll interact with most directly:

- **WPILib** — commands, subsystems, simulation, and the scheduler.
- **Phoenix 6** — TalonFX control, MotionMagic, gain configuration, and CAN diagnostics via [Phoenix Tuner X](../tools/phoenix-tuner-x.md).
- **DogLog / Telemetry** — logging, covered in [Logging](../tools/logging.md).
- **PathPlanner** — autonomous path generation and following, covered in [Auton](../tools/auton.md).
- **MapleSim** — physics simulation for the drivetrain and game pieces, covered in [Simulation](../tools/simulation.md).
